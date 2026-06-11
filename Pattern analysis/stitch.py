import sys
import math
from kipy import KiCad
from kipy.board_types import Via, ViaType
from kipy.geometry import Vector2
from kipy.util import from_mm

# Required in newer versions of kipy to specify what items to fetch
from kipy.proto.common.types.enums_pb2 import KiCadObjectType

def stitch_grounds_ipc(start_x_mm, start_y_mm, end_x_mm, end_y_mm,
                       spacing_x_mm, spacing_y_mm, stagger=True,
                       via_size_mm=0.6, via_drill_mm=0.3,
                       net_name="GND", clearance_mm=0.5):
    try:
        kicad = KiCad()
        board = kicad.get_board()
        print("Connected to KiCad IPC.")
    except Exception as e:
        print(f"Failed to connect to KiCad IPC: {e}")
        sys.exit(1)

    # 1. Convert user inputs to internal KiCad units (Integers)
    start_x = int(from_mm(start_x_mm))
    start_y = int(from_mm(start_y_mm))
    end_x = int(from_mm(end_x_mm))
    end_y = int(from_mm(end_y_mm))
    spacing_x = int(from_mm(spacing_x_mm))
    spacing_y = int(from_mm(spacing_y_mm))
    via_size = int(from_mm(via_size_mm))
    via_drill = int(from_mm(via_drill_mm))
    clearance = int(from_mm(clearance_mm))
    
    safe_distance = clearance + (via_size // 2)

    # 2. Find the target Net object
    target_net = None
    for net in board.get_nets():
        if net.name == net_name:
            target_net = net
            break
            
    if not target_net:
        print(f"Error: Net '{net_name}' not found on the board.")
        return
    print(f"Target Net '{net_name}' located.")

    # 3. Fetch specific geometry items for the obstacle map
    print("Building high-fidelity obstacle map...")
    types_to_fetch = [
        KiCadObjectType.KOT_PCB_PAD,
        KiCadObjectType.KOT_PCB_TRACE,
        KiCadObjectType.KOT_PCB_VIA,
        KiCadObjectType.KOT_PCB_ZONE
    ]
    
    all_items = board.get_items(types=types_to_fetch)
    obstacles = []
    
    for item in all_items:
        is_target_net = False
        if hasattr(item, "net") and item.net and item.net.name == net_name:
            is_target_net = True
            
        # Detect if this specific item is a component footprint pad
        is_pad = hasattr(item, "position") and hasattr(item, "size")
        
        # CRITICAL CHANGE: Only skip if it's target net AND NOT a pad.
        # This ensures GND pads are treated as strict obstacles to prevent solder wicking.
        if is_target_net and not is_pad:
            continue

        # Primitives Extraction 1: Traces / Tracks (with slope segmentation)
        if hasattr(item, "start") and hasattr(item, "end") and hasattr(item, "width"):
            w = int(item.width)
            x1, y1 = int(item.start.x), int(item.start.y)
            x2, y2 = int(item.end.x), int(item.end.y)
            
            if x1 != x2 and y1 != y2:
                length = math.hypot(x2 - x1, y2 - y1)
                step_distance = max(w // 2, 100000) 
                steps = max(int(length / step_distance), 1)
                
                for i in range(steps + 1):
                    t = i / steps
                    seg_x = int(x1 + (x2 - x1) * t)
                    seg_y = int(y1 + (y2 - y1) * t)
                    obstacles.append((
                        seg_x - w // 2,
                        seg_x + w // 2,
                        seg_y - w // 2,
                        seg_y + w // 2
                    ))
            else:
                obstacles.append((
                    min(x1, x2) - w // 2,
                    max(x1, x2) + w // 2,
                    min(y1, y2) - w // 2,
                    max(y1, y2) + w // 2
                ))
            
        # Primitives Extraction 2: Existing Vias (protects existing stitching/signal vias)
        elif hasattr(item, "position") and hasattr(item, "diameter") and not hasattr(item, "size"):
            d = int(item.diameter)
            obstacles.append((
                int(item.position.x - d // 2),
                int(item.position.x + d // 2),
                int(item.position.y - d // 2),
                int(item.position.y + d // 2)
            ))
            
        # Primitives Extraction 3: Component Pads (Now catches ALL nets, including GND)
        elif hasattr(item, "position") and hasattr(item, "size"):
            sx = int(item.size.x)
            sy = int(item.size.y)
            obstacles.append((
                int(item.position.x - sx // 2),
                int(item.position.x + sx // 2),
                int(item.position.y - sy // 2),
                int(item.position.y + sy // 2)
            ))

    print(f"Obstacle database built. Protecting {len(obstacles)} sensitive zones (including GND pads).")

    # 4. Strict Coordinate Collision Detector
    def is_collision(x, y):
        for min_x, max_x, min_y, max_y in obstacles:
            if (x >= min_x - safe_distance and 
                x <= max_x + safe_distance and 
                y >= min_y - safe_distance and 
                y <= max_y + safe_distance):
                return True
        return False

    vias_to_place = []
    y = start_y
    row = 0

    print("Stitching grid math executing...")

    # 5. Calculate Grid placement
    while y <= end_y:
        x = start_x
        if stagger and (row % 2 != 0):
            x += spacing_x // 2

        while x <= end_x:
            if not is_collision(x, y):
                new_via = Via()
                new_via.position = Vector2.from_xy(int(x), int(y))
                new_via.type = ViaType.VT_THROUGH
                new_via.diameter = via_size
                new_via.drill_diameter = via_drill
                new_via.net = target_net 
                
                vias_to_place.append(new_via)

            x += spacing_x
        
        y += spacing_y
        row += 1

    if not vias_to_place:
        print("Zero valid positions found. Try reducing clearance_mm slightly if still too restrictive.")
        return

    print(f"Identified {len(vias_to_place)} valid tight-fit locations. Submitting transaction...")

    # 6. Commit changes to KiCad
    commit = board.begin_commit()
    try:
        board.create_items(vias_to_place)
        board.push_commit(commit, f"IPC Script: Stitched {len(vias_to_place)} GND vias")
        print("Success! Vias safely placed while avoiding sloped lines and all component pads.")
    except Exception as e:
        board.drop_commit(commit)
        print(f"Transaction aborted due to error: {e}")

if __name__ == "__main__":
    stitch_grounds_ipc(
        start_x_mm=60.8,
        start_y_mm=40.8,
        end_x_mm=182.8,
        end_y_mm=139.2,
        spacing_x_mm=2.5,
        spacing_y_mm=2.5,
        stagger=True,
        via_size_mm=0.6,
        via_drill_mm=0.3,
        net_name="GND",
        clearance_mm=0.2
    )