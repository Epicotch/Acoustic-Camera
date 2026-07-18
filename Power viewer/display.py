import numpy as np
import serial
from matplotlib import pyplot as plt

vert_fov = np.pi/2
horiz_fov = np.pi/2
horiz_pix = 20
vert_pix = 20

def cartesian_product(*arrays):
    la = len(arrays)
    dtype = np.result_type(*arrays)
    arr = np.empty([len(a) for a in arrays] + [la], dtype=dtype)
    for i, a in enumerate(np.ix_(*arrays)):
        arr[...,i] = a
    return arr.reshape(-1, la)

def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError(f"wanted {n}, got {len(buf)}")
        buf += chunk
    return buf

# match firmware: FOV in radians -> degrees
HFOV_DEG = np.degrees(horiz_fov)
VFOV_DEG = np.degrees(vert_fov)
extent = [-HFOV_DEG/2, HFOV_DEG/2, -VFOV_DEG/2, VFOV_DEG/2]

plt.ion()
fig, ax = plt.subplots(figsize=(10, 7))
img = ax.imshow(np.zeros((vert_pix, horiz_pix)),
                extent=extent, origin='lower', aspect='equal',
                cmap='inferno', interpolation='bilinear')
cbar = fig.colorbar(img, ax=ax, label='Power')
ax.set_xlabel('θx (deg)'); ax.set_ylabel('θy (deg)')

# one text artist per pixel, reused every frame (creating new ones leaks/slows)
xs = np.linspace(extent[0], extent[1], horiz_pix, endpoint=False) + HFOV_DEG/horiz_pix/2
ys = np.linspace(extent[2], extent[3], vert_pix, endpoint=False) + VFOV_DEG/vert_pix/2
labels = [[ax.text(xs[i], ys[j], '', ha='center', va='center',
                   fontsize=6, color='w') for i in range(horiz_pix)]
          for j in range(vert_pix)]

peak_mark, = ax.plot([], [], 'c+', ms=15, mew=2)

def update(power, freq):
    # firmware arrays are [x][y]; imshow wants [row=y][col=x]
    P = power.reshape(horiz_pix, vert_pix).T
    F = freq.reshape(horiz_pix, vert_pix).T

    img.set_data(P)
    img.set_clim(P.min(), P.max())          # or fixed/log scale, see below

    thresh = P.max()                         # label only significant pixels
    for j in range(vert_pix):
        for i in range(horiz_pix):
            labels[j][i].set_text(f'{F[j,i]/1000:.1f}k' if P[j,i] >= thresh else '')

    jy, ix = np.unravel_index(P.argmax(), P.shape)
    peak_mark.set_data([xs[ix]], [ys[jy]])
    ax.set_title(f'peak {F[jy,ix]:.0f} Hz @ ({xs[ix]:.1f}°, {ys[jy]:.1f}°)')

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

pix_x = np.linspace(-horiz_fov / 2, horiz_fov / 2, horiz_pix)
pix_y = np.linspace(-vert_fov / 2, vert_fov / 2, vert_pix)

ser = serial.Serial("COM4")

MAGIC = {b'\xFD\xFD\xFD\xFD': 'power', b'\xFE\xFE\xFE\xFE': 'freq'}
PAYLOAD = horiz_pix * vert_pix * 4

power = np.zeros((horiz_pix, vert_pix))
freq = np.zeros((horiz_pix, vert_pix))

window = b''
while True:
    b = ser.read(1)
    if not b:
        continue
    window = (window + b)[-4:]
    kind = MAGIC.get(window)
    if kind:
        window = b''
        payload = read_exact(ser, PAYLOAD)
        if kind == 'power':
            power = np.frombuffer(payload, dtype='<f4').reshape(horiz_pix, vert_pix)
        elif kind == 'freq':
            freq = np.frombuffer(payload, dtype='<f4').reshape(horiz_pix, vert_pix)
            update(power, freq)