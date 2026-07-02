#include "ipc_messages.h"

__attribute__((section(".sram4_shared"))) strength_t acoustic_ping[ACOUSTIC_SIZE];
__attribute__((section(".sram4_shared"))) strength_t acoustic_pong[ACOUSTIC_SIZE];

__attribute__((section(".sram4_shared"))) char text_ping[TEXT_MSG_SIZE];
__attribute__((section(".sram4_shared"))) char text_pong[TEXT_MSG_SIZE];

__attribute__((section(".sram4_shared"))) uint8_t ctrl_ping[CTRL_MSG_SIZE];
__attribute__((section(".sram4_shared"))) uint8_t ctrl_pong[CTRL_MSG_SIZE];    