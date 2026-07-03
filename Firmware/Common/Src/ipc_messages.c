#include "ipc_messages.h"
#include <stddef.h>

__attribute__((section(".sram4_shared"))) strength_t acoustic_ping[ACOUSTIC_SIZE];
__attribute__((section(".sram4_shared"))) strength_t acoustic_pong[ACOUSTIC_SIZE];

__attribute__((section(".sram4_shared"))) char text_ping[TEXT_MSG_SIZE];
__attribute__((section(".sram4_shared"))) char text_pong[TEXT_MSG_SIZE];

__attribute__((section(".sram4_shared"))) uint8_t ctrl_ping[CTRL_MSG_SIZE];
__attribute__((section(".sram4_shared"))) uint8_t ctrl_pong[CTRL_MSG_SIZE];

volatile own_t* get_own_flag(void* ptr, own_list_t* own_list) {
    if (ptr == acoustic_ping) return &own_list->acoustic_ping;
    if (ptr == acoustic_pong) return &own_list->acoustic_pong;
    if (ptr == text_ping) return &own_list->text_ping;
    if (ptr == text_pong) return &own_list->text_pong;
    if (ptr == ctrl_ping) return &own_list->control_ping;
    if (ptr == ctrl_pong) return &own_list->control_pong;
    return NULL;
}