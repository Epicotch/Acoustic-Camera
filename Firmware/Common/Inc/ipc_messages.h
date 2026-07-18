#include "defines.h"
#include <stdint.h>

#ifndef IPC_MESSAGES_H
#define IPC_MESSAGES_H

#define RPMSG_CHAN_NAME "notifs"

typedef float strength_t;

typedef enum {
    MSG_LOGGING,
    MSG_ERROR,
    MSG_TEXT,
    MSG_ACOUSTIC,
    MSG_TAKE,
    MSG_RELEASE,
} MSG_TYPE;

typedef struct {
    MSG_TYPE type;
    uint32_t length;
    void *address;
} message_notif_t;

typedef enum {
    M4,
    M7,
    ANY,
} own_t;

typedef struct {
    own_t acoustic_p;
    own_t acoustic_f;
    own_t text_ping;
    own_t text_pong;
    own_t control_ping;
    own_t control_pong;
} own_list_t;

extern __attribute__((section(".sram4_shared"))) strength_t acoustic_power[ACOUSTIC_HORIZ][ACOUSTIC_VERT];
extern __attribute__((section(".sram4_shared"))) strength_t acoustic_freq[ACOUSTIC_HORIZ][ACOUSTIC_VERT];

extern __attribute__((section(".sram4_shared"))) char text_ping[TEXT_MSG_SIZE];
extern __attribute__((section(".sram4_shared"))) char text_pong[TEXT_MSG_SIZE];

extern __attribute__((section(".sram4_shared"))) uint8_t ctrl_ping[CTRL_MSG_SIZE];
extern __attribute__((section(".sram4_shared"))) uint8_t ctrl_pong[CTRL_MSG_SIZE];

volatile own_t* get_own_flag(void* ptr, own_list_t* own_list);

#endif