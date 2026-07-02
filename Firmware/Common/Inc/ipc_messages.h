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
    uintptr_t address;
} message_notif_t;

typedef enum {
    M4,
    M7,
    ANY,
} OWN_TYPE;

typedef struct {
    OWN_TYPE acoustic_ping;
    OWN_TYPE acoustic_pong;
    OWN_TYPE text_ping;
    OWN_TYPE text_pong;
    OWN_TYPE control_ping;
    OWN_TYPE control_pong;
} own_list_t;

extern __attribute__((section(".sram4_shared"))) strength_t acoustic_ping[ACOUSTIC_SIZE];
extern __attribute__((section(".sram4_shared"))) strength_t acoustic_pong[ACOUSTIC_SIZE];

extern __attribute__((section(".sram4_shared"))) char text_ping[TEXT_MSG_SIZE];
extern __attribute__((section(".sram4_shared"))) char text_pong[TEXT_MSG_SIZE];

extern __attribute__((section(".sram4_shared"))) uint8_t ctrl_ping[CTRL_MSG_SIZE];
extern __attribute__((section(".sram4_shared"))) uint8_t ctrl_pong[CTRL_MSG_SIZE];

#endif