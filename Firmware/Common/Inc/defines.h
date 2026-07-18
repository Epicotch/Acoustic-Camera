#ifndef DEFINES_H
#define DEFINES_H

#ifndef DEBUG
// #define DEBUG
#endif
// #define RELEASE

#define ACOUSTIC_HORIZ      40
#define ACOUSTIC_VERT       40
#define ACOUSTIC_SIZE       (ACOUSTIC_HORIZ * ACOUSTIC_VERT)

#define TEXT_MSG_SIZE       512
#define CTRL_MSG_SIZE       8

#define VERTICAL_FOV        90.0 * 3.1415926535 / 180
#define HORIZ_FOV           90.0 * 3.1415926535 / 180

#define NUM_CHANNELS        16
#define SAMPLES_PER_CH      1024
#define DMA_BUF_SIZE        (NUM_CHANNELS * SAMPLES_PER_CH * 2)

#define SAMPLE_RATE         48000

#define HSEM_I2C4  (5U)

#endif