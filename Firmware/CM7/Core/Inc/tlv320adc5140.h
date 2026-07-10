/**
 * @file tlv320adc5140_regs.h
 * @brief Register address definitions for the Texas Instruments TLV320ADC5140.
 * * This file maps the Page 0 device configuration register names to their 
 * respective 8-bit hex addresses, tailored for an 8x PDM microphone 
 * daisy-chained TDM setup.
 */

#ifndef TLV320ADC5140_REGS_H_
#define TLV320ADC5140_REGS_H_

/* ========================================================================== */
/* PAGE CONTROL REGISTER                                */
/* ========================================================================== */
#define ADC5140_PAGE_SELECT         0x00 /**< Selects the register page (0x00-0xFF) */

/* ========================================================================== */
/* SYSTEM & POWER RESET CONFIGURATIONS                         */
/* ========================================================================== */
#define ADC5140_SW_RESET            0x01 /**< Software Reset Controls */
#define ADC5140_SLEEP_CFG           0x02 /**< Sleep and Wake-up Configurations */
#define ADC5140_SHDN_CFG            0x05 /**< Shutdown Configuration */

/* ========================================================================== */
/* AUDIO SERIAL INTERFACE (ASI) CONFIG                         */
/* ========================================================================== */
#define ADC5140_ASI_CFG0            0x07 /**< ASI Format, Word Length, and Clock Protocol */
#define ADC5140_ASI_CFG1            0x08 /**< ASI Bus Keeper, Tx Edge, and Hi-Z Control */
#define ADC5140_ASI_CFG2            0x09 /**< ASI Daisy-Chain and Sync Error Recovery */

/* ========================================================================== */
/* ASI OUTPUT TDM SLOT ALLOCATIONS (CH1 - CH8)                  */
/* ========================================================================== */
#define ADC5140_ASI_CH1             0x0B /**< Channel 1 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH2             0x0C /**< Channel 2 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH3             0x0D /**< Channel 3 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH4             0x0E /**< Channel 4 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH5             0x0F /**< Channel 5 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH6             0x10 /**< Channel 6 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH7             0x11 /**< Channel 7 TDM Slot Mapping Configuration */
#define ADC5140_ASI_CH8             0x12 /**< Channel 8 TDM Slot Mapping Configuration */

/* ========================================================================== */
/* MASTER CLOCK & MULTIPLEXER CONTROL                      */
/* ========================================================================== */
#define ADC5140_MST_CFG0            0x13 /**< ASI Master Mode Clock Config 0 */
#define ADC5140_MST_CFG1            0x14 /**< ASI Master Mode Clock Config 1 */
#define ADC5140_ASI_STS             0x15 /**< ASI Status Monitor (Read-Only) */
#define ADC5140_CLK_SRC             0x16 /**< Master Clock Input Source Selection */
#define ADC5140_PDMCLK_CFG          0x1F /**< PDM Clock Frequency Divider Divider Options */
#define ADC5140_PDMIN_CFG           0x20 /**< PDM Digital Input Sampling Options */

/* ========================================================================== */
/* DIGITAL HARDWARE IO CONFIGURATION (PIN MUX)                */
/* ========================================================================== */
#define ADC5140_GPIO1_CFG           0x21 /**< GPIO1 Pin Function (e.g., SDIN Loop-back Input) */
#define ADC5140_GPO1_CFG            0x22 /**< GPO1 Pin Function (e.g., Master PDMCLK Output) */
#define ADC5140_GPO2_CFG            0x23 /**< GPO2 Pin Function Selection */
#define ADC5140_GPO3_CFG            0x24 /**< GPO3 Pin Function Selection */
#define ADC5140_GPO4_CFG            0x25 /**< GPO4 Pin Function Selection */
#define ADC5140_GPO_VAL             0x29 /**< Manual GPO State Value Configuration */
#define ADC5140_GPIO_MON            0x2A /**< Monitor GPIO Input Values (Read-Only) */
#define ADC5140_GPI_CFG0            0x2B /**< Channel 1-4 Digital Input Buffers & Channel Links */
#define ADC5140_GPI_CFG1            0x2C /**< Channel 5-8 Digital Input Buffers & Channel Links */
#define ADC5140_GPI_MON             0x2F /**< Monitor GPI Input Pin Latched Values (Read-Only) */

/* ========================================================================== */
/* CHANNEL INPUT MODE & CONFIGURATION MUX (CH1 - CH8)             */
/* ========================================================================== */
#define ADC5140_BIAS_CFG            0x3B /**< Microphone Bias Voltage Control Selection */

#define ADC5140_CH1_CFG0            0x3C /**< CH1 Input Type (Set 0x02/0x42 for PDM Rising/Falling) */
#define ADC5140_CH1_CFG1            0x3D /**< CH1 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH2_CFG0            0x41 /**< CH2 Input Type Configuration */
#define ADC5140_CH2_CFG1            0x42 /**< CH2 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH3_CFG0            0x46 /**< CH3 Input Type Configuration */
#define ADC5140_CH3_CFG1            0x47 /**< CH3 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH4_CFG0            0x4B /**< CH4 Input Type Configuration */
#define ADC5140_CH4_CFG1            0x4C /**< CH4 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH5_CFG0            0x50 /**< CH5 Configuration (Digital PDM Only) */
#define ADC5140_CH5_CFG1            0x51 /**< CH5 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH6_CFG0            0x55 /**< CH6 Configuration (Digital PDM Only) */
#define ADC5140_CH6_CFG1            0x56 /**< CH6 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH7_CFG0            0x5A /**< CH7 Configuration (Digital PDM Only) */
#define ADC5140_CH7_CFG1            0x5B /**< CH7 Digital Volume Control Attenuation/Gain */
#define ADC5140_CH8_CFG0            0x5F /**< CH8 Configuration (Digital PDM Only) */
#define ADC5140_CH8_CFG1            0x60 /**< CH8 Digital Volume Control Attenuation/Gain */

/* ========================================================================== */
/* MASTER CHANNEL ENABLE & POWER CONTROLS                    */
/* ========================================================================== */
#define ADC5140_IN_CH_EN            0x73 /**< Input Channel Programmable Hardware Enable Block */
#define ADC5140_ASI_OUT_CH_EN       0x74 /**< ASI Output Slots Master Bitmask Enable */
#define ADC5140_PWR_CFG             0x75 /**< Power-Up Signal Chain (PLL, MicBias, Core Blocks) */

/* ========================================================================== */
/* DEVICE CORE STATUS MONITOR                             */
/* ========================================================================== */
#define ADC5140_DEV_STS0            0x76 /**< Channel Power Status Summary (Read-Only) */
#define ADC5140_DEV_STS1            0x77 /**< Core Device Operating Status Flag (Read-Only) */

#endif /* TLV320ADC5140_REGS_H_ */