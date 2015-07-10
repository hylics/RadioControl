/*rf22.c

 Copyright (C) 2014 Aethylic
 $Id: rf22.c,v 0.01 2014/08/03

*/

#ifndef RF22_H
#define RF22_H

#ifndef __stdint_h
#include <stdint.h>
#endif

//macro for spi handler
#define SPI_HANDLE        &hspi1

// packet length
#define PKT_LEN        0x19 // 25 byte

// This is the bit in the SPI address that marks it as a write
#define RF22_SPI_WRITE_MASK 0x80

// Max number of octets the RF22 Rx and Tx FIFOs can hold
#define RF22_FIFO_SIZE 64

// Keep track of the mode the RF22 is in
#define RF22_MODE_IDLE         0
#define RF22_MODE_RX           1
#define RF22_MODE_TX           2

// These values we set for FIFO thresholds (4, 55) are actually the same as the POR values
#define RF22_TXFFAEM_THRESHOLD 4
#define RF22_RXFFAFULL_THRESHOLD 55

// This is the default node address,
#define RF22_DEFAULT_NODE_ADDRESS 0

// This address in the TO addreess signifies a broadcast
#define RF22_BROADCAST_ADDRESS 0xff

// Number of registers to be passed to setModemConfig()
#define RF22_NUM_MODEM_CONFIG_REGS 18 

/* commands to function rf22_PwrChange(uint8_t arg)*/
#define POW_INCR          0x08
#define POW_DECR          0x10
//#define POW_RETURN        0x20

// Register names
#define RF22_REG_00_DEVICE_TYPE                         0x00
#define RF22_REG_01_VERSION_CODE                        0x01
#define RF22_REG_02_DEVICE_STATUS                       0x02
#define RF22_REG_03_INTERRUPT_STATUS1                   0x03
#define RF22_REG_04_INTERRUPT_STATUS2                   0x04
#define RF22_REG_05_INTERRUPT_ENABLE1                   0x05
#define RF22_REG_06_INTERRUPT_ENABLE2                   0x06
#define RF22_REG_07_OPERATING_MODE1                     0x07
#define RF22_REG_08_OPERATING_MODE2                     0x08
#define RF22_REG_09_OSCILLATOR_LOAD_CAPACITANCE         0x09
#define RF22_REG_0A_UC_OUTPUT_CLOCK                     0x0a
#define RF22_REG_0B_GPIO_CONFIGURATION0                 0x0b
#define RF22_REG_0C_GPIO_CONFIGURATION1                 0x0c
#define RF22_REG_0D_GPIO_CONFIGURATION2                 0x0d
#define RF22_REG_0E_IO_PORT_CONFIGURATION               0x0e
#define RF22_REG_0F_ADC_CONFIGURATION                   0x0f
#define RF22_REG_10_ADC_SENSOR_AMP_OFFSET               0x10
#define RF22_REG_11_ADC_VALUE                           0x11
#define RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION      0x12
#define RF22_REG_13_TEMPERATURE_VALUE_OFFSET            0x13
#define RF22_REG_14_WAKEUP_TIMER_PERIOD1                0x14
#define RF22_REG_15_WAKEUP_TIMER_PERIOD2                0x15
#define RF22_REG_16_WAKEUP_TIMER_PERIOD3                0x16
#define RF22_REG_17_WAKEUP_TIMER_VALUE1                 0x17
#define RF22_REG_18_WAKEUP_TIMER_VALUE2                 0x18
#define RF22_REG_19_LDC_MODE_DURATION                   0x19
#define RF22_REG_1A_LOW_BATTERY_DETECTOR_THRESHOLD      0x1a
#define RF22_REG_1B_BATTERY_VOLTAGE_LEVEL               0x1b
#define RF22_REG_1C_IF_FILTER_BANDWIDTH                 0x1c
#define RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE         0x1d
#define RF22_REG_1E_AFC_TIMING_CONTROL                  0x1e
#define RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   0x1f
#define RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    0x20
#define RF22_REG_21_CLOCK_RECOVERY_OFFSET2              0x21
#define RF22_REG_22_CLOCK_RECOVERY_OFFSET1              0x22
#define RF22_REG_23_CLOCK_RECOVERY_OFFSET0              0x23
#define RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    0x24
#define RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    0x25
#define RF22_REG_26_RSSI                                0x26
#define RF22_REG_27_RSSI_THRESHOLD                      0x27
#define RF22_REG_28_ANTENNA_DIVERSITY1                  0x28
#define RF22_REG_29_ANTENNA_DIVERSITY2                  0x29
#define RF22_REG_2A_AFC_LIMITER                         0x2a
#define RF22_REG_2B_AFC_CORRECTION_READ                 0x2b
#define RF22_REG_2C_OOK_COUNTER_VALUE_1                 0x2c
#define RF22_REG_2D_OOK_COUNTER_VALUE_2                 0x2d
#define RF22_REG_2E_SLICER_PEAK_HOLD                    0x2e
#define RF22_REG_30_DATA_ACCESS_CONTROL                 0x30
#define RF22_REG_31_EZMAC_STATUS                        0x31
#define RF22_REG_32_HEADER_CONTROL1                     0x32
#define RF22_REG_33_HEADER_CONTROL2                     0x33
#define RF22_REG_34_PREAMBLE_LENGTH                     0x34
#define RF22_REG_35_PREAMBLE_DETECTION_CONTROL1         0x35
#define RF22_REG_36_SYNC_WORD3                          0x36
#define RF22_REG_37_SYNC_WORD2                          0x37
#define RF22_REG_38_SYNC_WORD1                          0x38
#define RF22_REG_39_SYNC_WORD0                          0x39
#define RF22_REG_3A_TRANSMIT_HEADER3                    0x3a
#define RF22_REG_3B_TRANSMIT_HEADER2                    0x3b
#define RF22_REG_3C_TRANSMIT_HEADER1                    0x3c
#define RF22_REG_3D_TRANSMIT_HEADER0                    0x3d
#define RF22_REG_3E_PACKET_LENGTH                       0x3e
#define RF22_REG_3F_CHECK_HEADER3                       0x3f
#define RF22_REG_40_CHECK_HEADER2                       0x40
#define RF22_REG_41_CHECK_HEADER1                       0x41
#define RF22_REG_42_CHECK_HEADER0                       0x42
#define RF22_REG_43_HEADER_ENABLE3                      0x43
#define RF22_REG_44_HEADER_ENABLE2                      0x44
#define RF22_REG_45_HEADER_ENABLE1                      0x45
#define RF22_REG_46_HEADER_ENABLE0                      0x46
#define RF22_REG_47_RECEIVED_HEADER3                    0x47
#define RF22_REG_48_RECEIVED_HEADER2                    0x48
#define RF22_REG_49_RECEIVED_HEADER1                    0x49
#define RF22_REG_4A_RECEIVED_HEADER0                    0x4a
#define RF22_REG_4B_RECEIVED_PACKET_LENGTH              0x4b
#define RF22_REG_50_ANALOG_TEST_BUS_SELECT              0x50
#define RF22_REG_51_DIGITAL_TEST_BUS_SELECT             0x51
#define RF22_REG_52_TX_RAMP_CONTROL                     0x52
#define RF22_REG_53_PLL_TUNE_TIME                       0x53
#define RF22_REG_55_CALIBRATION_CONTROL                 0x55
#define RF22_REG_56_MODEM_TEST                          0x56
#define RF22_REG_57_CHARGE_PUMP_TEST                    0x57
#define RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING        0x58
#define RF22_REG_59_DIVIDER_CURRENT_TRIMMING            0x59
#define RF22_REG_5A_VCO_CURRENT_TRIMMING                0x5a
#define RF22_REG_5B_VCO_CALIBRATION                     0x5b
#define RF22_REG_5C_SYNTHESIZER_TEST                    0x5c
#define RF22_REG_5D_BLOCK_ENABLE_OVERRIDE1              0x5d
#define RF22_REG_5E_BLOCK_ENABLE_OVERRIDE2              0x5e
#define RF22_REG_5F_BLOCK_ENABLE_OVERRIDE3              0x5f
#define RF22_REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS  0x60
#define RF22_REG_61_CHANNEL_FILTER_COEFFICIENT_VALUE    0x61
#define RF22_REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL      0x62
#define RF22_REG_63_RC_OSCILLATOR_COARSE_CALIBRATION    0x63
#define RF22_REG_64_RC_OSCILLATOR_FINE_CALIBRATION      0x64
#define RF22_REG_65_LDO_CONTROL_OVERRIDE                0x65
#define RF22_REG_66_LDO_LEVEL_SETTINGS                  0x66
#define RF22_REG_67_DELTA_SIGMA_ADC_TUNING1             0x67
#define RF22_REG_68_DELTA_SIGMA_ADC_TUNING2             0x68
#define RF22_REG_69_AGC_OVERRIDE1                       0x69
#define RF22_REG_6A_AGC_OVERRIDE2                       0x6a
#define RF22_REG_6B_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS 0x6b
#define RF22_REG_6C_GFSK_FIR_FILTER_COEFFICIENT_VALUE   0x6c
#define RF22_REG_6D_TX_POWER                            0x6d
#define RF22_REG_6E_TX_DATA_RATE1                       0x6e
#define RF22_REG_6F_TX_DATA_RATE0                       0x6f
#define RF22_REG_70_MODULATION_CONTROL1                 0x70
#define RF22_REG_71_MODULATION_CONTROL2                 0x71
#define RF22_REG_72_FREQUENCY_DEVIATION                 0x72
#define RF22_REG_73_FREQUENCY_OFFSET1                   0x73
#define RF22_REG_74_FREQUENCY_OFFSET2                   0x74
#define RF22_REG_75_FREQUENCY_BAND_SELECT               0x75
#define RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1          0x76
#define RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0          0x77
#define RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT    0x79
#define RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE         0x7a
#define RF22_REG_7C_TX_FIFO_CONTROL1                    0x7c
#define RF22_REG_7D_TX_FIFO_CONTROL2                    0x7d
#define RF22_REG_7E_RX_FIFO_CONTROL                     0x7e
#define RF22_REG_7F_FIFO_ACCESS                         0x7f

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RF-22 Manual
// RF22_REG_00_DEVICE_TYPE                      0x00
#define RF22_DEVICE_TYPE_RX_TRX                 0x08
#define RF22_DEVICE_TYPE_TX                     0x07

// RF22_REG_02_DEVICE_STATUS                    0x02
#define RF22_FFOVL                              0x80
#define RF22_FFUNFL                             0x40
#define RF22_RXFFEM                             0x20
#define RF22_HEADERR                            0x10
#define RF22_FREQERR                            0x08
#define RF22_LOCKDET                            0x04
#define RF22_CPS                                0x03
#define RF22_CPS_IDLE                           0x00
#define RF22_CPS_RX                             0x01
#define RF22_CPS_TX                             0x10

// RF22_REG_03_INTERRUPT_STATUS1                0x03
/*Function*/
/*Detailed Description of Status Registers when not Enabled as Interrupts*/
/*FIFO Underflow/Overflow Error. When set to 1 the TX or RX FIFO has overflowed or underflowed*/
/*It is cleared only by applying FIFO reset to the specific FIFO that caused the condition*/
#define RF22_IFFERROR                           0x0080

/*TX FIFO Almost Full. When set to 1 the TX FIFO has met its almost full threshold and needs to be transmitted*/
/*Will be set when the number of bytes written to the TX FIFO is greater than the TX Almost 
Full Threshold set in SPI Reg 7Ch. It is automatically cleared when a sufficient number of 
bytes have been read from the TX FIFO and transmitted, such that the remaining number 
of bytes in the TX FIFO is less than or equal to the TX Almost Full Threshold.*/
#define RF22_ITXFFAFULL                         0x0040

/*TX FIFO Almost Empty. When set to 1 the TX FIFO is almost empty and needs to be filled*/
/*Will be set when the number of bytes remaining for transmission in the TX FIFO is less 
than or equal to the TX Almost Empty Threshold set in SPI Reg 7Dh. It is automatically 
cleared when a sufficient number of bytes have been written to the TX FIFO, such that the 
number of data bytes not yet transmitted is above the TX Almost Empty Threshold. 
Update of this status flag requires a clock from the internal TX domain circuitry, and thus 
may not indicate accurately until TX mode is entered*/
#define RF22_ITXFFAEM                           0x0020

/*RX FIFO Almost Full. When set to 1 the RX FIFO has met its almost full threshold and 
needs to be read by the microcontroller*/
/*Will be set when the number of bytes received (and not yet read-out) in RX FIFO is greater 
than the RX Almost Full threshold set in SPI Reg 7Eh. It is automatically cleared when a 
sufficient number of bytes are read from the RX FIFO, such that the remaining number of 
bytes in the RX FIFO is below the RX Almost Full Threshold. Update of this status flag 
requires a clock from the internal RX domain circuitry, and thus may not indicate accurately until RX mode is entered.*/
#define RF22_IRXFFAFULL                         0x0010

/*External Interrupt. When set to 1 an interrupt occurred on one ofthe GPIO’s if it is programmed so.
The status can be checked in register 0Eh. See GPIOx Configuration section for the details*/
/*Will be set upon complete transmission ofa packet (no TX abort). This status will be 
cleared if 1) The chip is commanded to leave FIFO mode, or 2) While the chip is in FIFO 
mode a new transmission is started. Packet Sent functionality remains available even if 
the TX Packet Handler (enpactx bit D3 in SPI Reg 30h) is not enabled, as it is possible 
construct and send an entire packet from the FIFO without making use of the Packet Handler.*/
#define RF22_IEXT                               0x0008

/*Packet Sent Interrupt. When set to1 a valid packet has been transmitted*/
/*Will be set upon complete transmission ofa packet (no TX abort). This status will be 
cleared if 1) The chip is commanded to leave FIFO mode, or 2) While the chip is in FIFO 
mode a new transmission is started. Packet Sent functionality remains available even if 
the TX Packet Handler (enpactx bit D3 in SPI Reg 30h) is not enabled, as it is possible 
construct and send an entire packet from the FIFO without making use of the Packet Handler.*/
#define RF22_IPKSENT                            0x0004

/*Valid Packet Received.When set to 1 a valid packet has been received*/
/*Will be set upon full and correct reception of a packet (no RX abort). It is not automatically 
cleared by simply re-entering RX mode, but isonly cleared upon detection of a valid Sync 
Word in the next RX packet. Packet Valid functionality is not available if the RX Packet 
Handler (enpacrx bit D7 in SPI Reg 30h) is not enabled.*/	
#define RF22_IPKVALID                           0x0002

/*CRC Error. When set to 1 the cyclic redundancy check is failed*/
/*Will be set if the CRC computed during RX differs from the CRC sent in the packet by the 
TX. It is cleared upon start of data reception in a new packet. CRC functionality is not 
available if the RX Packet Handler (enpacrxbit D7 in SPI Reg 30h) is not enabled.*/
#define RF22_ICRCERROR                          0x0001

// RF22_REG_04_INTERRUPT_STATUS2                0x04
// <<8

/*Sync Word Detected. When a sync word is detected this bit will be set to 1*/
/*Goes high once the Sync Word is detected. Goes low once we are done receiving the current packet.*/
#define RF22_ISWDET                             0x8000

/*Valid Preamble Detected. When a preamble is detected this bit will be set to 1.*/
/*Goes high once the preamble is detected. Goes low once the sync is detected or the RX*/ 
#define RF22_IPREAVAL                           0x4000

/*Invalid Preamble Detected. When the preamble is not found within a period of time set by the invalid preamble detection threshold
in Register 60h, this bit will be set to 1.*/
/*Self clearing, user should use this as an interrupt source rather than a status.*/
#define RF22_IPREAINVAL                         0x2000

/*RSSI. When RSSI level exceeds the programmed threshold this bit will be set to 1.*/
/*Should remain high as long as the RSSI value is above programmed threshold level*/
#define RF22_IRSSI                              0x1000

/*Wake-Up-Timer. On the expiration of programmed wake-up timer this bit will be set to 1.*/
/*Wake time timer interrupt. Use as an interrupt, not as a status.*/
#define RF22_IWUT                               0x0800

/*Low Battery Detect. When a low battery event has been detected this bit will be set to 1. This interrupt event 
is saved even if it is not enabled by the mask register bit and causes an interrupt after it is enabled.*/
/*Low Battery Detect. When a low battery event has been detected this bit will be set to 1.
It will remain set as long as the battery voltage is below the threshold but will reset
if the voltage returns to a level higher than the threshold.*/
#define RF22_ILBD                               0x0400

/*Chip Ready (XTAL). When a chip ready event has been detected this bit will be set to 1.*/
/*Chip ready goes high once we enable the xtal, TX or RX, and a settling time for the Xtal 
 clock elapses. The status stay high unless we go back to Idle mode.*/
#define RF22_ICHIPRDY                           0x0200

/*Power-on-Reset (POR). When the chip detects a Power on Reset above the desired setting this bit will be set to 1*/
#define RF22_IPOR                               0x0100

// RF22_REG_05_INTERRUPT_ENABLE1                0x05
#define RF22_ENFFERR                            0x80
#define RF22_ENTXFFAFULL                        0x40
#define RF22_ENTXFFAEM                          0x20
#define RF22_ENRXFFAFULL                        0x10
#define RF22_ENEXT                              0x08
#define RF22_ENPKSENT                           0x04
#define RF22_ENPKVALID                          0x02
#define RF22_ENCRCERROR                         0x01

// RF22_REG_06_INTERRUPT_ENABLE2                0x06
#define RF22_ENSWDET                            0x80
#define RF22_ENPREAVAL                          0x40
#define RF22_ENPREAINVAL                        0x20
#define RF22_ENRSSI                             0x10
#define RF22_ENWUT                              0x08
#define RF22_ENLBDI                             0x04
#define RF22_ENCHIPRDY                          0x02
#define RF22_ENPOR                              0x01

// RF22_REG_07_OPERATING_MODE                   0x07
#define RF22_SWRES                              0x80
#define RF22_ENLBD                              0x40
#define RF22_ENWT                               0x20
#define RF22_X32KSEL                            0x10
#define RF22_TXON                               0x08
#define RF22_RXON                               0x04
#define RF22_PLLON                              0x02
#define RF22_XTON                               0x01

// RF22_REG_08_OPERATING_MODE2                  0x08
#define RF22_ANTDIV                             0xc0
#define RF22_RXMPK                              0x10
#define RF22_AUTOTX                             0x08
#define RF22_ENLDM                              0x04
#define RF22_FFCLRRX                            0x02
#define RF22_FFCLRTX                            0x01

// RF22_REG_0A_UC_OUTPUT_CLOCK
#define RF22_UC_30MHZ                           0x00
#define RF22_UC_15MHZ                           0x02
#define RF22_UC_10MHZ                           0x04
#define RF22_UC_4MHZ                            0x06
#define RF22_UC_3MHZ                            0x08
#define RF22_UC_2MHZ                            0x0a
#define RF22_UC_1MHZ                            0x0c
#define RF22_UC_32786HZ                         0x0e

// RF22_REG_0B-0C_GPIO_CONFIGURATION0
#define RF22_GPIOX_DRV001PUP0                    0xe0
#define RF22_GPIOX_DRV00                         0x40
#define RF22_GPIOX_DRV01                         0x80
#define RF22_GPIOX_PUP0                          0x20
#define RF22_GPIOX_DIRECTDIGITALOUT              0x0a
#define RF22_GPIOX_ADC_INPUT                     0x07
#define RF22_GPIOX_TX_STATE                      0x12
#define RF22_GPIOX_RX_STATE                      0x15
#define RF22_GPIOX_ANTDIV1                       0x17
#define RF22_GPIOX_ANTDIV2                       0x18
#define RF22_GPIOX_CCA                           0x1c

// RF22_REG_0D_GPIO_CONFIGURATION2
#define RF22_GPIO2_MCUCLOCK_OUT                 0x00




// RF22_REG_0F_ADC_CONFIGURATION                0x0f
#define RF22_ADCSTART                           0x80
#define RF22_ADCDONE                            0x80
#define RF22_ADCSEL                             0x70
#define RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00
#define RF22_ADCSEL_GPIO0_SINGLE_ENDED          0x10
#define RF22_ADCSEL_GPIO1_SINGLE_ENDED          0x20
#define RF22_ADCSEL_GPIO2_SINGLE_ENDED          0x30
#define RF22_ADCSEL_GPIO0_GPIO1_DIFFERENTIAL    0x40
#define RF22_ADCSEL_GPIO1_GPIO2_DIFFERENTIAL    0x50
#define RF22_ADCSEL_GPIO0_GPIO2_DIFFERENTIAL    0x60
#define RF22_ADCSEL_GND                         0x70
#define RF22_ADCREF                             0x0c
#define RF22_ADCREF_BANDGAP_VOLTAGE             0x00
#define RF22_ADCREF_VDD_ON_3                    0x08
#define RF22_ADCREF_VDD_ON_2                    0x0c
#define RF22_ADCGAIN                            0x03

// RF22_REG_10_ADC_SENSOR_AMP_OFFSET            0x10
#define RF22_ADCOFFS                            0x0f

// RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION   0x12
#define RF22_TSRANGE                            0xc0
#define RF22_TSRANGE_M64_64C                    0x00
#define RF22_TSRANGE_M64_192C                   0x40
#define RF22_TSRANGE_0_128C                     0x80
#define RF22_TSRANGE_M40_216F                   0xc0
#define RF22_ENTSOFFS                           0x20
#define RF22_ENTSTRIM                           0x10
#define RF22_TSTRIM                             0x0f

// RF22_REG_14_WAKEUP_TIMER_PERIOD1             0x14
#define RF22_WTR                                0x3c
#define RF22_WTD                                0x03

// RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE      0x1d
#define RF22_AFBCD                              0x80
#define RF22_ENAFC                              0x40
#define RF22_AFCGEARH                           0x38
#define RF22_AFCGEARL                           0x07

// RF22_REG_1E_AFC_TIMING_CONTROL               0x1e
#define RF22_SWAIT_TIMER                        0xc0
#define RF22_SHWAIT                             0x38
#define RF22_ANWAIT                             0x07

// RF22_REG_30_DATA_ACCESS_CONTROL              0x30
#define RF22_ENPACRX                            0x80
#define RF22_MSBFRST                            0x00
#define RF22_LSBFRST                            0x40
#define RF22_CRCHDRS                            0x00
#define RF22_CRCDONLY                           0x20
#define RF22_ENPACTX                            0x08
#define RF22_ENCRC                              0x04
#define RF22_CRC                                0x03
#define RF22_CRC_CCITT                          0x00
#define RF22_CRC_CRC_16_IBM                     0x01
#define RF22_CRC_IEC_16                         0x02
#define RF22_CRC_BIACHEVA                       0x03

// RF22_REG_32_HEADER_CONTROL1                  0x32
#define RF22_BCEN                               0xf0
#define RF22_BCEN_NONE                          0x00
#define RF22_BCEN_HEADER0                       0x10
#define RF22_BCEN_HEADER1                       0x20
#define RF22_BCEN_HEADER2                       0x40
#define RF22_BCEN_HEADER3                       0x80
#define RF22_HDCH                               0x0f
#define RF22_HDCH_NONE                          0x00
#define RF22_HDCH_HEADER0                       0x01
#define RF22_HDCH_HEADER1                       0x02
#define RF22_HDCH_HEADER2                       0x04
#define RF22_HDCH_HEADER3                       0x08

// RF22_REG_33_HEADER_CONTROL2                  0x33
#define RF22_HDLEN                              0x70
#define RF22_HDLEN_0                            0x00
#define RF22_HDLEN_1                            0x10
#define RF22_HDLEN_2                            0x20
#define RF22_HDLEN_3                            0x30
#define RF22_HDLEN_4                            0x40
#define RF22_VARPKLEN                           0x00
#define RF22_FIXPKLEN                           0x08
#define RF22_SYNCLEN                            0x06
#define RF22_SYNCLEN_1                          0x00
#define RF22_SYNCLEN_2                          0x02
#define RF22_SYNCLEN_3                          0x04
#define RF22_SYNCLEN_4                          0x06
#define RF22_PREALEN8                           0x01

// RF22_REG_6D_TX_POWER                         0x6d
#define RF22_TXPOW                              0x07
#define RF22_TXPOW_4X31                         0x08 // Not used in RFM22B
#define RF22_TXPOW_1DBM                         0x00
#define RF22_TXPOW_2DBM                         0x01
#define RF22_TXPOW_5DBM                         0x02
#define RF22_TXPOW_8DBM                         0x03
#define RF22_TXPOW_11DBM                        0x04
#define RF22_TXPOW_14DBM                        0x05
#define RF22_TXPOW_17DBM                        0x06
#define RF22_TXPOW_20DBM                        0x07
// IN RFM23B
#define RF22_TXPOW_LNA_SW                       0x08

// RF22_REG_71_MODULATION_CONTROL2              0x71
#define RF22_TRCLK                              0xc0
#define RF22_TRCLK_NONE                         0x00
#define RF22_TRCLK_GPIO                         0x40
#define RF22_TRCLK_SDO                          0x80
#define RF22_TRCLK_NIRQ                         0xc0
#define RF22_DTMOD                              0x30
#define RF22_DTMOD_DIRECT_GPIO                  0x00
#define RF22_DTMOD_DIRECT_SDI                   0x10
#define RF22_DTMOD_FIFO                         0x20
#define RF22_DTMOD_PN9                          0x30
#define RF22_ENINV                              0x08
#define RF22_FD8                                0x04
#define RF22_MODTYP                             0x30
#define RF22_MODTYP_UNMODULATED                 0x00
#define RF22_MODTYP_OOK                         0x01
#define RF22_MODTYP_FSK                         0x02
#define RF22_MODTYP_GFSK                        0x03

// RF22_REG_75_FREQUENCY_BAND_SELECT            0x75
#define RF22_SBSEL                              0x40
#define RF22_HBSEL                              0x20
#define RF22_FB                                 0x1f 

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  RF22 handle Structure definition  
  */ 
/*typedef struct __RF22_HandleTypeDef
{
	
} RF22_HandleTypeDef;*/

/** 
  * @brief  RF22 Status structures definition  
  */  
typedef enum 
{
  RF22_OK       = 0x00,
  RF22_ERROR    = 0x01,
  RF22_BUSY     = 0x02,
  RF22_TIMEOUT  = 0x03
} RF22_StatusTypeDef;



/* Public variables ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/


/* Public function prototypes ------------------------------------------------*/
//RF22_StatusTypeDef rf22_init(uint8_t (*config)[2]);
RF22_StatusTypeDef rf22_Init(void);
RF22_StatusTypeDef rf22_BrstWriteFifo(uint8_t *pwbdata, uint8_t size);
RF22_StatusTypeDef rf22_BrstReadFifo(uint8_t *prbdata, uint8_t size);
RF22_StatusTypeDef rf22_ItRead(volatile uint16_t *ptr_status);
uint8_t rf22_RssiRead(void);
RF22_StatusTypeDef rf22_RssiThreshholdSet(uint8_t rssi_threshold);
RF22_StatusTypeDef rf22_TxOn(void);
RF22_StatusTypeDef rf22_RxOn(void);
uint8_t rf22_PwrChange(uint8_t arg);
//RF22_StatusTypeDef rf22_write();
//RF22_StatusTypeDef rf22_read();
//RF22_StatusTypeDef rf22_last_RSSI();





#endif

