#ifndef COMMAND_H
#define COMMAND_H

#define COMMAND_UPDATE_FIRMWARE            0x8f
#define COMMAND_START_FILE                 0x90
#define COMMAND_FILE_CHUNK                 0x91
#define COMMAND_END_FILE                   0x92
#define COMMAND_READ_SEND_FILE_REPORT      0x93
#define COMMAND_RUN_JOB                    0x94
#define COMMAND_EXECUTE_GCODE              0x95
#define COMMAND_LIST_FILES                 0x96
#define COMMAND_START_JOB_FILE             0x97
#define COMMAND_PAUSE_JOB                  0x98
#define COMMAND_WRITE_HEAD_EEPROM          0xa0
#define COMMAND_READ_HEAD_EEPROM           0xa1
#define COMMAND_WRITE_REEL0_EEPROM         0xa2
#define COMMAND_READ_REEL0_EEPROM          0xa3
#define COMMAND_WRITE_REEL1_EEPROM         0xa4
#define COMMAND_READ_REEL1_EEPROM          0xa5
#define COMMAND_SHOW_STATUS                0xb0
#define COMMAND_READ_UNIQUE_ID             0xb2
#define COMMAND_REPORT_ERRORS              0xb3
#define COMMAND_REPORT_FIRMWARE_REVISION   0xb4
#define COMMAND_READ_HOURS_COUNTER         0xb6
#define COMMAND_SHOW_SD_CARD_STATE         0xbb
#define COMMAND_CLEAR_ERRORS               0xc0
#define COMMAND_WRITE_UNIQUE_ID            0xc1
#define COMMAND_AMBIENT_LED                0xc2
#define COMMAND_SET_TEMPERATURES           0xc3
#define COMMAND_SET_D_FEED_RATE_MULTIPLIER 0xc4
#define COMMAND_BUTTON_LED                 0xc5
#define COMMAND_WRITE_HOURS_COUNTER        0xc6
#define COMMAND_SET_E_FEED_RATE_MULTIPLIER 0xc7
#define COMMAND_SET_E_FILAMENT_INFO        0xc8
#define COMMAND_SET_D_FILAMENT_INFO        0xc9
#define COMMAND_SET_HEAD_POWER             0xca
#define COMMAND_MEMORY_RW                  0xf0
#define COMMAND_FORMAT_HEAD_EEPROM         0xf8
#define COMMAND_FORMAT_REEL0_EEPROM        0xf9
#define COMMAND_FORMAT_REEL1_EEPROM        0xfa
#define COMMAND_READ_BENS_INFO             0xfb
#define COMMAND_READ_DEBUG_INFO            0xfc
#define COMMAND_RESET                      0xfd
#define COMMAND_FORMAT_SD_CARD             0xfe
#define COMMAND_ABORT_JOB                  0xff

#define REPORT_FILE_LIST                   0xe0
#define REPORT_STATUS                      0xe1
#define REPORT_HEAD_EEPROM                 0xe2
#define REPORT_ACK_ERRORS                  0xe3
#define REPORT_FIRMWARE_REVISION           0xe4
#define REPORT_UNIQUE_ID                   0xe5
#define REPORT_REEL0_EEPROM                0xe6
#define REPORT_GCODE_RESPONSE              0xe7
#define REPORT_REEL1_EEPROM                0xe8
#define REPORT_SEND_FILE                   0xe9
#define REPORT_HOURS_COUNTER               0xea
#define REPORT_SD_CARD_STATE               0xeb
#define REPORT_DEBUG_INFO                  0xef
#define REPORT_MEMORY_RW                   0xf0
#define REPORT_BENS_INFO                   0xfb

#define ERROR_SD_CARD                      0
#define ERROR_CHUNK_SEQUENCE               1
#define ERROR_FILE_TOO_LARGE               2
#define ERROR_GCODE_LINE_TOO_LONG          3
#define ERROR_USB_RX                       4
#define ERROR_USB_TX                       5
#define ERROR_BAD_COMMAND                  6
#define ERROR_HEAD_EEPROM                  7
#define ERROR_BAD_FIRMWARE_FILE            8
#define ERROR_FLASH_CHECKSUM               9
#define ERROR_GCODE_BUFFER_OVERRUN         10
#define ERROR_FILE_READ_CLOBBERED          11
#define ERROR_MAX_GANTRY_ADJUSTMENT        12
#define ERROR_REEL0_EEPROM                 13
#define ERROR_E_FILAMENT_SLIP              14
#define ERROR_D_FILAMENT_SLIP              15
#define ERROR_NOZZLE_FLUSH_NEEDED          16
#define ERROR_Z_TOP_SWITCH                 17
#define ERROR_B_STUCK                      18
#define ERROR_REEL1_EEPROM                 19
#define ERROR_HEAD_POWER_EEPROM            20
#define ERROR_HEAD_POWER_OVERTEMP          21
#define ERROR_BED_THERMISTOR               22
#define ERROR_NOZZLE0_THERMISTOR           23
#define ERROR_NOZZLE1_THERMISTOR           24
#define ERROR_B_POSITION_LOST              25
#define ERROR_E_LOAD_SLIP                  26
#define ERROR_D_LOAD_SLIP                  27
#define ERROR_E_UNLOAD_SLIP                28
#define ERROR_D_UNLOAD_SLIP                29
#define ERROR_POWEROFF_WHILST_HOT          30
#define ERROR_E_NO_FILAMENT                31
#define ERROR_D_NO_FILAMENT                32
#define ERROR_B_POSITION_WARNING           33
#define ERROR_HEAD_SHORTED                 34
#define ERROR_X_DRIVER                     35
#define ERROR_Y_DRIVER                     36
#define ERROR_ZA_DRIVER                    37
#define ERROR_ZB_DRIVER                    38
#define ERROR_E_DRIVER                     39
#define ERROR_D_DRIVER                     40
#define ERROR_BED_TEMPERATURE_DROOP        41

// These errors must not cause a pause
#define ERROR_PAUSE_MASK                   ((((uint64_t)1) << ERROR_B_POSITION_WARNING) | (((uint64_t)1) << ERROR_B_POSITION_LOST) | (((uint64_t)1) << ERROR_POWEROFF_WHILST_HOT) | (((uint64_t)1) << ERROR_E_NO_FILAMENT) | (((uint64_t)1) << ERROR_D_NO_FILAMENT))
// In addition, these errors must not cause a pause if they occur when Automaker is connected
#define ERROR_PAUSE_MASK_AM                ((((uint64_t)1) << ERROR_NOZZLE_FLUSH_NEEDED) | (((uint64_t)1) << ERROR_D_FILAMENT_SLIP) | (((uint64_t)1) << ERROR_E_FILAMENT_SLIP))
// These errors are cancelled by resuming from the button, but only when Automaker not connected
#define ERROR_RESUMABLE_MASK               ((((uint64_t)1) << ERROR_B_POSITION_WARNING) | (((uint64_t)1) << ERROR_B_POSITION_LOST) | (((uint64_t)1) << ERROR_D_FILAMENT_SLIP) | (((uint64_t)1) << ERROR_E_FILAMENT_SLIP))

#define AUTOMAKER_TIMEOUT                  5.0 // secs; if no error report requested in this time, Automaker is considered to be disconnected

void initialise_commands(void);
void wait_for_vmot(void);
void maintain_commands(void);
unsigned char is_automaker_connected(void);
void report_error(unsigned char error_number);
unsigned char is_error(void);
void clear_resumable_errors(void);
void fake_an_error(char line[]);

#endif
