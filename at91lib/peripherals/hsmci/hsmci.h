#ifndef HSMCI_H
#define HSMCI_H

#include "sd_mmc_protocol.h"


/** \brief Initializes the low level driver
 *
 * This enable the clock required and the hardware interface.
 */
void hsmci_init(void);

/**
 * \brief Select a slot and initialize it
 *
 * \param clock      Maximum clock to use (Hz)
 * \param bus_width  Bus width to use (1, 4 or 8)
 * \param high_speed true, to enable high speed mode
 */
void hsmci_select_device(unsigned long clock, unsigned char bus_width, unsigned char high_speed);

/** \brief Send 74 clock cycles on the line of selected slot
 * Note: It is required after card plug and before card install.
 */
void hsmci_send_clock(void);

/** \brief Send a command on the selected slot
 *
 * \param cmd        Command definition
 * \param arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
unsigned char hsmci_send_cmd(sdmmc_cmd_def_t cmd, unsigned long arg);

/** \brief Return the 32 bits response of the last command
 *
 * \return 32 bits response
 */
unsigned long hsmci_get_response(void);

/** \brief Return the 128 bits response of the last command
 *
 * \param response   Pointer on the array to fill with the 128 bits response
 */
void hsmci_get_response_128(unsigned char* response);

/** \brief Send an ADTC command on the selected slot
 * An ADTC (Addressed Data Transfer Commands) command is used
 * for read/write access.
 *
 * \param cmd          Command definition
 * \param arg          Argument of the command
 * \param block_size   Block size used for the transfer
 * \param nb_block     Total number of block for this transfer
 * \param access_block if true, the x_read_blocks() and x_write_blocks()
 * functions must be used after this function.
 * If false, the mci_read_word() and mci_write_word()
 * functions must be used after this function.
 *
 * \return true if success, otherwise false
 */
unsigned char hsmci_adtc_start(sdmmc_cmd_def_t cmd, unsigned long arg, unsigned short block_size, unsigned short nb_block, unsigned char access_block);

/** \brief Send a command to stop an ADTC command on the selected slot
 *
 * \param cmd        Command definition
 * \param arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
unsigned char hsmci_adtc_stop(sdmmc_cmd_def_t cmd, unsigned long arg);

/** \brief Read a word on the line
 *
 * \param value  Pointer on a word to fill
 *
 * \return true if success, otherwise false
 */
unsigned char hsmci_read_word(unsigned long* value);

/** \brief Write a word on the line
 *
 * \param value  Word to send
 *
 * \return true if success, otherwise false
 */
unsigned char hsmci_write_word(unsigned long value);

#endif
