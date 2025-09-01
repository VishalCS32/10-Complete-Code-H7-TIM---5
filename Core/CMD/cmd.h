/*
 * cmd.h
 *
 *  Created on: Jun 13, 2025
 *      Author: vishal
 */

#ifndef CMD_H
#define CMD_H

#include "main.h"
#include <stdbool.h>
#include <string.h>
//#include "../FILTERS/filters.h" // Added: For filter_state_t in calibration commands

/* Defines */
#define CMD_BUFFER_SIZE 64
#define CMD_TIMEOUT_MS  200  // Timeout after 200ms

/* Command buffer and state variables */
extern volatile char cmd_buffer[CMD_BUFFER_SIZE];
extern volatile uint8_t cmd_index;
extern volatile bool cmd_receiving;
extern volatile uint32_t last_char_time; // In ms

/* Command mode flag */
extern volatile uint8_t cmd_mode; // Added: Flag to indicate command mode (0 = normal, 1 = command mode)

/* Function prototypes */
uint8_t is_cmd_mode(void); // Added: Check command mode
void strip_brackets(char *str);
void process_command(char *cmd);

#endif /* CMD_H */
