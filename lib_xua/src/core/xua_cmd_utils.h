// Copyright 2025 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/**
 * @file xua_cmd_utils.h
 * @brief Common utility functions for XUA command communication
 */

#ifndef _XUA_CMD_UTILS_H_
#define _XUA_CMD_UTILS_H_

#include <xs1.h>

/**
 * @brief Structure to hold a command with two data words.
 * This is the format of commands that get forwarded from USB (ep0 or ep_buffer) to audio (audiohub).
 */
#define CMD_MAX_DATA_WORDS (2)
typedef struct {
    unsigned cmd;
    unsigned data[CMD_MAX_DATA_WORDS];
} xua_cmd_t;

/**
 * @brief Send a command with two data words over a channel
 * @param c Chanend to send the command on
 * @param cmd_struct Pointer to command structure
 */
static inline void xua_send_cmd(chanend c, const xua_cmd_t *cmd_struct)
{
    outct(c, cmd_struct->cmd);
    outuint(c, cmd_struct->data[0]);
    outuint(c, cmd_struct->data[1]);
}

#ifdef __XC__
/**
 * @brief Receive a command with two data words over a channel
 * @param c Chanend to receive the command on
 * @param cmd_struct Pointer to command structure
 */
static inline void xua_receive_cmd(chanend c, xua_cmd_t *cmd_struct)
{
    cmd_struct->cmd = inct(c);
    cmd_struct->data[0] = inuint(c);
    cmd_struct->data[1] = inuint(c);
}

static inline void xua_receive_cmd_only_data(chanend c, xua_cmd_t *cmd_struct)
{
    cmd_struct->data[0] = inuint(c);
    cmd_struct->data[1] = inuint(c);
}
#endif

#endif /* _XUA_CMD_UTILS_H_ */
