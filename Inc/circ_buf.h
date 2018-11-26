//
// Created by yplam on 18-11-26.
//

#ifndef USBLED_CIRC_BUF_H
#define USBLED_CIRC_BUF_H
#include <stdint.h>

typedef struct {
    uint8_t * const buffer;
    int head;
    int tail;
    const int maxlen;
} circ_buf_t;

#define CIRC_BUF_DEF(x,y)                \
    uint8_t x##_data_space[y+1];          \
    circ_buf_t x = {                     \
        .buffer = x##_data_space,         \
        .head = 0,                        \
        .tail = 0,                        \
        .maxlen = y+1                     \
    }

/*
 * Method: circ_buf_pop
 * Returns:
 *  0 - Success
 * -1 - Empty
 */
int circ_buf_pop(circ_buf_t *c, uint8_t *data);

/*
 * Method: circ_buf_push
 * Returns:
 *  0 - Success
 * -1 - Out of space
 */
int circ_buf_push(circ_buf_t *c, uint8_t data);

/*
 * Method: circ_bbuf_free_space
 * Returns: number of bytes available
 */
int circ_buf_free_space(circ_buf_t *c);

/*
 * Method: circ_buf_len
 * Returns: number of bytes available
 */
int circ_buf_len(circ_buf_t *c);

#endif //USBLED_CIRC_BUF_H
