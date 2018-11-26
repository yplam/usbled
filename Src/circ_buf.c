//
// Created by yplam on 18-11-26.
//

#include "circ_buf.h"

int circ_buf_pop(circ_buf_t *c, uint8_t *data) {
  int next;

  if (c->head == c->tail)  // if the head == tail, we don't have any data
    return -1;

  next = c->tail + 1;  // next is where tail will point to after this read.
  if(next >= c->maxlen)
    next = 0;

  *data = c->buffer[c->tail];  // Read data and then move
  c->tail = next;              // tail to next offset.
  return 0;  // return success to indicate successful push.
}

int circ_buf_push(circ_buf_t *c, uint8_t data) {
  int next;

  next = c->head + 1;  // next is where head will point to after this write.
  if (next >= c->maxlen)
    next = 0;

  // if the head + 1 == tail, circular buffer is full. Notice that one slot
  // is always left empty to differentiate empty vs full condition
  if (next == c->tail)
    return -1;

  c->buffer[c->head] = data;  // Load data and then move
  c->head = next;             // head to next data offset.
  return 0;  // return success to indicate successful push.
}

int circ_buf_free_space(circ_buf_t *c) {
  int freeSpace;
  freeSpace = c->tail - c->head;
  if (freeSpace <= 0)
    freeSpace += c->maxlen;
  return freeSpace - 1; // -1 to account for the always-empty slot.
}

int circ_buf_len(circ_buf_t *c) {
  int len ;
  len = c->head - c->tail;
  if(len < 0){
    len += c->maxlen;
  }
  return len;
}
