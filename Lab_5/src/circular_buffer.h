#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

// Data structure used to hold a circular buffer.
struct circularBuffer{
  int * data;
  int head;
  int tail;
  int maxLength;
  int numberOfElements;
};

void initCircularBuffer(struct circularBuffer *bufferPtr, int *data, int maxLength);

void addElement(struct circularBuffer* bufferPtr, int value);

void removeHead(struct circularBuffer* bufferPtr);

void printBuffer(struct circularBuffer *bufferPtr);

#endif

