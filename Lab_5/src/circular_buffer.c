#include <stdio.h>
#include <limits.h>
#include <stdbool.h>

#include "circular_buffer.h"

void initCircularBuffer(struct circularBuffer* bufferPtr, int* data, int maxLength) 
{
  bufferPtr->data = data;
  bufferPtr->head = 0;
  bufferPtr->tail = -1;
  bufferPtr->maxLength = maxLength;
  bufferPtr->numberOfElements = 0;
}

void addElement(struct circularBuffer* bufferPtr, int value)
{
  if (bufferPtr->numberOfElements < bufferPtr->maxLength)
  {
    // increment tail, modulus if index is above maxLength
    bufferPtr->tail = (bufferPtr->tail + 1) % bufferPtr->maxLength;
    // add data to tail 
    bufferPtr->data[bufferPtr->tail] = value;
    // increase number of elements in the buffer
    bufferPtr->numberOfElements++;
  }
}

void removeHead(struct circularBuffer* bufferPtr)
{
  //only remove if there is an element in the buffer
  if (bufferPtr->numberOfElements != 0)
  {
    // moving head one step, modulus if we are in the end of the array to restart at zero
    bufferPtr->head = (bufferPtr->head + 1) % bufferPtr->maxLength;
    //decrease number of elements in the buffer
    bufferPtr->numberOfElements--;
  }
}

void printBuffer(struct circularBuffer* bufferPtr)
{
  //Only print if elements exist 
  if (bufferPtr->numberOfElements > 0)
  {
    // If head's index is smaller the tail's index
    if (bufferPtr->head <= bufferPtr->tail)
    {
      for (int i = bufferPtr->head; i <= bufferPtr->tail; i++)
      {
        printf("Value: %d - Index: %d\n", bufferPtr->data[i], i);
      }
    }
    else
    {
      //For loop to print from where head is in the array to the last index of the array
      for (int i = bufferPtr->head; i < bufferPtr->maxLength; i++)
      {
        printf("Value: %d - Index: %d\n", bufferPtr->data[i], i);
      }
      // for loop to add upp the missing index in the begining 
      for (int i = 0; i <= bufferPtr->tail; i++)
      {
        printf("Value: %d - Index: %d\n", bufferPtr->data[i], i);
      }
    }
  }
}