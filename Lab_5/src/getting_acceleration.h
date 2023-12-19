#ifndef GETTING_ACCELERATION_H
#define GETTING_ACCELERATION_H

#include <stdint.h>

// Initialises I2C bus, uses controller 0
void initI2C();

//Writes one byte into a register
void writeI2C(uint8_t address, uint8_t reg, uint8_t data);

//Reads len bytes and places them into a buffer, buffer must be pre-allocated
int readI2C(uint8_t regLow, uint8_t regHigh);

// Calculate acceleratin 
double calculatedAcceleration();

#endif