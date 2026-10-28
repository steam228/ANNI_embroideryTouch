#pragma once

#include <Arduino.h>

// Binary serial protocol from the Illutron / Dzl 2012 touch sketch.
// Each packet: 0 (start), Command, yMSB, yLSB, xMSB, xLSB, zeroByte, Checksum

extern int sizeOfArray; // defined in main.cpp

void SendData(int Command, unsigned int yValue, unsigned int xValue);
void PlottArray(unsigned int Cmd, float Array1[], float Array2[]);
