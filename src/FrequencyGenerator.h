/*
 * FrequencyGenerator.h
 *
 *
 *  Created on:  01.01.2015
 *      Author:  Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmx.de
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *      Version: 1.0.0
 */

#ifndef FREQUENCYGENERATOR_H_
#define FREQUENCYGENERATOR_H_

#include <inttypes.h>
#include <avr/pgmspace.h>

void initFrequency(void);
void initFrequencyPage(void);
void drawFrequencyGui(void);

extern uint8_t TouchButtonFrequency;
extern const char StringStop[] PROGMEM; // "Stop"

#endif //FREQUENCYGENERATOR_H_
