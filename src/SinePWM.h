/*
 * SinePWM.h
 *
 *  Created on: 24.03.2017
 *      Author: Armin
 */

#ifndef SRC_LIB_SINEPWM_H_
#define SRC_LIB_SINEPWM_H_

void initTimer1For8BitPWM();
long setSineFrequency(uint16_t aFrequencyHz);
void setSinePeriodMicros(unsigned long aPeriodMicros);

void stopSine();
void stopTimer1();

// utility Function
void computeSineTableValues(uint8_t aSineTable[], unsigned int aNumber);

#endif /* SRC_LIB_SINEPWM_H_ */
