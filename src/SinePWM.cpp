/*
 *  SinePWM.cpp
 *
 *  Sketch generates a 62.5 kHz PWM signal resulting in a sine.
 *  Interrupt driven,
 *
 *
 *  Copyright (C) 2017  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "SinePWM.h"

//#define USE_STANDARD_SERIAL

#define TIMER_PRESCALER_MASK 0x07

/*
 * Sine table fom 0 to 90 degree including 0 AND 90 degree therefore we have an odd number
 * gives values from 1 to 255 including
 */
// gives 488 Hz output 1/16 us * 256 * 128 = 16*128 = 2048us = 488.28125 Hz base frequency
#define PERIOD_FOR_SINE_TABLE 2048UL // F_CPU / (PWM_RESOLUTION * SIZE_OF_SINE_TABLE_QUARTER *4)
#define SIZE_OF_SINE_TABLE_QUARTER 32
const uint8_t sSineTableQuarter128[SIZE_OF_SINE_TABLE_QUARTER + 1] PROGMEM = { 128, 135, 141, 147, 153, 159, 165, 171, 177, 182,
        188, 193, 199, 204, 209, 213, 218, 222, 226, 230, 234, 237, 240, 243, 245, 248, 250, 251, 253, 254, 254, 255, 255 };

int8_t sSineTableIndex = 0;
uint8_t sNumberOfSineQuadrant = 0;
uint8_t sNextOcrbValue = 0;
uint32_t sFrequencyFactorShift16; // 0x40000 gives 1953 Hz output 1/16 us * 256 * 32 = 16*32 = 512us = 1953 Hz fastest frequency
int32_t sFrequencyFactorAccumulator = 0; // used to handle fractions of factor above

/*
 * Timer 1 is used by Arduino for Servo Library. For 8 bit you can also use Timer 2 which is used for Arduino Tone().
 */
void initTimer1For8BitPWM() {
//    DDRB |= _BV(DDB1) | _BV(DDB2); // set pins OC1A = PortB1 -> PIN 9 and OC1B = PortB2 -> PIN 10 to output direction
    DDRB |= _BV(DDB2); // set pin OC1B = PortB2 -> PIN 10 to output direction

    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10); // Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode). Waveform Generation Mode 1 - PWM, Phase Correct, 8-bit
//    TCCR1A = _BV(COM1A1) | _BV(COM1B1) _BV(WGM11) || _BV(WGM10); //Waveform Generation Mode 3 - PWM, Phase Correct, 10-bit
    TCCR1B = _BV(WGM12);    // Fast PWM, stop Timer

    OCR1A = 0xFF;   // output HIGH
    OCR1B = 0x8F;   // output HIGH
    TCNT1 = 0;      // init counter
    TIMSK1 = _BV(TOIE1); // Enable Overflow Interrupt
}

/*
 * In order to be exact use periods instead of frequency
 */
long setSineFrequency(uint16_t aFrequencyHz) {
    unsigned long tPeriodMicros = (1000000UL + (aFrequencyHz / 2)) / aFrequencyHz;
    setSinePeriodMicros(tPeriodMicros);
    return tPeriodMicros;
}

/*
 * clip to 8 samples / 128us per period = 7,8125Hz
 */
void setSinePeriodMicros(unsigned long aPeriodMicros) {
    sFrequencyFactorShift16 = (PERIOD_FOR_SINE_TABLE << 16) / aPeriodMicros;
    if (sFrequencyFactorShift16 > 0x100000) {
        sFrequencyFactorShift16 = 0x100000;
    }
    // start timer
    TCCR1B |= _BV(CS10); // set prescaler to 1 -> gives 16us / 62.5kHz PWM
}

void stopSine() {
    // set prescaler choice to 0 -> timer stops
    TCCR1B &= ~TIMER_PRESCALER_MASK;
}

void stopTimer1() {
    // set prescaler choice to 0 -> timer stops
    TCCR1B &= ~TIMER_PRESCALER_MASK;
}

//Timer1 overflow interrupt vector handler
ISR(TIMER1_OVF_vect) {
    // output value at start of ISR to avoid jitter
    OCR1B = sNextOcrbValue;
    int8_t tIndexDelta = sFrequencyFactorShift16 >> 16;
    // handle fraction of frequency Factor
    sFrequencyFactorAccumulator += sFrequencyFactorShift16 & 0xFFFF;
    if (sFrequencyFactorAccumulator > 0x8000) {
        /*
         * Accumulated fraction is bigger than "half" so increase index
         */
        tIndexDelta++;
        sFrequencyFactorAccumulator -= 0x10000;
    }

    uint8_t tQuadrantIncrease = 0;
    switch (sNumberOfSineQuadrant) {
    case 0: // [0,90) Degree
    case 2: // [180,270) Degree
        sSineTableIndex += tIndexDelta;
        if (sSineTableIndex >= SIZE_OF_SINE_TABLE_QUARTER) {
            sSineTableIndex = SIZE_OF_SINE_TABLE_QUARTER - (sSineTableIndex - SIZE_OF_SINE_TABLE_QUARTER);
            tQuadrantIncrease = 1;
        }
        break;
    case 1: // [90,180) Degree
    case 3: // [270,360) Degree
        sSineTableIndex -= tIndexDelta;
        if (sSineTableIndex <= 0) {
            sSineTableIndex = -sSineTableIndex;
            tQuadrantIncrease = 1;
        }
        break;
    }
    if (sNumberOfSineQuadrant & 0x02) {
        // case 2 and 3   -128 = 128 ; -255 = 1
        sNextOcrbValue = -(pgm_read_byte(&sSineTableQuarter128[sSineTableIndex]));
    } else {
        sNextOcrbValue = pgm_read_byte(&sSineTableQuarter128[sSineTableIndex]);
    }

    sNumberOfSineQuadrant = (sNumberOfSineQuadrant + tQuadrantIncrease) & 0x03;

    /*
     * the same as loop with variable delay
     */
    //    // [0,90)
    //    for (int i = 0; i < SIZE_OF_SINE_TABLE_QUARTER; ++i) {
    //        OCR1B = sSineTableQuarter128[i];  // Pin9
    //        delayMicroseconds(sDelay);
    //    }
    //    // [90,180)
    //    for (int i = SIZE_OF_SINE_TABLE_QUARTER; i > 0; i--) {
    //        OCR1B = sSineTableQuarter128[i];  // Pin9
    //        delayMicroseconds(sDelay);
    //    }
    //    // [180,270)
    //    for (int i = 0; i < SIZE_OF_SINE_TABLE_QUARTER; ++i) {
    //        OCR1B = -(sSineTableQuarter128[i]);  // Pin9
    //        delayMicroseconds(sDelay);
    //    }
    //    // [270,360)
    //    for (int i = SIZE_OF_SINE_TABLE_QUARTER; i > 0; i--) {
    //        OCR1B = -(sSineTableQuarter128[i]);  // Pin9
    //        delayMicroseconds(sDelay);
    //    }
}

/*
 * Use it once, if you need a different size of table e.g. to generate different frequencies or increase accuracy for low frequencies
 */
void computeSineTableValues(uint8_t aSineTable[], unsigned int aNumber) {
    //
    float tRadianDelta = (M_PI * 2) / aNumber;
    float tRadian = 0.0;
    // (i <= aNumber) in order to include value for 360 degree
    for (unsigned int i = 0; i < aNumber; ++i) {
        float tSineFloat = (sin(tRadian) * 127) + 128;
        aSineTable[i] = (tSineFloat + 0.5);
        tRadian += tRadianDelta;
    }
}
