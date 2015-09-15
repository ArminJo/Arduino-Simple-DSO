/*
 *
 *  Created on: 01.01.2015
 *      Author: Armin Joachimsmeyer
 *      Email: armin.joachimsmeyer@gmail.com
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 *      Sources: http://code.google.com/p/simple-touch-screen-dso-software/
 *
 *      Features:
 *
 */

#include "FrequencyGenerator.h"
#include "SimpleTouchScreenDSO.h"

#include "BlueDisplay.h"
#include "EventHandler.h"

#include <stdio.h>   // for printf
#include <math.h>   // for pow and log10f
#include <stdlib.h> // for dtostrf

#define COLOR_BACKGROUND_FREQ COLOR_BACKGROUND_DSO

#define TIMER_PRESCALER_64 0x03
#define TIMER_PRESCALER_MASK 0x07

#define NUMBER_OF_FIXED_FREQUENCY_BUTTONS 10
#define NUMBER_OF_FREQUENCY_RANGE_BUTTONS 5

#define FREQ_SLIDER_SIZE 10 // width of bar / border
#define FREQ_SLIDER_MAX_VALUE 300 /* length of bar */

#define FREQ_SLIDER_X 5
#define FREQ_SLIDER_Y (5 * TEXT_SIZE_11_HEIGHT + 6)

const char StringStop[] PROGMEM = "Stop";

const uint16_t Frequency[NUMBER_OF_FIXED_FREQUENCY_BUTTONS] PROGMEM = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 };

// To have a value available on entering page
static float sFrequency = 20;
static int16_t sSliderValue = 100;
static const int BUTTON_INDEX_SELECTED_INITIAL = 3; // select kHz Button

const char StringmHz[] PROGMEM = "mHz";
const char StringHz[] PROGMEM = "Hz";
const char String10Hz[] PROGMEM = "10Hz";
const char StringkHz[] PROGMEM = "kHz";
const char StringMHz[] PROGMEM = "MHz";

const char* FrequencyButtonStrings[5] = { StringmHz, StringHz, String10Hz, StringkHz, StringMHz };
#define INDEX_OF_10HZ 2

const char FrequencyFactorChars[4] = { 'm', ' ', 'k', 'M' };
static uint8_t sFrequencyFactorIndex; // 0->mHz, 1->Hz, 2->kHz, 3->MHz
// factor for mHz/Hz/kHz/MHz - times 1000 because of mHz handling
// 1 -> 1 mHz, 1000 -> 1 Hz, 1000000 -> 1 kHz
static uint32_t sFrequencyFactorTimes1000;
static bool is10HzRange = false;

BDButton TouchButtonFrequencyPage;
BDButton TouchButtonFrequencyRanges[NUMBER_OF_FREQUENCY_RANGE_BUTTONS];
BDButton ActiveTouchButtonFrequencyRange; // Used to determine which range button is active

BDButton TouchButtonFirstFixedFrequency;
BDButton TouchButtonFrequencyStartStop;
BDButton TouchButtonGetFrequency;

BDSlider TouchSliderFrequency;

void doShowFrequencyPage(BDButton * aTheTouchedButton, int16_t aValue);

void doFrequencySlider(BDSlider * aTheTouchedSlider, uint16_t aValue);

void doSetFixedFrequency(BDButton * aTheTouchedButton, int16_t aValue);
void doChangeFrequencyRange(BDButton * aTheTouchedButton, int16_t aValue);
void doFrequencyGeneratorStartStop(BDButton * aTheTouchedButton, int16_t aValue);
void doGetFrequency(BDButton * aTheTouchedButton, int16_t aValue);
bool ComputePeriodAndSetTimer(bool aSetSlider);
void initTimer1(void);
void setFrequencyFactor(int aIndexValue);

/***********************
 * Code starts here
 ***********************/
void initFrequencyGenerator(void) {
    initTimer1();
}

void initFrequencyGeneratorPage(void) {
    setFrequencyFactor(2); // for kHz range
    // Button for chart history (erase color)
    TouchButtonFrequencyPage.initPGM(0, BUTTON_HEIGHT_4_256_LINE_4,
    BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256, COLOR_RED, PSTR("Frequency"), TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0,
            &doShowFrequencyPage);

    TouchSliderFrequency.init(FREQ_SLIDER_X, FREQ_SLIDER_Y, FREQ_SLIDER_SIZE, FREQ_SLIDER_MAX_VALUE,
    FREQ_SLIDER_MAX_VALUE, 0, COLOR_BLUE, COLOR_GREEN, FLAG_SLIDER_SHOW_BORDER | FLAG_SLIDER_IS_HORIZONTAL, &doFrequencySlider);

    // Fixed frequency buttons next
    uint16_t tXPos = 0;
    uint16_t tFrequency;
    const uint16_t * tFrequencyPtr = &Frequency[0];
    for (uint8_t i = 0; i < NUMBER_OF_FIXED_FREQUENCY_BUTTONS; ++i) {
        tFrequency = pgm_read_word(tFrequencyPtr);
        sprintf_P(StringBuffer, PSTR("%u"), tFrequency);
        TouchButtonFirstFixedFrequency.init(tXPos,
                DISPLAY_HEIGHT - BUTTON_HEIGHT_4 - BUTTON_HEIGHT_5 - BUTTON_HEIGHT_6 - 2 * BUTTON_DEFAULT_SPACING,
                BUTTON_WIDTH_10, BUTTON_HEIGHT_6, COLOR_BLUE, StringBuffer, TEXT_SIZE_11, 0, tFrequency, &doSetFixedFrequency);
        tXPos += BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER;
        tFrequencyPtr++;
    }
    TouchButtonFirstFixedFrequency.mButtonHandle -= NUMBER_OF_FIXED_FREQUENCY_BUTTONS - 1;

    // Range next
    tXPos = 0;
    int tYPos = BlueDisplay1.getDisplayHeight() - BUTTON_HEIGHT_4 - BUTTON_HEIGHT_5 - BUTTON_DEFAULT_SPACING;
    for (int i = 0; i < NUMBER_OF_FREQUENCY_RANGE_BUTTONS; ++i) {
        uint16_t tButtonColor = BUTTON_AUTO_RED_GREEN_FALSE_COLOR;
        if (i == BUTTON_INDEX_SELECTED_INITIAL) {
            tButtonColor = BUTTON_AUTO_RED_GREEN_TRUE_COLOR;
        }
        TouchButtonFrequencyRanges[i].initPGM(tXPos, tYPos, BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING_HALF,
        BUTTON_HEIGHT_5, tButtonColor, FrequencyButtonStrings[i],
        TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, i, &doChangeFrequencyRange);
        if (i == BUTTON_INDEX_SELECTED_INITIAL) {
            ActiveTouchButtonFrequencyRange = TouchButtonFrequencyRanges[i];
        }
        tXPos += BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING - 2;
    }

    TouchButtonFrequencyStartStop.initPGM(0, LAYOUT_256_HEIGHT - BUTTON_HEIGHT_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_GREEN, StringStop, TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH | BUTTON_FLAG_TYPE_AUTO_RED_GREEN, true,
            &doFrequencyGeneratorStartStop);

    TouchButtonGetFrequency.initPGM(BUTTON_WIDTH_3_POS_2, LAYOUT_256_HEIGHT - BUTTON_HEIGHT_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Hz..."), TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, true, &doGetFrequency);
}

void startFrequencyGeneratorPage(void) {
    drawFrequencyGeneratorPage();
}

// not used here
void loopFrequencyGeneratorPage(void) {
    checkAndHandleEvents();
}
void stopFrequencyGeneratorPage(void) {
}

void drawFrequencyGeneratorPage(void) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_FREQUENCY;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_FREQ);

    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    TouchButtonBack.drawButton();

    TouchSliderFrequency.drawSlider();
    BlueDisplay1.drawTextPGM(TEXT_SIZE_11_WIDTH, FREQ_SLIDER_Y + 3 * FREQ_SLIDER_SIZE + TEXT_SIZE_11_HEIGHT, PSTR("1"),
    TEXT_SIZE_11, COLOR_BLUE, COLOR_BACKGROUND_FREQ);
    BlueDisplay1.drawTextPGM(DISPLAY_WIDTH - 5 * TEXT_SIZE_11_WIDTH, FREQ_SLIDER_Y + 3 * FREQ_SLIDER_SIZE + TEXT_SIZE_11_HEIGHT,
            PSTR("1000"), TEXT_SIZE_11, COLOR_BLUE, COLOR_BACKGROUND_FREQ);

    // fixed frequency buttons
    // we know that the buttons handles are increasing numbers
    BDButton tButton(TouchButtonFirstFixedFrequency);
    for (uint8_t i = 0; i < NUMBER_OF_FIXED_FREQUENCY_BUTTONS; ++i) {
        tButton.drawButton();
        tButton.mButtonHandle++;
    }

    for (uint8_t i = 0; i < NUMBER_OF_FREQUENCY_RANGE_BUTTONS; ++i) {
        TouchButtonFrequencyRanges[i].drawButton();
    }

    TouchButtonFrequencyStartStop.drawButton();
    TouchButtonGetFrequency.drawButton();

    // show values
    ComputePeriodAndSetTimer(true);
}

/**
 * show gui of settings screen
 */
void doShowFrequencyPage(BDButton * aTheTouchedButton, int16_t aValue) {
    startFrequencyGeneratorPage();
}

/**
 * changes the unit (mHz - MHz)
 * set color for old and new button
 */
void doChangeFrequencyRange(BDButton * aTheTouchedButton, int16_t aValue) {
    if (ActiveTouchButtonFrequencyRange != *aTheTouchedButton) {
        ActiveTouchButtonFrequencyRange.setButtonColorAndDraw( BUTTON_AUTO_RED_GREEN_FALSE_COLOR);
        ActiveTouchButtonFrequencyRange = *aTheTouchedButton;
        aTheTouchedButton->setButtonColorAndDraw( BUTTON_AUTO_RED_GREEN_TRUE_COLOR);
        // Handling of 10 Hz button
        if (aValue == INDEX_OF_10HZ) {
            is10HzRange = true;
        } else {
            is10HzRange = false;
        }
        if (aValue >= INDEX_OF_10HZ) {
            aValue--;
        }
        setFrequencyFactor(aValue);
        ComputePeriodAndSetTimer(true);
    }
}

/**
 * Set frequency to fixed value 1,2,5,10...,1000
 */
void doSetFixedFrequency(BDButton * aTheTouchedButton, int16_t aValue) {
    sFrequency = aValue;
    BlueDisplay1.playFeedbackTone(ComputePeriodAndSetTimer(true));
}

/**
 * Handler for number receive event - set frequency to value
 */
void doSetFrequency(float aValue) {
    uint8_t tIndex = 1;
    while (aValue > 1000) {
        aValue /= 1000;
        tIndex++;
    }
    if (aValue < 1) {
        tIndex = 0; //mHz
        aValue *= 1000;
    }
    setFrequencyFactor(tIndex);
    sFrequency = aValue;
    BlueDisplay1.playFeedbackTone(ComputePeriodAndSetTimer(true));
}

/**
 * Request frequency numerical
 */
void doGetFrequency(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPromptPGM(&doSetFrequency, PSTR("frequency [Hz]"));
}

void doFrequencyGeneratorStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    aValue = !aValue;
    if (aValue) {
        // Start timer and print green stop button
        TouchButtonFrequencyStartStop.setCaptionPGM(StringStop);
        ComputePeriodAndSetTimer(true);
    } else {
        // Stop timer and print red start button
        TouchButtonFrequencyStartStop.setCaptionPGM(PSTR("Start"));
        TCCR1B &= ~TIMER_PRESCALER_MASK;
    }
    TouchButtonFrequencyStartStop.setValueAndDraw(aValue);
}

void doFrequencySlider(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    float tValue = aValue;
    tValue = tValue / (FREQ_SLIDER_MAX_VALUE / 3); // gives 0-3
    // 950 byte program space needed for pow() and log10f()
    sFrequency = pow(10, tValue);
    if (is10HzRange) {
        sFrequency *= 10;
    }
    ComputePeriodAndSetTimer(false);
}

void setFrequencyFactor(int aIndexValue) {
    sFrequencyFactorIndex = aIndexValue;
    uint32_t tFactor = 1;
    while (aIndexValue > 0) {
        tFactor *= 1000;
        aIndexValue--;
    }
    sFrequencyFactorTimes1000 = tFactor;
}

/**
 * Computes Autoreload value for synthesizer from 8,381 mHz (0xFFFFFFFF) to 18MHz (0x02) and prints frequency value
 * @param aSetSlider
 * @return true if error happened
 */
bool ComputePeriodAndSetTimer(bool aSetSlider) {
    bool tIsError = false;
    /*
     * Times 500 would be correct because timer runs in toggle mode and has 8 MHz
     * But F_CPU * 500 does not fit in a 32 bit integer so use half of it which fits
     */
    uint32_t tDividerInt = ((F_CPU * 250) / sFrequencyFactorTimes1000);
    float tFrequency = sFrequency;
    if (tDividerInt > 0x7FFFFFFF) {
        tFrequency /= 2;
    } else {
        tDividerInt *= 2;
    }
    uint32_t tDividerSave = tDividerInt;
    tDividerInt /= tFrequency;

    if (tDividerInt == 0) {
        // 8 Mhz / 0.125 us is Max
        tIsError = true;
        tDividerInt = 1;
        tFrequency = 8;
    }

    // determine prescaler
    uint8_t tPrescalerHWValue = 1; // direct clock
    uint16_t tPrescaler = 1;
    if (tDividerInt > 0x10000) {
        tDividerInt >>= 3;
        if (tDividerInt <= 0x10000) {
            tPrescaler = 8;
            tPrescalerHWValue = 2;
        } else {
            tDividerInt >>= 3;
            if (tDividerInt <= 0x10000) {
                tPrescaler = 64;
                tPrescalerHWValue = 3;
            } else {
                tDividerInt >>= 2;
                if (tDividerInt <= 0x10000) {
                    tPrescaler = 256;
                    tPrescalerHWValue = 4;
                } else {
                    tDividerInt >>= 2;
                    tPrescaler = 1024;
                    tPrescalerHWValue = 5;
                    if (tDividerInt > 0x10000) {
                        // clip to 16 bit value
                        tDividerInt = 0x10000;
                    }
                }
            }
        }
    }
    TCCR1B &= ~TIMER_PRESCALER_MASK;
    TCCR1B |= tPrescalerHWValue;
    OCR1A = tDividerInt - 1; // set compare match register

    // recompute exact period and frequency for given integer period
    tDividerInt *= tPrescaler;
    // output period
    if (tDividerInt < F_CPU / 2) {
        float tPeriod = tDividerInt;
        tPeriod /= 8;
        dtostrf(tPeriod, 10, 3, &StringBuffer[20]);
        sprintf_P(StringBuffer, PSTR("%s\xB5s"), &StringBuffer[20]); //micro seconds
    } else {
        sprintf_P(StringBuffer, PSTR("%10lums"), (tDividerInt / (F_CPU / 2000)));
    }
    BlueDisplay1.drawText(TEXT_SIZE_22_WIDTH, FONT_SIZE_INFO_SHORT + TEXT_SIZE_22_ASCEND + TEXT_SIZE_22_HEIGHT, StringBuffer, 16,
    COLOR_BLUE, COLOR_BACKGROUND_FREQ);

    // output frequency
    tFrequency = tDividerSave;
    tFrequency /= tDividerInt;
    if (tDividerSave > 0x7FFFFFFF) {
        tFrequency *= 2;
    }
    sFrequency = tFrequency;

    dtostrf(tFrequency, 9, 3, &StringBuffer[20]);
    sprintf_P(StringBuffer, PSTR("%s%cHz"), &StringBuffer[20], FrequencyFactorChars[sFrequencyFactorIndex]);
    // just below DSO info line
    BlueDisplay1.drawText(2 * TEXT_SIZE_22_WIDTH, FONT_SIZE_INFO_SHORT + TEXT_SIZE_22_ASCEND, StringBuffer, TEXT_SIZE_22, COLOR_RED,
    COLOR_BACKGROUND_FREQ);

    sSliderValue = log10(tFrequency) * 100;
    if (aSetSlider) {
        // 950 byte program space needed for pow() and log10f()
        TouchSliderFrequency.setActualValueAndDrawBar(sSliderValue);
    }
    return tIsError;
}

void initTimer1(void) {
    // initialization with 0 is essential otherwise timer will not work correctly!!!
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B

    TIMSK1 = 0; // no interrupts
    TCNT1 = 0; // init counter
    OCR1A = 125 - 1; // set compare match register for 1kHz

    TCCR1A = (1 << COM1B0); // Toggle OC1B on compare match / CTC mode
    TCCR1B = (1 << WGM12); // CTC with OCR1A - no clock->timer disabled
}
