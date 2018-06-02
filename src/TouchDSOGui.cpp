/*
 * TouchDSOGui.cpp
 *
 * Implements the common (GUI) parts of AVR and ARM development
 *
 *  Copyright (C) 2017  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Simple-DSO https://github.com/ArminJo/Arduino-Simple-DSO.
 *
 *  Arduino-Simple-DSO is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */
#ifdef AVR
#else
#endif

#ifdef AVR
#include <Arduino.h>
#include "SimpleTouchScreenDSO.h"
#include "digitalWriteFast.h"
#else
#include "Pages.h"
#include "TouchDSO.h"

#include "Chart.h" // for adjustIntWithScaleFactor()
#endif

#include "BlueDisplay.h"

uint8_t sLastPickerValue;

#define MIN_SAMPLES_PER_PERIOD_FOR_RELIABLE_FREQUENCY_VALUE 3

/************************************************************************
 * Data analysis section
 ************************************************************************/
#ifndef AVR
/**
 * Get max and min for display and automatic triggering.
 */
void computeMinMax(void) {
    uint16_t tMax, tMin;

    uint16_t * tDataBufferPointer = DataBufferControl.DataBufferDisplayStart
    + adjustIntWithScaleFactor(DisplayControl.DatabufferPreTriggerDisplaySize, DisplayControl.XScale);
    if (DataBufferControl.DataBufferEndPointer <= tDataBufferPointer) {
        return;
    }
    uint16_t tAcquisitionSize = DataBufferControl.DataBufferEndPointer + 1 - tDataBufferPointer;

    tMax = *tDataBufferPointer;
    if (MeasurementControl.isEffectiveMinMaxMode) {
        tMin = *(tDataBufferPointer + DATABUFFER_MIN_OFFSET);
    } else {
        tMin = *tDataBufferPointer;
    }

    for (int i = 0; i < tAcquisitionSize; ++i) {
        /*
         * get new Min and Max
         */
        uint16_t tValue = *tDataBufferPointer;
        if (tValue > tMax) {
            tMax = tValue;
        }
        if (MeasurementControl.isEffectiveMinMaxMode) {
            uint16_t tValueMin = *(tDataBufferPointer + DATABUFFER_MIN_OFFSET);
            if (tValueMin < tMin) {
                tMin = tValueMin;
            }
        } else {
            if (tValue < tMin) {
                tMin = tValue;
            }
        }

        tDataBufferPointer++;
    }

    MeasurementControl.RawValueMin = tMin;
    MeasurementControl.RawValueMax = tMax;
}
#endif

void computePeriodFrequency(void) {
#ifdef AVR
    /*
     * Scan data buffer (8 Bit display (!inverted!) values) for trigger conditions.
     * Assume that first value is the first valid after a trigger
     * (which does not hold for fast timebases and delayed or external trigger)
     */
    uint8_t *tDataBufferPointer = &DataBufferControl.DataBuffer[0];
    uint8_t tValue;
    int16_t tCount;
    uint16_t tStartPositionForPulsPause = 0;
    uint16_t tFirstEndPositionForPulsPause = 0;
    uint16_t tCountPosition = 0;
    uint16_t i = 0;
    uint8_t tTriggerStatus = TRIGGER_STATUS_START;
    uint8_t tActualCompareValue = getDisplayFromRawInputValue(MeasurementControl.RawTriggerLevelHysteresis);
    uint8_t tActualCompareValueForFirstPeriod = getDisplayFromRawInputValue(MeasurementControl.RawTriggerLevel);
//    BlueDisplay1.debug("trgr=", MeasurementControl.RawTriggerLevel);
//    BlueDisplay1.debug("trg=",getDisplayFromRawValue( MeasurementControl.RawTriggerLevel));
//    BlueDisplay1.debug("tcmp=", tActualCompareValue);
//    BlueDisplay1.debug("data5=", DataBufferControl.DataBuffer[5]);

    MeasurementControl.PeriodFirst = 0;
    MeasurementControl.PeriodSecond = 0;

    if (MeasurementControl.TriggerMode >= TRIGGER_MODE_FREE || MeasurementControl.TriggerDelayMode != TRIGGER_DELAY_NONE) {
        /*
         * For TRIGGER_MODE_FREE, TRIGGER_MODE_EXTERN or delayed trigger first display value is not any trigger,
         * so start with search for begin of period.
         */
        tCount = -1;
    } else {
        // First display value is the first after triggering condition so we have at least one trigger, but still no period.
        tCount = 0;
    }
    for (; i < REMOTE_DISPLAY_WIDTH; ++i) {
#else
        /**
         * Get period and frequency and average for display
         *
         * Use databuffer and only post trigger area!
         * For frequency use only max values!
         */
        uint16_t * tDataBufferPointer = DataBufferControl.DataBufferDisplayStart
        + adjustIntWithScaleFactor(DisplayControl.DatabufferPreTriggerDisplaySize, DisplayControl.XScale);
        if (DataBufferControl.DataBufferEndPointer <= tDataBufferPointer) {
            return;
        }
        uint16_t tAcquisitionSize = DataBufferControl.DataBufferEndPointer + 1 - tDataBufferPointer;

        uint16_t tValue;
        uint32_t tIntegrateValue = 0;
        uint32_t tIntegrateValueForTotalPeriods = 0;
        int tCount = 0;
        uint16_t tStartPositionForPulsPause = 0;
        uint16_t tFirstEndPositionForPulsPause = 0;
        int tCountPosition = 0;
        int tPeriodDelta = 0;
        int tPeriodMin = 1024;
        int tPeriodMax = 0;
        int tTriggerStatus = TRIGGER_STATUS_START;
        uint16_t tActualCompareValue = MeasurementControl.RawTriggerLevelHysteresis;
        uint16_t tActualCompareValueForFirstPeriod = MeasurementControl.RawTriggerLevel;

        bool tReliableValue = true;

        /*
         * Trigger condition and average taken only from entire periods
         * Use only max value for period
         */
        for (int i = 0; i < tAcquisitionSize; ++i) {
#endif
        tValue = *tDataBufferPointer;

        bool tValueGreaterRefForFirstPeriod;
        if (tFirstEndPositionForPulsPause == 00 && tCount == 0) {
            tValueGreaterRefForFirstPeriod = (tValue > tActualCompareValueForFirstPeriod);
#ifdef AVR
            // toggle compare result if TriggerSlopeRising == true since all values are inverted Display values we have to use just the inverted condition
            tValueGreaterRefForFirstPeriod = tValueGreaterRefForFirstPeriod ^ MeasurementControl.TriggerSlopeRising;
#else
            // toggle compare result if TriggerSlopeRising == false
            tValueGreaterRefForFirstPeriod = tValueGreaterRefForFirstPeriod ^ (!MeasurementControl.TriggerSlopeRising);
#endif
        }

        bool tValueGreaterRef = (tValue > tActualCompareValue);
#ifdef AVR
        // toggle compare result if TriggerSlopeRising == true since all values are inverted Display values we have to use just the inverted condition
        tValueGreaterRef = tValueGreaterRef ^ MeasurementControl.TriggerSlopeRising;
#else
        // toggle compare result if TriggerSlopeRising == false
        tValueGreaterRef = tValueGreaterRef ^ (!MeasurementControl.TriggerSlopeRising);
#endif
        /*
         * First value is the first after triggering condition (including delay)
         */
        if (tTriggerStatus == TRIGGER_STATUS_START) {
            // rising slope - wait for value below 1. threshold
            // falling slope - wait for value above 1. threshold
            if (tFirstEndPositionForPulsPause == 00 && tCount == 0) {
                if (!tValueGreaterRefForFirstPeriod) {
                    tFirstEndPositionForPulsPause = i;
                    MeasurementControl.PeriodFirst = getMicrosFromHorizontalDisplayValue(i - tStartPositionForPulsPause, 1);
                }
            }
            if (!tValueGreaterRef) {
                tTriggerStatus = TRIGGER_STATUS_BEFORE_THRESHOLD;
#ifdef AVR
                tActualCompareValue = getDisplayFromRawInputValue(MeasurementControl.RawTriggerLevel);
#else
                tActualCompareValue = MeasurementControl.RawTriggerLevel;
#endif
            }
        } else {
            // rising slope - wait for value to rise above 2. threshold
            // falling slope - wait for value to go below 2. threshold
            if (tValueGreaterRef) {
#ifdef AVR
                tTriggerStatus = TRIGGER_STATUS_START;
                tActualCompareValue = getDisplayFromRawInputValue(MeasurementControl.RawTriggerLevelHysteresis);
#else
                if ((tPeriodDelta) < MIN_SAMPLES_PER_PERIOD_FOR_RELIABLE_FREQUENCY_VALUE) {
                    // found new trigger in less than MIN_SAMPLES_PER_PERIOD_FOR_RELIABLE_FREQUENCY_VALUE samples => no reliable value
                    tReliableValue = false;
                } else {
                    // search for next slope
                    tTriggerStatus = TRIGGER_STATUS_START;
                    tActualCompareValue = MeasurementControl.RawTriggerLevelHysteresis;
                    if (tPeriodDelta < tPeriodMin) {
                        tPeriodMin = tPeriodDelta;
                    } else if (tPeriodDelta > tPeriodMax) {
                        tPeriodMax = tPeriodDelta;
                    }
                    tPeriodDelta = 0;
                    // found and search for next slope
                    tIntegrateValueForTotalPeriods = tIntegrateValue;
#endif
                tCount++;
                if (tCount == 0) {
                    // set start position for TRIGGER_MODE_FREE, TRIGGER_MODE_EXTERN or delayed trigger.
                    tStartPositionForPulsPause = i;
                } else if (tCount == 1) {
                    MeasurementControl.PeriodSecond = getMicrosFromHorizontalDisplayValue(i - tFirstEndPositionForPulsPause, 1);
                }
                tCountPosition = i;
#ifndef AVR
            }
#endif
            }
        }
#ifndef AVR
        if (MeasurementControl.isEffectiveMinMaxMode) {
            uint16_t tValueMin = *(tDataBufferPointer + DATABUFFER_MIN_OFFSET);
            tIntegrateValue += (tValue + tValueMin) / 2;

        } else {
            tIntegrateValue += tValue;
        }
        tPeriodDelta++;
#endif
        tDataBufferPointer++;
    } // for
#ifndef AVR
    /*
     * check for plausi of period values
     * allow delta of periods to be at least 1/8 period + 3
     */
    tPeriodDelta = tPeriodMax - tPeriodMin;
    if (((tCountPosition / (8 * tCount)) + 3) < tPeriodDelta) {
        tReliableValue = false;
    }
#endif

    /*
     * compute period and frequency
     */
#ifdef AVR
    if (tCount <= 0) {
#else
        if (tCountPosition <= 0 || tCount <= 0 || !tReliableValue) {
            MeasurementControl.FrequencyHertz = 0;
            MeasurementControl.RawValueAverage = (tIntegrateValue + (tAcquisitionSize / 2)) / tAcquisitionSize;
#endif
        MeasurementControl.PeriodMicros = 0;
    } else {
#ifdef AVR
        tCountPosition -= tStartPositionForPulsPause;
        if (MeasurementControl.TimebaseIndex <= TIMEBASE_INDEX_FAST_MODES + 1) {
            tCountPosition++; // compensate for 1 measurement delay between trigger detection and acquisition
        }
        uint32_t tPeriodMicros = getMicrosFromHorizontalDisplayValue(tCountPosition, tCount);
        MeasurementControl.PeriodMicros = tPeriodMicros;
#else
        MeasurementControl.RawValueAverage = (tIntegrateValueForTotalPeriods + (tCountPosition / 2)) / tCountPosition;

        // compute microseconds per period
        float tPeriodMicros = getMicrosFromHorizontalDisplayValue(tCountPosition, tCount) + 0.0005;
#endif
        MeasurementControl.PeriodMicros = tPeriodMicros;
        // frequency
        float tHertz = 1000000.0 / tPeriodMicros;
        MeasurementControl.FrequencyHertz = tHertz + 0.5;
    }
    return;
}

/**
 * compute new trigger value and hysteresis
 * If old values are reasonable don't change them to avoid jitter
 */
void computeAutoTrigger(void) {
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_AUTOMATIC) {
        /*
         * Set auto trigger in middle between min and max
         */
        uint16_t tPeakToPeak = MeasurementControl.RawValueMax - MeasurementControl.RawValueMin;
        // middle between min and max
        uint16_t tPeakToPeakHalf = tPeakToPeak / 2;
        uint16_t tNewRawTriggerValue = MeasurementControl.RawValueMin + tPeakToPeakHalf;

        //set effective hysteresis to quarter delta
        int tTriggerHysteresis = tPeakToPeakHalf / 2;

        // keep reasonable value - avoid jitter - abs does not work, it may give negative values
        // int tTriggerDelta = abs(tNewTriggerValue - MeasurementControl.RawTriggerLevel);
        int tTriggerDelta = tNewRawTriggerValue - MeasurementControl.RawTriggerLevel;
        if (tTriggerDelta < 0) {
            tTriggerDelta = -tTriggerDelta;
        }
        int tOldHysteresis3Quarter = (MeasurementControl.RawHysteresis / 4) * 3;
        if (tTriggerDelta > (tTriggerHysteresis / 4) || tTriggerHysteresis <= tOldHysteresis3Quarter) {
            setTriggerLevelAndHysteresis(tNewRawTriggerValue, tTriggerHysteresis);
        }
    }
}

void setTriggerLevelAndHysteresis(int aRawTriggerValue, int aRawTriggerHysteresis) {
    MeasurementControl.RawTriggerLevel = aRawTriggerValue;
    MeasurementControl.RawHysteresis = aRawTriggerHysteresis;
    if (MeasurementControl.TriggerSlopeRising) {
        MeasurementControl.RawTriggerLevelHysteresis = aRawTriggerValue - aRawTriggerHysteresis;
    } else {
        MeasurementControl.RawTriggerLevelHysteresis = aRawTriggerValue + aRawTriggerHysteresis;
    }
}

/************************************************************************
 * Logic section
 ************************************************************************/

/*
 * Sets isACMode and in turn ChannelIsACMode accordingly - no AC mode for channels without attenuator
 * uses MeasurementControl.DisplayRangeIndex for getRawOffsetValueFromGridCount()
 */
void setACMode(bool aNewACMode) {
#ifdef AVR
    if (MeasurementControl.isRunning) {
        //clear old grid, since it will be changed
        BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    }
#endif
    MeasurementControl.isACMode = aNewACMode;
#ifdef AVR
    // Allow manual setting of AC mode
    MeasurementControl.ChannelIsACMode = aNewACMode;

#else
    if (MeasurementControl.ChannelHasACDCSwitch) {
        MeasurementControl.ChannelIsACMode = aNewACMode;
    } else {
        MeasurementControl.ChannelIsACMode = false;
    }
#endif

    /*
     * Handle AC hardware switching
     */
#ifdef AVR
    uint8_t tRelaisPin;
    if (aNewACMode) {
        // Change AC_DC_BIAS_PIN pin to input
        DDRC = 0; // all analog channels set to input

        // no OffsetAutomatic for AC mode
        MeasurementControl.OffsetMode = OFFSET_MODE_0_VOLT;
        MeasurementControl.OffsetValue = 0;
        tRelaisPin = AC_DC_RELAIS_PIN_1;
    } else {
        // Change AC_DC_BIAS_PIN pin to output and shorten bias to 0V
        DDRC = OUTPUT_MASK_PORTC;
        digitalWriteFast(AC_DC_BIAS_PIN, LOW);

        tRelaisPin = AC_DC_RELAIS_PIN_2;
    }
// change power latching relay state
    digitalWriteFast(tRelaisPin, HIGH);
#else
    DSO_setACMode(aNewACMode);
#endif

    /*
     * New Offset for AC Mode
     */
#ifdef AVR
    // must do it here after settings of flags and before drawing
    setACModeButtonCaption();
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        // hide/show offset
        drawDSOSettingsPage();
    }
    if (MeasurementControl.isRunning) {
        drawGridLinesWithHorizLabelsAndTriggerLine();
    }
    resetOffset();
#else
    setOffsetGridCountAccordingToACMode();
#endif

#ifdef AVR
// Wait for latching relay to switch - 2ms does not work reliable after reset, so take 4ms.
    delay(4);
// No need for relay power any more
    digitalWriteFast(tRelaisPin, LOW);
#endif
}

/************************************************************************
 * Text output section
 ************************************************************************/
/*
 * clear info line
 */
void clearInfo(uint8_t aOldMode) {
// +1 because I have seen artifacts otherwise
    uint8_t tHeight = FONT_SIZE_INFO_SHORT + 1;
    if (aOldMode == INFO_MODE_LONG_INFO) {
        tHeight = (2 * FONT_SIZE_INFO_LONG) + 1;
    }
    BlueDisplay1.fillRectRel(INFO_LEFT_MARGIN, 0, REMOTE_DISPLAY_WIDTH, tHeight, COLOR_BACKGROUND_DSO);
}

/***********************************************************************
 * GUI initialization
 ***********************************************************************/
#ifdef AVR
BDButton TouchButtonADCReference;
const char ReferenceButtonVCC[] PROGMEM = "Ref VCC";
const char ReferenceButton1_1V[] PROGMEM = "Ref 1.1V";
#else
BDButton TouchButtonFFT;
BDButton TouchButtonShowPretriggerValuesOnOff;
BDButton TouchButtonDSOMoreSettings;
BDButton TouchButtonCalibrateVoltage;
BDButton TouchButtonMinMaxMode;

#ifdef FUTURE
BDButton TouchButtonDrawModeTriggerLine;
#endif
#endif
#ifdef LOCAL_DISPLAY_EXISTS
BDButton TouchButtonDrawModeLinePixel;
const char DrawModeButtonStringLine[] = "Line";
const char DrawModeButtonStringPixel[] = "Pixel";

BDButton TouchButtonADS7846TestOnOff;
BDSlider TouchSliderBacklight;
#endif

BDButton TouchButtonSingleshot;
BDButton TouchButtonStartStopDSOMeasurement;

BDButton TouchButtonTriggerMode;
BDButton TouchButtonTriggerDelay;
BDButton TouchButtonChartHistoryOnOff;
BDButton TouchButtonSlope;
char SlopeButtonString[] = "Slope A";

BDButton TouchButtonChannels[NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR];
BDButton TouchButtonChannelSelect;
const char StringChannel0[] PROGMEM = "Ch 0";
const char StringChannel1[] PROGMEM = "Ch 1";
const char StringChannel2[] PROGMEM = "Ch 2";
const char StringChannel3[] PROGMEM = "Ch 3";
const char StringChannel4[] PROGMEM = "Ch 4";
const char StringTemperature[] PROGMEM = "Temp";
const char StringVRefint[] PROGMEM = "VRef";
const char StringVBattDiv2[] PROGMEM = "\xBD" "VBatt";
#ifdef AVR
const char * const ADCInputMUXChannelStrings[ADC_CHANNEL_COUNT] = { StringChannel0, StringChannel1, StringChannel2, StringChannel3,
        StringChannel4, StringTemperature, StringVRefint };
#else
#ifdef STM32F30X
const char * const ADCInputMUXChannelStrings[ADC_CHANNEL_COUNT] = {StringChannel2, StringChannel3, StringChannel4,
    StringTemperature, StringVBattDiv2, StringVRefint};
uint8_t const ADCInputMUXChannels[ADC_CHANNEL_COUNT] = {ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,
    ADC_CHANNEL_TEMPSENSOR, ADC_CHANNEL_VBAT, ADC_CHANNEL_VREFINT};
#else
const char * const ADCInputMUXChannelStrings[ADC_CHANNEL_COUNT] = {StringChannel0, StringChannel1, StringChannel2, StringChannel3,
    StringTemperature, StringVRefint};
const uint8_t ADCInputMUXChannels[ADC_CHANNEL_COUNT] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_TEMPSENSOR, ADC_CHANNEL_VREFINT};
#endif
#endif
const char ChannelDivBy1ButtonString[] PROGMEM = "\xF7" "1";
const char ChannelDivBy10ButtonString[] PROGMEM = "\xF7" "10";
const char ChannelDivBy100ButtonString[] PROGMEM = "\xF7" "100";
const char * const ChannelDivByButtonStrings[NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR] = { ChannelDivBy1ButtonString,
        ChannelDivBy10ButtonString, ChannelDivBy100ButtonString };
BDButton TouchButtonChannelMode;

BDButton TouchButtonAutoOffsetMode;
const char AutoOffsetButtonStringMan[] PROGMEM = "Offset man";
const char AutoOffsetButtonStringAuto[] PROGMEM = "Offset auto";
const char AutoOffsetButtonString0[] PROGMEM = "Offset 0V";

BDButton TouchButtonAutoRangeOnOff;
const char AutoRangeButtonStringAuto[] PROGMEM = "Range auto";
const char AutoRangeButtonStringManual[] PROGMEM = "Range man";

BDButton TouchButtonSettingsPage;
BDButton TouchButtonFrequencyPage;
BDButton TouchButtonAcDc;

/*
 * Slider for trigger level and voltage picker
 */
BDSlider TouchSliderTriggerLevel;
BDSlider TouchSliderVoltagePicker;

void initDSOGUI(void) {
    BlueDisplay1.setButtonsGlobalFlags(USE_UP_EVENTS_FOR_BUTTONS); // since swipe recognition needs it

    int tPosY = 0;
    /***************************
     * Start page
     ***************************/
// 1. row
    // Button for Singleshot
    TouchButtonSingleshot.initPGM(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, START_PAGE_BUTTON_HEIGHT,
    COLOR_GUI_CONTROL, PSTR("Singleshot"), TEXT_SIZE_14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStartSingleshot);

// 2. row
    tPosY += START_PAGE_ROW_INCREMENT;

#ifdef LOCAL_FILESYSTEM_EXISTS
    TouchButtonStore.init(0, tPosY, BUTTON_WIDTH_5, START_PAGE_BUTTON_HEIGHT, COLOR_GUI_SOURCE_TIMEBASE,
            "Store", TEXT_SIZE_11, BUTTON_FLAG_NO_BEEP_ON_TOUCH, MODE_STORE, &doStoreLoadAcquisitionData);

    TouchButtonLoad.init(BUTTON_WIDTH_5_POS_2, tPosY, BUTTON_WIDTH_5, START_PAGE_BUTTON_HEIGHT,
            COLOR_GUI_SOURCE_TIMEBASE, "Load", TEXT_SIZE_11, BUTTON_FLAG_NO_BEEP_ON_TOUCH, MODE_LOAD, &doStoreLoadAcquisitionData);
#endif

    // big start stop button
    TouchButtonStartStopDSOMeasurement.initPGM(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3,
            (2 * START_PAGE_BUTTON_HEIGHT) + BUTTON_DEFAULT_SPACING, COLOR_GUI_CONTROL, PSTR("Start\nStop"), TEXT_SIZE_26,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStartStopDSO);

// 4. row
    tPosY += 2 * START_PAGE_ROW_INCREMENT;
#ifndef AVR
    // Button for show FFT - only for Start and Chart pages
    TouchButtonFFT.init(0, tPosY, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_GREEN, "FFT",
            TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN_MANUAL_REFRESH, DisplayControl.ShowFFT,
            &doShowFFT);
#endif

    // Button for settings page
    TouchButtonSettingsPage.initPGM(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, START_PAGE_BUTTON_HEIGHT, COLOR_GUI_CONTROL,
            PSTR("Settings"), TEXT_SIZE_18, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doShowSettingsPage);

    /***************************
     * Settings page
     ***************************/
// 1. row
    tPosY = 0;

    /*
     * Button for chart history (erase color). Must be at first row since it is (hidden) active at running mode
     */
#ifdef LOCAL_DISPLAY_EXISTS
    TouchButtonChartHistoryOnOff.init(SLIDER_DEFAULT_BAR_WIDTH + 6, tPosY, BUTTON_WIDTH_3 - (SLIDER_DEFAULT_BAR_WIDTH + 6),
            SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_DISPLAY_CONTROL, "History", TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN_MANUAL_REFRESH, 0, &doChartHistory);
#else
    TouchButtonChartHistoryOnOff.initPGM(0, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_RED, PSTR("History"),
    TEXT_SIZE_18, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN_MANUAL_REFRESH, 0, &doChartHistory);
#endif

    // Button for slope
    TouchButtonSlope.init(BUTTON_WIDTH_3_POS_2, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "",
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTriggerSlope);
    setSlopeButtonCaption();

    // Back button for sub pages
    TouchButtonBack.initPGM(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_CONTROL,
            PSTR("Back"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doDefaultBackButton);

// 2. row
    tPosY += SETTINGS_PAGE_ROW_INCREMENT;

#ifdef AVR
    // Button for delay
    TouchButtonTriggerDelay.init(0, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "", TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doPromptForTriggerDelay);
    setTriggerDelayCaption();
#endif

    // Button for trigger mode
    TouchButtonTriggerMode.init(BUTTON_WIDTH_3_POS_2, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "",
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTriggerMode);
    setTriggerModeButtonCaption();

    // Button for channel 0
    TouchButtonChannels[0].init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_6, SETTINGS_PAGE_BUTTON_HEIGHT,
    BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doChannelSelect);

    // Button for channel 1
    TouchButtonChannels[1].init(REMOTE_DISPLAY_WIDTH - BUTTON_WIDTH_6, tPosY, BUTTON_WIDTH_6, SETTINGS_PAGE_BUTTON_HEIGHT,
    BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doChannelSelect);

// 3. row
    tPosY += SETTINGS_PAGE_ROW_INCREMENT;

#ifndef AVR
// Button for pretrigger area show
#ifdef LOCAL_DISPLAY_EXISTS
    TouchButtonShowPretriggerValuesOnOff.init(SLIDER_DEFAULT_BAR_WIDTH + 6, tPosY,
            BUTTON_WIDTH_3 - (SLIDER_DEFAULT_BAR_WIDTH + 6), SETTINGS_PAGE_BUTTON_HEIGHT,
            COLOR_BLACK, "Show\nPretrigger", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            (DisplayControl.DatabufferPreTriggerDisplaySize != 0), &doShowPretriggerValuesOnOff);
#else
    TouchButtonShowPretriggerValuesOnOff.init(0, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT,
            COLOR_BLACK, "Show\nPretrigger", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            (DisplayControl.DatabufferPreTriggerDisplaySize != 0), &doShowPretriggerValuesOnOff);
#endif
#endif

    // Button for AutoRange on off
    TouchButtonAutoRangeOnOff.init(BUTTON_WIDTH_3_POS_2, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "",
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doRangeMode);
    setAutoRangeModeAndButtonCaption(true);

    // Button for channel 2
    TouchButtonChannels[2].init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_6, SETTINGS_PAGE_BUTTON_HEIGHT,
    BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 2, &doChannelSelect);
    setChannelButtonsCaption();

    // Button for channel select
    TouchButtonChannelSelect.initPGM(REMOTE_DISPLAY_WIDTH - BUTTON_WIDTH_6, tPosY, BUTTON_WIDTH_6, SETTINGS_PAGE_BUTTON_HEIGHT,
    BUTTON_AUTO_RED_GREEN_FALSE_COLOR, StringChannel3, TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 42, &doChannelSelect);

// 4. row
    tPosY += SETTINGS_PAGE_ROW_INCREMENT;

#ifndef AVR
    // Button for min/max acquisition mode
#ifdef LOCAL_DISPLAY_EXISTS
    TouchButtonMinMaxMode.init(SLIDER_DEFAULT_BAR_WIDTH + 6, tPosY, BUTTON_WIDTH_3 - (SLIDER_DEFAULT_BAR_WIDTH + 6),
            SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_DISPLAY_CONTROL, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH| FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            MeasurementControl.isMinMaxMode, &doMinMaxMode);
    setMinMaxModeButtonCaption();
#else
    TouchButtonMinMaxMode.init(0, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_BLACK, "Sample\nmode", TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, MeasurementControl.isMinMaxMode, &doMinMaxMode);
    TouchButtonMinMaxMode.setCaptionForValueTrue("Min/Max\nmode");
#endif

#endif

    // Button for auto offset on, 0-Volt, manual
    TouchButtonAutoOffsetMode.init(BUTTON_WIDTH_3_POS_2, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "",
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doOffsetMode);
    setAutoOffsetButtonCaption();

#ifdef FUTURE
    // Button for trigger line mode
    TouchButtonDrawModeTriggerLine.init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT,
            COLOR_GUI_DISPLAY_CONTROL, "Trigger\nline", TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, DisplayControl.showTriggerInfoLine,
            &doDrawModeTriggerLine);
#endif

// 5. row
    tPosY = REMOTE_DISPLAY_HEIGHT - SETTINGS_PAGE_BUTTON_HEIGHT;

    // Button for selecting Frequency page.
    TouchButtonFrequencyPage.initPGM(0, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_RED, PSTR("Frequency\nGenerator"),
    TEXT_SIZE_14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doShowFrequencyPage);

    // Button for AC / DC
    TouchButtonAcDc.init(BUTTON_WIDTH_3_POS_2, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GUI_TRIGGER, "",
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doAcDcMode);
    setACModeButtonCaption();

#ifdef AVR
    // Button for reference voltage switching
    TouchButtonADCReference.init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT,
    COLOR_GUI_SOURCE_TIMEBASE, "", TEXT_SIZE_18, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doADCReference);
    setReferenceButtonCaption();
#else
    // Button for more-settings pages
    TouchButtonDSOMoreSettings.init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, BUTTON_HEIGHT_5,
            COLOR_GUI_CONTROL, "More", TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doShowMoreSettingsPage);

    /***************************
     * More Settings page
     ***************************/
// 1. row
    tPosY = 0;
    // Button for voltage calibration
    TouchButtonCalibrateVoltage.init(0, tPosY, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
            COLOR_GUI_SOURCE_TIMEBASE, "Calibrate U", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doVoltageCalibration);
// 2. row
    tPosY += SETTINGS_PAGE_ROW_INCREMENT;
    // Button for system info
    TouchButtonShowSystemInfo.init(BUTTON_WIDTH_3_POS_3, tPosY, BUTTON_WIDTH_3, SETTINGS_PAGE_BUTTON_HEIGHT, COLOR_GREEN,
            "System\ninfo", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doShowSystemInfoPage);
#endif

    /*
     * SLIDER
     */
// make slider slightly visible
// slider for voltage picker
    TouchSliderVoltagePicker.init(SLIDER_VPICKER_POS_X, 0, SLIDER_SIZE, REMOTE_DISPLAY_HEIGHT, REMOTE_DISPLAY_HEIGHT, 0, 0,
    COLOR_VOLTAGE_PICKER_SLIDER, FLAG_SLIDER_VALUE_BY_CALLBACK, &doVoltagePicker);
    TouchSliderVoltagePicker.setBarBackgroundColor( COLOR_VOLTAGE_PICKER_SLIDER);

// slider for trigger level
    TouchSliderTriggerLevel.init(SLIDER_TLEVEL_POS_X, 0, SLIDER_SIZE, REMOTE_DISPLAY_HEIGHT, REMOTE_DISPLAY_HEIGHT, 0, 0,
    COLOR_TRIGGER_SLIDER, FLAG_SLIDER_VALUE_BY_CALLBACK, &doTriggerLevel);
    TouchSliderTriggerLevel.setBarBackgroundColor( COLOR_TRIGGER_SLIDER);

#ifdef LOCAL_DISPLAY_EXISTS
// 2. row
// Button for switching draw mode - line/pixel
    TouchButtonDrawModeLinePixel.init(0, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3,
            BUTTON_HEIGHT_4, COLOR_GUI_DISPLAY_CONTROL, DrawModeButtonStringLine, TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0,
            &doDrawMode);

// Button for ADS7846 channel
    TouchButtonADS7846TestOnOff.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, 0, "ADS7846",
            TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, MeasurementControl.ADS7846ChannelsAsDatasource,
            &doADS7846TestOnOff);

    /*
     * Backlight slider
     */
    TouchSliderBacklight.init(0, 0, SLIDER_DEFAULT_BAR_WIDTH, BACKLIGHT_MAX_VALUE, BACKLIGHT_MAX_VALUE, getBacklightValue(),
            COLOR_BLUE, COLOR_GREEN, FLAG_SLIDER_VERTICAL_SHOW_NOTHING, &doBacklightSlider);
#endif

}

/*
 * activate elements if returning from settings screen or if starting acquisition
 */
void activateChartGui(void) {
    TouchButtonChartHistoryOnOff.activate();
    TouchButtonSingleshot.activate();
    TouchButtonStartStopDSOMeasurement.activate();
    TouchButtonSettingsPage.activate();
#ifndef AVR
    TouchButtonFFT.activate();
#endif

    TouchSliderVoltagePicker.drawSlider();
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL_TIMEOUT || MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
        TouchSliderTriggerLevel.drawSlider();
    }

#ifdef LOCAL_DISPLAY_EXISTS
    TouchButtonMainHome.activate();
#endif
}

/************************************************************************
 * Output and draw section
 ************************************************************************/

void redrawDisplay() {
    clearDisplayAndDisableButtonsAndSliders(COLOR_BACKGROUND_DSO);

    if (MeasurementControl.isRunning) {
        /*
         * running mode
         */
        if (DisplayControl.DisplayPage >= DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPage();
        } else {
            activateChartGui();
            // refresh grid - not really needed, since after MILLIS_BETWEEN_INFO_OUTPUT it is done by loop
            drawGridLinesWithHorizLabelsAndTriggerLine();
            printInfo();
#ifndef AVR
            TouchButtonChartHistoryOnOff.activate(); //???
            // initialize FFTDisplayBuffer
            memset(&DisplayBufferFFT[0], REMOTE_DISPLAY_HEIGHT - 1, sizeof(DisplayBufferFFT));
#endif
        }
    } else {
        /*
         * analyze mode
         */
        if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
            drawStartPage();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
            activateChartGui();
            drawGridLinesWithHorizLabelsAndTriggerLine();
            drawMinMaxLines();
            // draw from last scroll position
#ifdef AVR
            drawDataBuffer(DataBufferControl.DataBufferDisplayStart, COLOR_DATA_HOLD, DisplayControl.EraseColor);
#else
            drawDataBuffer(DataBufferControl.DataBufferDisplayStart, REMOTE_DISPLAY_WIDTH, COLOR_DATA_HOLD, 0,
                    DRAW_MODE_REGULAR, MeasurementControl.isEffectiveMinMaxMode);
#endif
            printInfo();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPage();
        }
    }
}

void drawStartPage(void) {
//1. Row
    TouchButtonChartHistoryOnOff.drawButton();
    TouchButtonSingleshot.drawButton();
//2. Row
#ifdef LOCAL_FILESYSTEM_EXISTS
    TouchButtonStore.drawButton();
    TouchButtonLoad.drawButton();
#endif
    TouchButtonStartStopDSOMeasurement.drawButton();
// 4. Row
#ifndef AVR
    TouchButtonFFT.drawButton();
#endif
    TouchButtonSettingsPage.drawButton();
//Welcome text
    BlueDisplay1.drawTextPGM(10, BUTTON_HEIGHT_4_LINE_2 + 32, PSTR("Welcome to\nArduino DSO"), 32, COLOR_BLUE,
    COLOR_BACKGROUND_DSO);
    BlueDisplay1.drawTextPGM(10, BUTTON_HEIGHT_4_LINE_2 + (3 * 32), PSTR("300 kSamples/s"), 22, COLOR_BLUE,
    COLOR_BACKGROUND_DSO);
    uint8_t tPos = BlueDisplay1.drawTextPGM(10, BUTTON_HEIGHT_4_LINE_2 + (3 * 32) + 22,
            PSTR("V" VERSION_DSO "/" VERSION_BLUE_DISPLAY), 11, COLOR_BLUE, COLOR_BACKGROUND_DSO);
    BlueDisplay1.drawTextPGM(tPos, BUTTON_HEIGHT_4_LINE_2 + (3 * 32) + 22, PSTR(" from " __DATE__), 11, COLOR_BLUE,
    COLOR_BACKGROUND_DSO);

// Hints
#ifndef AVR
    BlueDisplay1.drawText(BUTTON_WIDTH_3_POS_2, TEXT_SIZE_22_ASCEND, "\xABScale\xBB",
            TEXT_SIZE_22, COLOR_YELLOW, COLOR_BACKGROUND_DSO);
#endif
    BlueDisplay1.drawTextPGM(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 + BUTTON_DEFAULT_SPACING + TEXT_SIZE_22_ASCEND,
            PSTR("\xABScroll\xBB"), TEXT_SIZE_22, COLOR_GREEN, COLOR_BACKGROUND_DSO);

#ifdef LOCAL_DISPLAY_EXISTS
    TouchButtonMainHome.drawButton();
#endif
}

/**
 * draws elements active for settings page
 */
void drawDSOSettingsPage(void) {
//1. Row
    TouchButtonChartHistoryOnOff.drawButton();
    TouchButtonSlope.drawButton();
    TouchButtonBack.drawButton();

//2. Row
#ifdef AVR
    TouchButtonTriggerDelay.drawButton();
#endif
    TouchButtonTriggerMode.drawButton();

    int16_t tButtonColor;
    /*
     * Determine colors for 3 fixed channel buttons
     */
    for (int i = 0; i < NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR; ++i) {
        if (i == MeasurementControl.ADCInputMUXChannelIndex) {
            tButtonColor = BUTTON_AUTO_RED_GREEN_TRUE_COLOR;
        } else {
            tButtonColor = BUTTON_AUTO_RED_GREEN_FALSE_COLOR;
        }
        TouchButtonChannels[i].setButtonColorAndDraw(tButtonColor);
    }
    if (MeasurementControl.ADCInputMUXChannelIndex >= NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR) {
        tButtonColor = BUTTON_AUTO_RED_GREEN_TRUE_COLOR;
    } else {
        tButtonColor = BUTTON_AUTO_RED_GREEN_FALSE_COLOR;
    }
    TouchButtonChannelSelect.setButtonColorAndDraw(tButtonColor);

//3. Row
#ifndef AVR
    TouchButtonShowPretriggerValuesOnOff.drawButton();
#endif
    TouchButtonAutoRangeOnOff.drawButton();
    if (MeasurementControl.OffsetMode == OFFSET_MODE_AUTOMATIC) {
// lock button in OFFSET_MODE_AUTOMATIC
        TouchButtonAutoRangeOnOff.deactivate();
    }

    TouchButtonChannelSelect.drawButton();

// 4. Row
#ifndef AVR
    TouchButtonMinMaxMode.drawButton();
#endif
    TouchButtonAutoOffsetMode.drawButton();
#ifdef FUTURE
    TouchButtonDrawModeTriggerLine.drawButton();
#endif

#ifdef LOCAL_DISPLAY_EXISTS
    TouchSliderBacklight.drawSlider();
#endif

//5. Row
    TouchButtonFrequencyPage.drawButton();
//    if (MeasurementControl.ChannelHasACDCSwitch) {
    TouchButtonAcDc.drawButton();
//    }

#ifdef AVR
    TouchButtonADCReference.drawButton();

    // print minimum stacksize and enable new stack measurement
    printFreeStack();
    initStackFreeMeasurement();
#else
    TouchButtonDSOMoreSettings.drawButton();
#endif
}

#ifndef AVR
void drawDSOMoreSettingsPage(void) {
// do not clear screen here since gui is refreshed periodically while DSO is running
    BDButton::deactivateAllButtons();
    BDSlider::deactivateAllSliders();
//1. Row
    TouchButtonCalibrateVoltage.drawButton();
    TouchButtonBack.drawButton();

#ifdef LOCAL_DISPLAY_EXISTS
//2. Row
    TouchButtonDrawModeLinePixel.drawButton();
    TouchButtonADS7846TestOnOff.drawButton();
#endif
// 4. Row
    TouchButtonShowSystemInfo.drawButton();
}

void startDSOMoreSettingsPage(void) {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    drawDSOMoreSettingsPage();
}
#endif

/**
 * draws elements which are active while measurement is running
 */
void drawRunningOnlyPartOfGui(void) {
//    if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
//        TouchSliderTriggerLevel.drawSlider();
//    }

    if (!MeasurementControl.RangeAutomatic || MeasurementControl.OffsetMode == OFFSET_MODE_MANUAL) {
#ifdef LOCAL_DISPLAY_EXISTS
        BlueDisplay1.drawMLText(TEXT_SIZE_11_WIDTH, TEXT_SIZE_11_HEIGHT + TEXT_SIZE_22_ASCEND, "\xD4\nR\na\nn\ng\ne\n\xD5",
                TEXT_SIZE_22, COLOR_GUI_TRIGGER, COLOR_NO_BACKGROUND);
#else
        //START_PAGE_BUTTON_HEIGHT + TEXT_SIZE_22_ASCEND is to much for 240 display
        BlueDisplay1.drawTextPGM(TEXT_SIZE_11_WIDTH, START_PAGE_BUTTON_HEIGHT, PSTR("\xD4\nR\na\nn\ng\ne\n\xD5"), TEXT_SIZE_22,
        COLOR_GUI_TRIGGER, COLOR_NO_BACKGROUND);
#endif
    }

    if (MeasurementControl.OffsetMode != OFFSET_MODE_0_VOLT) {
#ifdef LOCAL_DISPLAY_EXISTS
        BlueDisplay1.drawMLText(BUTTON_WIDTH_3_POS_3 - TEXT_SIZE_22_WIDTH, TEXT_SIZE_11_HEIGHT + TEXT_SIZE_22_ASCEND,
                "\xD4\nO\nf\nf\ns\ne\nt\n\xD5", TEXT_SIZE_22, COLOR_GUI_TRIGGER, COLOR_NO_BACKGROUND);
#else
        BlueDisplay1.drawTextPGM(BUTTON_WIDTH_3_POS_3 - TEXT_SIZE_22_WIDTH, TEXT_SIZE_11_HEIGHT + TEXT_SIZE_22_ASCEND,
                PSTR("\xD4\nO\nf\nf\ns\ne\nt\n\xD5"), TEXT_SIZE_22, COLOR_GUI_TRIGGER, COLOR_NO_BACKGROUND);
#endif
    }

    BlueDisplay1.drawTextPGM(BUTTON_WIDTH_8, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_22_DECEND, PSTR("\xABTimeBase\xBB"),
    TEXT_SIZE_22, COLOR_GUI_SOURCE_TIMEBASE, COLOR_BACKGROUND_DSO);

//1. Row
    TouchButtonChartHistoryOnOff.drawButton();
    TouchButtonSingleshot.drawButton();
//2. Row
    TouchButtonStartStopDSOMeasurement.drawButton();

// 5. row
#ifndef AVR
    TouchButtonFFT.drawButton();
#endif
    TouchButtonSettingsPage.drawButton();
}

void clearTriggerLine(uint8_t aTriggerLevelDisplayValue) {
// clear old line
    BlueDisplay1.drawLineRel(0, aTriggerLevelDisplayValue, REMOTE_DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);

    if (DisplayControl.showInfoMode != INFO_MODE_NO_INFO) {
        // restore grid at old y position
        for (uint16_t tXPos = TIMING_GRID_WIDTH - 1; tXPos < REMOTE_DISPLAY_WIDTH - 1; tXPos += TIMING_GRID_WIDTH) {
            BlueDisplay1.drawPixel(tXPos, aTriggerLevelDisplayValue, COLOR_GRID_LINES);
        }
    }
#ifndef AVR
    if (!MeasurementControl.isRunning) {
        // in analysis mode restore graph at old y position
        uint8_t* ScreenBufferPointer = &DisplayBuffer[0];
        for (unsigned int i = 0; i < REMOTE_DISPLAY_WIDTH; ++i) {
            int tValueByte = *ScreenBufferPointer++;
            if (tValueByte == aTriggerLevelDisplayValue) {
                // restore old pixel
                BlueDisplay1.drawPixel(i, tValueByte, COLOR_DATA_HOLD);
            }
        }
    }
#endif
}

/**
 * draws trigger line if it is visible - do not draw clipped value e.g. value was higher than display range
 */
void drawTriggerLine(void) {
    uint8_t tValue = DisplayControl.TriggerLevelDisplayValue;
    if (tValue != 0 && MeasurementControl.TriggerMode < TRIGGER_MODE_FREE) {
        BlueDisplay1.drawLineRel(0, tValue, REMOTE_DISPLAY_WIDTH, 0, COLOR_TRIGGER_LINE);
    }
}

/*
 * draws min, max lines
 */
void drawMinMaxLines(void) {
// draw max line
#ifdef AVR
    uint8_t tValueDisplay;
#else
    int tValueDisplay;
#endif
    tValueDisplay = getDisplayFromRawInputValue(MeasurementControl.RawValueMax);
    if (tValueDisplay != 0) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, REMOTE_DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
    }
// min line
    tValueDisplay = getDisplayFromRawInputValue(MeasurementControl.RawValueMin);
    if (tValueDisplay != DISPLAY_VALUE_FOR_ZERO) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, REMOTE_DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
    }
}

/*
 * draws vertical timing + horizontal reference voltage lines
 */
void drawGridLinesWithHorizLabelsAndTriggerLine() {
    if (DisplayControl.DisplayPage != DISPLAY_PAGE_CHART) {
        return;
    }
// vertical (timing) lines
    for (unsigned int tXPos = TIMING_GRID_WIDTH - 1; tXPos < REMOTE_DISPLAY_WIDTH; tXPos += TIMING_GRID_WIDTH) {
        BlueDisplay1.drawLineRel(tXPos, 0, 0, REMOTE_DISPLAY_HEIGHT, COLOR_GRID_LINES);
    }
#ifdef AVR
    /*
     * drawHorizontalLineLabels
     * adjust the distance between the lines to the actual range which is also determined by the reference voltage
     */
    float tActualVoltage = 0;
    char tStringBuffer[6];
    uint8_t tPrecision = 2 - MeasurementControl.AttenuatorValue;
    uint8_t tLength = 2 + tPrecision;
    if (MeasurementControl.ChannelIsACMode) {
        /*
         * draw from middle of screen to top and "mirror" lines for negative values
         */
        // Start at DISPLAY_HEIGHT/2 shift 8
        for (int32_t tYPosLoop = 0x8000; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // horizontal line
            BlueDisplay1.drawLineRel(0, tYPos, REMOTE_DISPLAY_WIDTH, 0, COLOR_GRID_LINES);
            dtostrf(tActualVoltage, tLength, tPrecision, tStringBuffer);
            // draw label over the line
            BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X, tYPos + (TEXT_SIZE_11_ASCEND / 2), tStringBuffer, 11,
            COLOR_HOR_REF_LINE_LABEL, COLOR_NO_BACKGROUND);
            if (tYPos != REMOTE_DISPLAY_HEIGHT / 2) {
                // line with negative value
                BlueDisplay1.drawLineRel(0, REMOTE_DISPLAY_HEIGHT - tYPos, REMOTE_DISPLAY_WIDTH, 0, COLOR_GRID_LINES);
                dtostrf(-tActualVoltage, tLength, tPrecision, tStringBuffer);
                // draw label over the line
                BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X - TEXT_SIZE_11_WIDTH,
                        REMOTE_DISPLAY_HEIGHT - tYPos + (TEXT_SIZE_11_ASCEND / 2), tStringBuffer, 11, COLOR_HOR_REF_LINE_LABEL,
                        COLOR_NO_BACKGROUND);
            }
            tActualVoltage += MeasurementControl.HorizontalGridVoltage;
        }
    } else {
        if (MeasurementControl.OffsetMode == OFFSET_MODE_AUTOMATIC) {
            tActualVoltage = MeasurementControl.HorizontalGridVoltage * MeasurementControl.OffsetGridCount;
        }
        // draw first caption over the line
        int8_t tCaptionOffset = 1;
        // Start at (DISPLAY_VALUE_FOR_ZERO) shift 8) + 1/2 shift8 for better rounding
        for (int32_t tYPosLoop = 0xFF80; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // horizontal line
            BlueDisplay1.drawLineRel(0, tYPos, REMOTE_DISPLAY_WIDTH, 0, COLOR_GRID_LINES);
            dtostrf(tActualVoltage, tLength, tPrecision, tStringBuffer);
            // draw label over the line
            BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X, tYPos - tCaptionOffset, tStringBuffer, 11,
            COLOR_HOR_REF_LINE_LABEL, COLOR_NO_BACKGROUND);
            // draw next caption on the line
            tCaptionOffset = -(TEXT_SIZE_11_ASCEND / 2);
            tActualVoltage += MeasurementControl.HorizontalGridVoltage;
        }
    }

#else
    /*
     * Here we have a fixed layout
     */
// add 0.0001 to avoid display of -0.00
    float tActualVoltage = (ScaleVoltagePerDiv[MeasurementControl.DisplayRangeIndexForPrint] * (MeasurementControl.OffsetGridCount)
            + 0.0001);
    /*
     * check if range or offset changed in order to change label
     */
    bool tLabelChanged = false;
    if (DisplayControl.LastDisplayRangeIndex != MeasurementControl.DisplayRangeIndexForPrint
            || DisplayControl.LastOffsetGridCount != MeasurementControl.OffsetGridCount) {
        DisplayControl.LastDisplayRangeIndex = MeasurementControl.DisplayRangeIndexForPrint;
        DisplayControl.LastOffsetGridCount = MeasurementControl.OffsetGridCount;
        tLabelChanged = true;
    }
    int tCaptionOffset = 1;
    color16_t tLabelColor;
    int tPosX;
    for (int tYPos = DISPLAY_VALUE_FOR_ZERO; tYPos > 0; tYPos -= HORIZONTAL_GRID_HEIGHT) {
        if (tLabelChanged) {
            // clear old label
            int tXpos = REMOTE_DISPLAY_WIDTH - PIXEL_AFTER_LABEL - (5 * TEXT_SIZE_11_WIDTH);
            int tY = tYPos - tCaptionOffset;
            BlueDisplay1.fillRect(tXpos, tY - TEXT_SIZE_11_ASCEND, REMOTE_DISPLAY_WIDTH - PIXEL_AFTER_LABEL + 1,
                    tY + TEXT_SIZE_11_HEIGHT - TEXT_SIZE_11_ASCEND, COLOR_BACKGROUND_DSO);
            // restore last vertical line since label may overlap these line
            BlueDisplay1.drawLineRel(9 * TIMING_GRID_WIDTH - 1, tY, 0, TEXT_SIZE_11_HEIGHT, COLOR_GRID_LINES);
        }
        // draw horizontal line
        BlueDisplay1.drawLineRel(0, tYPos, REMOTE_DISPLAY_WIDTH, 0, COLOR_GRID_LINES);

        int tCount = snprintf(sStringBuffer, sizeof sStringBuffer, "%0.*f",
                RangePrecision[MeasurementControl.DisplayRangeIndexForPrint], tActualVoltage);
        // right align but leave 2 pixel free after label for the last horizontal line
        tPosX = REMOTE_DISPLAY_WIDTH - (tCount * TEXT_SIZE_11_WIDTH) - PIXEL_AFTER_LABEL;
        // draw label over the line - use different color for negative values
        if (tActualVoltage >= 0) {
            tLabelColor = COLOR_HOR_GRID_LINE_LABEL;
        } else {
            tLabelColor = COLOR_HOR_GRID_LINE_LABEL_NEGATIVE;
        }
        BlueDisplay1.drawText(tPosX, tYPos - tCaptionOffset, sStringBuffer, TEXT_SIZE_11, tLabelColor, COLOR_BACKGROUND_DSO);
        tCaptionOffset = -(TEXT_SIZE_11_ASCEND / 2);
        tActualVoltage += ScaleVoltagePerDiv[MeasurementControl.DisplayRangeIndexForPrint];
    }
#endif
    drawTriggerLine();
}

/************************************************************************
 * Button caption section
 ************************************************************************/
void setChannelButtonsCaption(void) {
    for (uint8_t i = 0; i < NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR; ++i) {
        if (MeasurementControl.AttenuatorType == ATTENUATOR_TYPE_FIXED_ATTENUATOR) {
            TouchButtonChannels[i].setCaptionPGM(ChannelDivByButtonStrings[i]);
        } else {
            TouchButtonChannels[i].setCaptionPGM(ADCInputMUXChannelStrings[i]);
        }
    }
}

void setSlopeButtonCaption(void) {
    uint8_t tChar;
    if (MeasurementControl.TriggerSlopeRising) {
        tChar = 'A'; // 0xD1; //ascending
    } else {
        tChar = 'D'; // 0xD2; //descending
    }
    SlopeButtonString[SLOPE_STRING_INDEX] = tChar;
    TouchButtonSlope.setCaption(SlopeButtonString, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setTriggerModeButtonCaption(void) {
    const char * tCaption;
// switch statement code is 12 bytes shorter here
    switch (MeasurementControl.TriggerMode) {
    case TRIGGER_MODE_AUTOMATIC:
        tCaption = PSTR("Trigger auto");
        break;
    case TRIGGER_MODE_MANUAL_TIMEOUT:
        tCaption = PSTR("Trigger man\ntimeout");
        break;
    case TRIGGER_MODE_MANUAL:
        tCaption = PSTR("Trigger man");
        break;
    case TRIGGER_MODE_FREE:
        tCaption = PSTR("Trigger free");
        break;
    default:
        tCaption = PSTR("Trigger ext");
        break;
    }
    TouchButtonTriggerMode.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
    /* This saves >12 byte program space but needs 10 byte RAM
     TouchButtonTriggerMode.setCaptionPGM(TriggerModeButtonStrings[MeasurementControl.TriggerMode],(DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
     */
}

void setAutoRangeModeAndButtonCaption(bool aNewAutoRangeMode) {
    MeasurementControl.RangeAutomatic = aNewAutoRangeMode;
    const char * tCaption;
    if (MeasurementControl.RangeAutomatic) {
        tCaption = AutoRangeButtonStringAuto;
    } else {
        tCaption = AutoRangeButtonStringManual;
    }
    TouchButtonAutoRangeOnOff.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setAutoOffsetButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.OffsetMode == OFFSET_MODE_0_VOLT) {
        tCaption = AutoOffsetButtonString0;
    } else if (MeasurementControl.OffsetMode == OFFSET_MODE_AUTOMATIC) {
        tCaption = AutoOffsetButtonStringAuto;
    } else {
        tCaption = AutoOffsetButtonStringMan;
    }
    TouchButtonAutoOffsetMode.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setACModeButtonCaption(void) {
    if (MeasurementControl.ChannelIsACMode) {
        TouchButtonAcDc.setCaptionPGM(PSTR("AC"));
    } else {
        TouchButtonAcDc.setCaptionPGM(PSTR("DC"));
    }
}

#ifdef AVR
void setTriggerDelayCaption(void) {
    strcpy_P(&sStringBuffer[0], PSTR("Trigger delay\n"));
    if (MeasurementControl.TriggerDelayMode != TRIGGER_DELAY_NONE) {
        printfTriggerDelay(&sStringBuffer[14], MeasurementControl.TriggerDelayMillisOrMicros);
    }
    TouchButtonTriggerDelay.setCaption(sStringBuffer, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setReferenceButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.ADCReference == DEFAULT) {
        tCaption = ReferenceButtonVCC;
    } else {
        tCaption = ReferenceButton1_1V;
    }
    TouchButtonADCReference.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}
#endif

#ifdef LOCAL_DISPLAY_EXISTS
void setMinMaxModeButtonCaption(void) {
    if (MeasurementControl.isMinMaxMode) {
        TouchButtonMinMaxMode.setCaption("Min/Max\nmode");
    } else {
        TouchButtonMinMaxMode.setCaption("Sample\nmode");
    }
}
#endif

void startDSOSettingsPage(void) {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    drawDSOSettingsPage();
}

/************************************************************************
 * Event handler section
 ************************************************************************/

/*
 * Handler for "empty" touch
 * Use touch up in order not to interfere with long touch
 * Switch between upper info line short/long/off
 */
void doTouchUp(struct TouchEvent * const aTouchPosition) {
#ifdef LOCAL_DISPLAY_EXISTS
// first check for buttons
    if (!TouchButton::checkAllButtons(aTouchPosition->TouchPosition.PosX, aTouchPosition->TouchPosition.PosY)) {
#endif
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        // Wrap display mode
        uint8_t tOldMode = DisplayControl.showInfoMode;
        uint8_t tNewMode = tOldMode + 1;
        if (tNewMode > INFO_MODE_LONG_INFO) {
            tNewMode = INFO_MODE_NO_INFO;
        }
        DisplayControl.showInfoMode = tNewMode;

        if (tNewMode == INFO_MODE_NO_INFO) {
            if (MeasurementControl.isRunning) {
                // erase former info line
                clearInfo(tOldMode);
            } else {
                redrawDisplay();
            }
        } else {
            // erase former info line
            clearInfo(tOldMode);
            printInfo();
        }
    }
#ifdef LOCAL_DISPLAY_EXISTS
}
#endif
}

/*
 * If stopped toggle between Start and Chart page
 * If running toggle between gui display and chart only
 */
void doLongTouchDownDSO(struct TouchEvent * const aTochPosition) {
    static bool sIsGUIVisible = false;
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        if (MeasurementControl.isRunning) {
            if (sIsGUIVisible) {
                // hide GUI
                redrawDisplay();
            } else {
                // Show usable GUI
                drawRunningOnlyPartOfGui();
            }
            sIsGUIVisible = !sIsGUIVisible;
        } else {
            // clear screen and show start gui
            DisplayControl.DisplayPage = DISPLAY_PAGE_START;
            redrawDisplay();
        }
    } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
        // only chart (and voltage picker)
        DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
        redrawDisplay();
    }
}

/**
 * responsible for swipe detection and dispatching
 */
void doSwipeEndDSO(struct Swipe * const aSwipeInfo) {
#ifdef AVR
    uint8_t tFeedbackType = FEEDBACK_TONE_ERROR;
#else
    int tFeedbackType = FEEDBACK_TONE_ERROR;
#endif

    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        if (MeasurementControl.isRunning) {
            /*
             * Running mode
             */
            if (aSwipeInfo->SwipeMainDirectionIsX) {
                /*
                 * Horizontal swipe - Timebase -> use TouchDeltaX/64
                 */
#ifdef AVR
                int8_t tTouchDeltaXGrid = aSwipeInfo->TouchDeltaX / 64;
#else
                int tTouchDeltaXGrid = aSwipeInfo->TouchDeltaX / 64;
#endif
                if (tTouchDeltaXGrid != 0) {
                    tFeedbackType = changeTimeBaseValue(-tTouchDeltaXGrid);
                    printInfo();
                }
            } else {
                // Vertical swipe
#ifndef AVR
                int tTouchDeltaYGrid = aSwipeInfo->TouchDeltaY / 32;
#endif
                if (!MeasurementControl.RangeAutomatic) {
#ifdef AVR
                    tFeedbackType = changeRange(aSwipeInfo->TouchDeltaY / 64);
#else
                    /*
                     * range manual. If offset not fixed, check if swipe in the right third of screen, then do changeOffsetGridCount()
                     */
                    if (MeasurementControl.OffsetMode != OFFSET_MODE_0_VOLT) {
                        // decide which swipe to perform according to x position of swipe
                        if (aSwipeInfo->TouchStartX > BUTTON_WIDTH_3_POS_2) {
                            //offset
                            tFeedbackType = changeOffsetGridCount(tTouchDeltaYGrid);
                        } else {
                            tFeedbackType = changeDisplayRangeAndAdjustOffsetGridCount(tTouchDeltaYGrid / 2);
                        }
                    }
                } else if (MeasurementControl.OffsetMode != OFFSET_MODE_0_VOLT) {
                    tFeedbackType = changeOffsetGridCount(tTouchDeltaYGrid);
#endif
                }
            }
        } else {
            /*
             * Analyze Mode scroll or scale
             */
#ifndef AVR
            if (aSwipeInfo->TouchStartY > BUTTON_HEIGHT_4_LINE_3 + TEXT_SIZE_22) {
#endif
            tFeedbackType = scrollChart(-aSwipeInfo->TouchDeltaX);
#ifndef AVR
        } else {
            // scale
            tFeedbackType = changeXScale(aSwipeInfo->TouchDeltaX / 64);
        }
#endif
        }
    }
#ifdef LOCAL_DISPLAY_EXISTS
    FeedbackTone(tFeedbackType);
#else
    BlueDisplay1.playFeedbackTone(tFeedbackType);
#endif
}

/************************************************************************
 * Button handler section
 ************************************************************************/
/*
 * default handler for back button
 */
void doDefaultBackButton(BDButton * aTheTouchedButton, int16_t aValue) {
    sBackButtonPressed = true;
}

/*
 * show gui of settings screen
 */
void doShowSettingsPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_SETTINGS;
    redrawDisplay();
}

void doShowFrequencyPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_FREQUENCY;
    startFrequencyGeneratorPage();
}

/*
 * toggle between ascending and descending trigger slope
 */
void doTriggerSlope(BDButton * aTheTouchedButton, int16_t aValue) {
    MeasurementControl.TriggerSlopeRising = (!MeasurementControl.TriggerSlopeRising);
    setTriggerLevelAndHysteresis(MeasurementControl.RawTriggerLevel, MeasurementControl.RawHysteresis);
    setSlopeButtonCaption();
}

/*
 * switch between automatic, manual, free and external trigger mode
 */
void doTriggerMode(BDButton * aTheTouchedButton, int16_t aValue) {
    uint8_t tNewMode = MeasurementControl.TriggerMode + 1;
    if (tNewMode > TRIGGER_MODE_EXTERN) {
        tNewMode = TRIGGER_MODE_AUTOMATIC;
        MeasurementControl.TriggerMode = tNewMode;
#ifdef AVR
        cli();
        if (EIMSK != 0) {
            // Release waiting for external trigger
            INT0_vect();
        }
        sei();
#endif
    }
    MeasurementControl.TriggerMode = tNewMode;
    setTriggerModeButtonCaption();
}

void doRangeMode(BDButton * aTheTouchedButton, int16_t aValue) {
    setAutoRangeModeAndButtonCaption(!MeasurementControl.RangeAutomatic);
}

/*
 * step from 0 Volt to auto to manual offset
 * No auto offset in AC Mode for AVR
 */
void doOffsetMode(BDButton * aTheTouchedButton, int16_t aValue) {
    MeasurementControl.OffsetMode++;
    if (MeasurementControl.OffsetMode > OFFSET_MODE_MANUAL) {
        // switch back from Mode Manual to mode 0 Volt and set range mode to automatic
        MeasurementControl.OffsetMode = OFFSET_MODE_0_VOLT;
        setAutoRangeModeAndButtonCaption(true);
#ifdef AVR
        MeasurementControl.OffsetValue = 0;
#else
        setOffsetGridCountAccordingToACMode();

    } else if (MeasurementControl.OffsetMode == OFFSET_MODE_MANUAL) {
// Offset mode manual implies range mode manual
        aTheTouchedButton->setCaptionAndDraw(AutoOffsetButtonStringMan);
        setAutoRangeModeAndButtonCaption(false);
        TouchButtonAutoRangeOnOff.deactivate();
#endif
    }
    setAutoOffsetButtonCaption();
}

/*
 * Cycle through all external and internal adc channels if button value is > 20
 */
void doChannelSelect(BDButton * aTheTouchedButton, int16_t aValue) {
#ifdef LOCAL_DISPLAY_EXISTS
    if (MeasurementControl.ADS7846ChannelsAsDatasource) {
// ADS7846 channels
        MeasurementControl.ADCInputMUXChannelIndex++;
        if (MeasurementControl.ADCInputMUXChannelIndex >= ADS7846_CHANNEL_COUNT) {
            // wrap to first channel connected to attenuator and restore ACRange
            MeasurementControl.ADCInputMUXChannelIndex = 0;
            MeasurementControl.isACMode = DSO_getACMode();
        }
        aTheTouchedButton->setCaption(ADS7846ChannelStrings[MeasurementControl.ADCInputMUXChannelIndex]);

    } else
#endif
    {

        uint8_t tNewChannelValue = aValue;
        if (tNewChannelValue > 20) {
            /*
             * aValue > 20 means TouchButtonChannelSelect was pressed, so increment button caption here ( "Ch. 3", "Ch. 4", "Temp." etc.)
             */
            uint8_t tOldValue = MeasurementControl.ADCInputMUXChannelIndex;
            // if channel 3 is not selected, increment channel, otherwise select channel 3
            if (tOldValue < NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR) {
                // first press on this button -> stay at channel 3
                tNewChannelValue = NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR;
            } else {
                tNewChannelValue = tOldValue + 1;
                uint8_t tCaptionIndex = tNewChannelValue;
                if (tNewChannelValue >= ADC_CHANNEL_COUNT) {
                    tNewChannelValue = 0;
                    //reset caption of 4. button to "Ch 3"
                    tCaptionIndex = NUMBER_OF_CHANNELS_WITH_FIXED_ATTENUATOR;
                }
                TouchButtonChannelSelect.setCaptionPGM(ADCInputMUXChannelStrings[tCaptionIndex]);
            }
        }
        setChannel(tNewChannelValue);
    }
    /*
     * Refresh page if necessary
     */
    // check it here since it is also called by setup
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        // manage AC/DC and auto offset buttons
        redrawDisplay();
    }
}

/*
 *  Toggle history mode
 */
void doChartHistory(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.showHistory = aValue;
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        aTheTouchedButton->drawButton();
    }

    if (DisplayControl.showHistory) {
        DisplayControl.EraseColor = COLOR_DATA_HISTORY;
    } else {
        DisplayControl.EraseColor = COLOR_BACKGROUND_DSO;
        if (MeasurementControl.isRunning) {
            // clear history on screen
            redrawDisplay();
        }
    }
}

/*
 * set to singleshot mode and draw an indicating "S" for AVR
 */
void doStartSingleshot(BDButton * aTheTouchedButton, int16_t aValue) {
    aTheTouchedButton->deactivate();
    DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;

#ifdef AVR
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    drawGridLinesWithHorizLabelsAndTriggerLine();
// draw an S to indicate running single shot trigger
    BlueDisplay1.drawChar(INFO_LEFT_MARGIN + SINGLESHOT_PPRINT_VALUE_X, INFO_UPPER_MARGIN + TEXT_SIZE_11_HEIGHT, 'S', TEXT_SIZE_11,
    COLOR_BLACK, COLOR_INFO_BACKGROUND);
#endif

    // prepare info output - which is shown at least 1 sec later
    sMillisSinceLastInfoOutput = 0;
    MeasurementControl.RawValueMax = 0;
    MeasurementControl.RawValueMin = 0;
    MeasurementControl.isSingleShotMode = true;

// Start a new single shot
#ifdef AVR
    DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];
    startAcquisition();
    MeasurementControl.isRunning = true;
#else
    prepareForStart();
#endif
}

/*
 * Slider is only activated if trigger mode == TRIGGER_MODE_MANUAL_TIMEOUT or TRIGGER_MODE_MANUAL
 */
void doTriggerLevel(BDSlider * aTheTouchedSlider, uint16_t aValue) {
// to get display value take DISPLAY_VALUE_FOR_ZERO - aValue and vice versa
    aValue = DISPLAY_VALUE_FOR_ZERO - aValue;
    if (DisplayControl.TriggerLevelDisplayValue == aValue) {
        return;
    }

    // clear old trigger line
    clearTriggerLine(DisplayControl.TriggerLevelDisplayValue);

    // store actual display value
    DisplayControl.TriggerLevelDisplayValue = aValue;

    // modify trigger values according to display value
    int tRawLevel = getInputRawFromDisplayValue(aValue);
    setTriggerLevelAndHysteresis(tRawLevel, 2); // 2 = value for effective trigger hysteresis in manual trigger mode

    // draw new line
    drawTriggerLine();
    // print value
    printTriggerInfo();
}

/*
 * The value printed has a resolution of 0,00488 * scale factor
 */
void doVoltagePicker(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    if (sLastPickerValue == aValue) {
        return;
    }
// clear old line
    int tYpos = DISPLAY_VALUE_FOR_ZERO - sLastPickerValue;
    BlueDisplay1.drawLine(0, tYpos, REMOTE_DISPLAY_WIDTH, tYpos, COLOR_BACKGROUND_DSO);
// restore grid at old y position
    for (unsigned int tXPos = TIMING_GRID_WIDTH - 1; tXPos < REMOTE_DISPLAY_WIDTH - 1; tXPos += TIMING_GRID_WIDTH) {
        BlueDisplay1.drawPixel(tXPos, tYpos, COLOR_GRID_LINES);
    }
#ifndef AVR
    if (!MeasurementControl.isRunning) {
        // restore graph
        uint8_t* ScreenBufferPointer = &DisplayBuffer[0];
        uint8_t* ScreenBufferMinPointer = &DisplayBufferMin[0];
        for (unsigned int i = 0; i < REMOTE_DISPLAY_WIDTH; ++i) {
            int tValueByte = *ScreenBufferPointer++;
            if (tValueByte == tYpos) {
                BlueDisplay1.drawPixel(i, tValueByte, COLOR_DATA_HOLD);
            }
            if (MeasurementControl.isEffectiveMinMaxMode) {
                tValueByte = *ScreenBufferMinPointer++;
                if (tValueByte == tYpos) {
                    BlueDisplay1.drawPixel(i, tValueByte, COLOR_DATA_HOLD);
                }
            }
        }
    }
#endif

// draw new line
    int tValue = DISPLAY_VALUE_FOR_ZERO - aValue;
    BlueDisplay1.drawLine(0, tValue, REMOTE_DISPLAY_WIDTH, tValue, COLOR_VOLTAGE_PICKER);
    sLastPickerValue = aValue;

    float tVoltage = getFloatFromDisplayValue(tValue);
#ifdef AVR
    dtostrf(tVoltage, 4, 2, sStringBuffer);
    sStringBuffer[4] = 'V';
    sStringBuffer[5] = '\0';
#else
    snprintf(sStringBuffer, sizeof sStringBuffer, "%6.*fV", RangePrecision[MeasurementControl.DisplayRangeIndex] + 1, tVoltage);
#endif

    int tYPos = SLIDER_VPICKER_INFO_SHORT_Y;
    if (DisplayControl.showInfoMode == INFO_MODE_NO_INFO) {
        tYPos = FONT_SIZE_INFO_SHORT_ASC;
    } else if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        tYPos = SLIDER_VPICKER_INFO_LONG_Y;
    }
// print value
    BlueDisplay1.drawText(SLIDER_VPICKER_INFO_X, tYPos, sStringBuffer, FONT_SIZE_INFO_SHORT, COLOR_BLACK,
    COLOR_INFO_BACKGROUND);
}

#ifdef AVR
/*
 * Request delay value as number
 */
void doPromptForTriggerDelay(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPromptPGM(&doSetTriggerDelay, PSTR("Trigger delay [\xB5s]"), NUMBER_INITIAL_VALUE_DO_NOT_SHOW);
}
#else
void doShowPretriggerValuesOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DatabufferPreTriggerDisplaySize = 0;
    if (aValue) {
        DisplayControl.DatabufferPreTriggerDisplaySize = (2 * DATABUFFER_DISPLAY_RESOLUTION);
    }
}

/*
 * Sets only the flag and button caption
 */
void doMinMaxMode(BDButton * aTheTouchedButton, int16_t aValue) {
    MeasurementControl.isMinMaxMode = aValue;
    if (MeasurementControl.TimebaseEffectiveIndex >= TIMEBASE_INDEX_CAN_USE_OVERSAMPLING) {
// changeTimeBase() manages oversampling rate for Min/Max oversampling
        if (MeasurementControl.isRunning) {
            // signal to main loop in thread mode
            MeasurementControl.ChangeRequestedFlags |= CHANGE_REQUESTED_TIMEBASE_FLAG;
        } else {
            changeTimeBase();
        }
    }
}
/*
 * show gui of more settings screen
 */
void doShowMoreSettingsPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_MORE_SETTINGS;
    startDSOMoreSettingsPage();
}

void doShowSystemInfoPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_SYST_INFO;
    startSystemInfoPage();
}

/**
 * 3ms for FFT, 9ms complete with -OS
 */
void doShowFFT(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.ShowFFT = aValue;
    aTheTouchedButton->setValue(aValue);

    if (MeasurementControl.isRunning) {
        if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            aTheTouchedButton->drawButton();
        }
        if (aValue) {
            // initialize FFTDisplayBuffer
            memset(&DisplayBufferFFT[0], REMOTE_DISPLAY_HEIGHT - 1, sizeof(DisplayBufferFFT));
        } else {
            clearFFTValuesOnDisplay();
        }
    } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        if (aValue) {
            // compute and draw FFT
            drawFFT();
        } else {
            // show graph data
            redrawDisplay();
        }
    }
}
#endif

#ifdef LOCAL_DISPLAY_EXISTS
/*
 * Toggle between pixel and line draw mode (for data chart)
 */
void doDrawMode(BDButton * aTheTouchedButton, int16_t aValue) {
// erase old chart in old mode
    drawDataBuffer(NULL, REMOTE_DISPLAY_WIDTH, DisplayControl.EraseColor, 0, DRAW_MODE_CLEAR_OLD,
            MeasurementControl.isEffectiveMinMaxMode);
// switch mode
    if (!DisplayControl.drawPixelMode) {
        aTheTouchedButton->setCaptionAndDraw(DrawModeButtonStringPixel);
    } else {
        aTheTouchedButton->setCaptionAndDraw(DrawModeButtonStringLine);
    }
    DisplayControl.drawPixelMode = !DisplayControl.drawPixelMode;
}

/*
 * Toggles ADS7846Test on / off
 */
void doADS7846TestOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    aValue = !aValue;
    MeasurementControl.ADS7846ChannelsAsDatasource = aValue;
    MeasurementControl.ADCInputMUXChannelIndex = 0;
    if (aValue) {
// ADS7846 Test on
        doAcDcMode(&TouchButtonAcDc, true);
        MeasurementControl.ChannelHasActiveAttenuator = false;
        setDisplayRange(NO_ATTENUATOR_MAX_DISPLAY_RANGE_INDEX);
    } else {
        MeasurementControl.ChannelHasActiveAttenuator = true;
    }
    aTheTouchedButton->setValueAndDraw(aValue);
}
#endif

uint32_t getMicrosFromHorizontalDisplayValue(uint16_t aDisplayValueHorizontal, uint8_t aNumberOfPeriods) {
#ifdef AVR
    uint32_t tMicros = aDisplayValueHorizontal * pgm_read_float(&TimebaseExactDivValuesMicros[MeasurementControl.TimebaseIndex]);
#else
    uint32_t tMicros = aDisplayValueHorizontal * getDataBufferTimebaseExactValueMicros(MeasurementControl.TimebaseEffectiveIndex);
#endif
    return (tMicros / (aNumberOfPeriods * TIMING_GRID_WIDTH));
}
