/*
 * SimpleTouchScreenDSO.h
 *
 *
 *  Created on:  01.01.2015
 *      Author:  Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmx.de
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *      Version: 1.0.0
 */

#ifndef SIMPLETOUCHSCREENDSO_H_
#define SIMPLETOUCHSCREENDSO_H_

// use simple serial otherwise the usart interrupt kills the timing
#ifndef USE_SIMPLE_SERIAL
#define USE_SIMPLE_SERIAL
#endif

/*
 * Change this if you have reprogrammed the hc05 module for other baud rate
 */
#define HC_05_BAUD_RATE BAUD_115200

// Display size
#define DISPLAY_HEIGHT 256
#define DISPLAY_WIDTH 320

#define THOUSANDS_SEPARATOR '.'

#define MAX_ADC_CHANNEL 5

/*
 * COLORS
 */
#define COLOR_BACKGROUND_DSO COLOR_WHITE

// Data colors
#define COLOR_DATA_RUN COLOR_BLUE
#define COLOR_DATA_HOLD COLOR_RED
// to see old chart values
#define COLOR_DATA_HISTORY RGB(0x20,0xFF,0x20)

//Line colors
#define COLOR_DATA_PICKER COLOR_YELLOW
#define COLOR_MAX_MIN_LINE 0X0200 // light green
#define COLOR_HOR_REF_LINE_LABEL COLOR_BLUE
#define COLOR_TRIGGER_LINE COLOR_MAGENTA
#define COLOR_TIMING_LINES RGB(0x00,0x98,0x00)

// GUI element colors
#define COLOR_GUI_CONTROL RGB(0xC0,0x00,0x00)
#define COLOR_GUI_TRIGGER RGB(0x00,0x00,0xD0) // blue
#define COLOR_GUI_SOURCE_TIMEBASE RGB(0x00,0x90,0x00)
#define COLOR_GUI_DISPLAY_CONTROL RGB(0xC8,0xC8,0x00)
#define COLOR_GUI_SELECTED COLOR_GREEN
#define COLOR_GUI_NOT_SELECTED COLOR_RED

#define COLOR_INFO_BACKGROUND RGB(0xC8,0xC8,0x00)

#define COLOR_SLIDER RGB(0xD8,0xE8,0xD8)

#define TRIGGER_MODE_AUTO 0
#define TRIGGER_MODE_MANUAL 1
#define TRIGGER_MODE_FREE 2

#define ATTENUATOR_TYPE_NO_ATTENUATOR 0
#define ATTENUATOR_TYPE_SIMPLE_ATTENUATOR 1
#define MAX_ADC_CHANNEL_WITH_SIMPLE_ATTENUATOR 2
#define ATTENUATOR_TYPE_ACTIVE_ATTENUATOR 2 // and 3
#define MAX_ADC_CHANNEL_WITH_ACTIVE_ATTENUATOR 1

struct MeasurementControlStruct {
    // State
    bool isRunning;
    bool StopRequested;
    // Trigger flag for ISR and single shot mode
    bool searchForTrigger;
    bool isSingleShotMode;

    float VCC; // Volt of VCC
    uint8_t ADCReference; // DEFAULT = 1 =VCC   INTERNAL = 3 = 1.1V

    // Input select
    uint8_t ADCInputMUXChannel;
    char ADCInputMUXChannelChar;
    uint8_t AttenuatorType; //ATTENUATOR_TYPE_NO_ATTENUATOR, ATTENUATOR_TYPE_SIMPLE_ATTENUATOR, ATTENUATOR_TYPE_ACTIVE_ATTENUATOR
    bool ChannelHasActiveAttenuator;
    bool ChannelHasACDC; // has AC / DC switch - only for channels with active or passive attenuators
    bool isACMode; // AC mode for attenuator channels
    bool storeForACMode;
    uint16_t RawDSOReadingACZero;

    // Trigger
    bool TriggerSlopeRising;
    uint16_t RawTriggerLevel;
    uint16_t TriggerLevelUpper;
    uint16_t TriggerLevelLower;
    uint16_t ValueBeforeTrigger;

    uint8_t TriggerMode; // adjust values automatically
    bool OffsetAutomatic; // false -> offset = 0 Volt
    uint8_t TriggerStatus;
    uint16_t TriggerSampleCount; // for trigger timeout
    uint16_t TriggerTimeoutSampleCount; // ISR max samples before trigger timeout

    // Statistics (for info and auto trigger)
    uint16_t RawValueMin;
    uint16_t RawValueMax;
    uint16_t ValueMinForISR;
    uint16_t ValueMaxForISR;
    uint16_t ValueAverage;
    uint32_t IntegrateValueForAverage;
    uint32_t PeriodMicros;

    // Timebase
    bool TimebaseFastFreerunningMode;
    uint8_t TimebaseIndex;
    // volatile saves 2 registers push in ISR
    // delay loop duration - 1/4 micros resolution
    volatile uint16_t TimebaseDelay;
    // remaining micros for long delays - 1/4 micros resolution
    uint16_t TimebaseDelayRemaining;

    bool RangeAutomatic; // RANGE_MODE_AUTOMATIC, MANUAL

    // Shift and scale
    uint16_t OffsetValue;
    uint8_t AttenuatorValue; // 0 for direct input or channels without attenuator, 1 -> factor 10, 2 -> factor 100, 3 -> input shortcut
    uint8_t ShiftValue; // shift (division) value  (0-2) for different voltage ranges
    uint16_t HorizontalGridSizeShift8; // depends on shift  for 5V reference 0,02 -> 41 other -> 51.2
    float HorizontalGridVoltage; // voltage per grid for offset etc.
    int8_t OffsetGridCount; // number of bottom line for offset != 0 Volt.
    uint32_t TimestampLastRangeChange;
};

extern struct MeasurementControlStruct MeasurementControl;

// values for DisplayPage
#define DISPLAY_PAGE_START 0    // Start GUI
#define DISPLAY_PAGE_CHART 1    // Chart in analyze and running mode
#define DISPLAY_PAGE_SETTINGS 2
#define DISPLAY_PAGE_FREQUENCY 3

// modes for showInfoMode
#define INFO_MODE_NO_INFO 0
#define INFO_MODE_SHORT_INFO 1
#define INFO_MODE_LONG_INFO 2
struct DisplayControlStruct {
    uint8_t TriggerLevelDisplayValue; // For clearing old line of manual trigger level setting
    int8_t XScale; // Factor for X Data expansion(>0)  0 = no scale, 2->display 1 value 2 times etc.
    uint8_t DisplayPage;
    bool DrawWhileAcquire;
    uint8_t showInfoMode;
    bool showHistory;
    uint16_t EraseColor;
};
extern DisplayControlStruct DisplayControl;

extern char StringBuffer[50];

extern uint8_t TouchButtonBack;

void FeedbackTone(bool isNoError);

#endif //SIMPLETOUCHSCREENDSO_H_
