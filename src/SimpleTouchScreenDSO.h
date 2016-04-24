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

#include "BDButton.h"

/*
 * Change this if you have reprogrammed the hc05 module for other baud rate
 */
#define HC_05_BAUD_RATE BAUD_115200

/*
 *  Display size
 */
const unsigned int REMOTE_DISPLAY_HEIGHT = 256;
const unsigned int REMOTE_DISPLAY_WIDTH = 320;

#define THOUSANDS_SEPARATOR '.'

/*
 * Pins on port D
 * AC/DC, attenuator control, AC/DC sense and external trigger input
 * !!! Pin layout only for Atmega328 !!!
 */
#define CONTROL_PORT PORTD
#define CONTROL_DDR  DDRD
#define EXTERN_TRIGGER_INPUT_PIN 2// PD2
#define AC_DC_PIN        3  // PD3/PCINT19
// Control
#define ATTENUATOR_SHIFT 4 // Bit 4+5
#define ATTENUATOR_MASK 0x30 // Bit 4+5
#define CONTROL_MASK    0xF0 // Bit 4-7
#define ATTENUATOR_0_PIN 4 // PD4
#define ATTENUATOR_1_PIN 5 // PD5
#define AC_DC_RELAIS_PIN_1 6 // PD6
#define AC_DC_RELAIS_PIN_2 7 // PD7

/*
 * Pins on port B
 */
#define OUTPUT_MASK_PORTB   0X0C
#define ATTENUATOR_DETECT_PIN_0 8 // PortB0
#define ATTENUATOR_DETECT_PIN_1 9 // PortB1
#define TIMER_1_OUTPUT_PIN 10 // Frequency generation OC1B TIMER1
#define VEE_PIN 11 // OC2A TIMER2 Square wave for VEE (-5V) generation

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
#define COLOR_DATA_PICKER_SLIDER RGB(0xFF,0XFF,0xE0) // Light yellow
#define COLOR_TRIGGER_LINE COLOR_MAGENTA
#define COLOR_TRIGGER_SLIDER RGB(0xFF,0XF0,0xFF)
#define COLOR_MAX_MIN_LINE 0X0200 // light green
#define COLOR_HOR_REF_LINE_LABEL COLOR_BLUE
#define COLOR_TIMING_LINES RGB(0x00,0x98,0x00)

// GUI element colors
#define COLOR_GUI_CONTROL RGB(0xE8,0x00,0x00)
#define COLOR_GUI_TRIGGER RGB(0x00,0x00,0xFF) // blue
#define COLOR_GUI_SOURCE_TIMEBASE RGB(0x00,0xE0,0x00)

#define COLOR_INFO_BACKGROUND RGB(0xC8,0xC8,0x00)

#define COLOR_SLIDER RGB(0xD0,0xD0,0xD0)

/*
 * POSITIONS + SIZES
 */
#define INFO_UPPER_MARGIN (1 + TEXT_SIZE_11_ASCEND)
#define INFO_LEFT_MARGIN 0

#define FONT_SIZE_INFO_SHORT        TEXT_SIZE_18    // for 1 line info
#define FONT_SIZE_INFO_LONG         TEXT_SIZE_11    // for 2 lines info
#define FONT_SIZE_INFO_SHORT_ASC    TEXT_SIZE_18_ASCEND
#define FONT_SIZE_INFO_LONG_ASC     TEXT_SIZE_11_ASCEND
#define FONT_SIZE_INFO_LONG_WIDTH   TEXT_SIZE_11_WIDTH

#define SLIDER_SIZE 24
#define SLIDER_VPICKER_POS_X        0 // Position of slider
#define SLIDER_VPICKER_INFO_X       (SLIDER_VPICKER_POS_X + SLIDER_SIZE)
#define SLIDER_VPICKER_INFO_SHORT_Y (FONT_SIZE_INFO_SHORT + FONT_SIZE_INFO_SHORT_ASC)
#define SLIDER_VPICKER_INFO_LONG_Y  (2 * FONT_SIZE_INFO_LONG + FONT_SIZE_INFO_SHORT_ASC) // since font size is always 18

#define SLIDER_TLEVEL_POS_X         (14 * FONT_SIZE_INFO_LONG_WIDTH) // Position of slider
#define TRIGGER_LEVEL_INFO_SHORT_X  (SLIDER_TLEVEL_POS_X  + SLIDER_SIZE)
#define TRIGGER_LEVEL_INFO_LONG_X   ((35 * FONT_SIZE_INFO_LONG_WIDTH) + 1) // +1 since we have a special character in the string before
#define TRIGGER_LEVEL_INFO_SHORT_Y  (FONT_SIZE_INFO_SHORT + FONT_SIZE_INFO_SHORT_ASC)
#define TRIGGER_LEVEL_INFO_LONG_Y   FONT_SIZE_INFO_LONG_ASC

/*
 * Trigger values
 */
#define TRIGGER_MODE_AUTO 0
#define TRIGGER_MODE_MANUAL 1
#define TRIGGER_MODE_FREE 2 // waits at least 23 ms (255 samples) for trigger
#define TRIGGER_MODE_EXTERN 3

// No trigger wait timeout for modes != TRIGGER_DELAY_NONE
#define TRIGGER_DELAY_NONE 0
#define TRIGGER_DELAY_MICROS 1
#define TRIGGER_DELAY_MILLIS 2

#define TRIGGER_DELAY_MICROS_POLLING_ADJUST_COUNT 1 // estimated value to be subtracted from value because of fast mode initial delay
#define TRIGGER_DELAY_MICROS_ISR_ADJUST_COUNT 4 // estimated value to be subtracted from value because of ISR initial delay

// States of tTriggerStatus
#define TRIGGER_STATUS_START 0 // No trigger condition met
#define TRIGGER_STATUS_BEFORE_THRESHOLD 1 // slope condition met, wait to go beyond threshold hysteresis
#define TRIGGER_STATUS_FOUND 2 // Trigger condition met - Used for shorten ISR handling
#define TRIGGER_STATUS_FOUND_AND_WAIT_FOR_DELAY 3 // Trigger condition met and waiting for ms delay
#define TRIGGER_STATUS_FOUND_AND_NOW_GET_ONE_VALUE 4 // Trigger condition met (and delay gone), now get first value for min/max initialization

/*
 * External attenuator values
 */
#define ATTENUATOR_TYPE_NO_ATTENUATOR 0
#define ATTENUATOR_TYPE_FIXED_ATTENUATOR 1  // assume manual AC/DC switch
#define NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR 3 // Channel0 = /1, Ch1= /10, Ch2= /100

#define ATTENUATOR_TYPE_ACTIVE_ATTENUATOR 2 // and 3
#define NUMBER_OF_CHANNEL_WITH_ACTIVE_ATTENUATOR 2

#define MAX_ADC_CHANNEL 5

struct MeasurementControlStruct {
	// State
	bool isRunning;
	bool StopRequested;
	// Used to disable trigger timeout and to specify full buffer read with stop after first read.
	bool isSingleShotMode;

	float VCC; // Volt of VCC
	uint8_t ADCReference; // DEFAULT = 1 =VCC   INTERNAL = 3 = 1.1V

	// Input select
	uint8_t ADCInputMUXChannel;
	char ADCInputMUXChannelChar;
	uint8_t AttenuatorType; //ATTENUATOR_TYPE_NO_ATTENUATOR, ATTENUATOR_TYPE_SIMPLE_ATTENUATOR, ATTENUATOR_TYPE_ACTIVE_ATTENUATOR
	bool ChannelHasActiveAttenuator;
	bool ChannelHasACDCSwitch; // has AC / DC switch - only for channels with active or passive attenuators
	bool ChannelIsACMode; // AC Mode for actual channel
	bool isACMode; // user AC mode setting
	volatile uint8_t ACModeFromISR; // 0 -> DC, 1 -> AC, 2 -> request was processed
	uint16_t RawDSOReadingACZero;

	// Trigger
	bool TriggerSlopeRising;
	uint16_t RawTriggerLevel;
	uint16_t TriggerLevelUpper;
	uint16_t TriggerLevelLower;
	uint16_t ValueBeforeTrigger;

	uint32_t TriggerDelayMillisEnd; // value of millis() at end of milliseconds trigger delay
	uint16_t TriggerDelayMillisOrMicros;
	uint8_t TriggerDelay; //  TRIGGER_DELAY_NONE 0, TRIGGER_DELAY_MICROS 1, TRIGGER_DELAY_MILLIS 2

	// Using type TriggerMode instead of uint8_t increases program size by 76 bytes
	uint8_t TriggerMode; // adjust values automatically
	bool OffsetAutomatic; // false -> offset = 0 Volt
	uint8_t TriggerStatus; //TRIGGER_STATUS_START 0, TRIGGER_STATUS_BEFORE_THRESHOLD 1, TRIGGER_STATUS_OK 2
	uint8_t TriggerSampleCountPrecaler; // for dividing sample count by 256 - to avoid 32bit variables in ISR
	uint16_t TriggerSampleCountDividedBy256; // for trigger timeout
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
	uint8_t TimebaseHWValue;
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
// using enums increases code size by 120 bytes
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

extern char sDataBuffer[50];

extern BDButton TouchButtonBack;

#endif //SIMPLETOUCHSCREENDSO_H_
