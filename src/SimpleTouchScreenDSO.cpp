/*
 *
 *  Created on: 29.03.2012
 *      Author: Armin Joachimsmeyer
 *      Email: armin.joachimsmeyer@gmail.com
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 *      Sources: https://github.com/ArminJo/Arduino-Simple-DSO
 *
 *      Features:
 *      No dedicated hardware, just a plain arduino, a HC-05 Bluetooth module and this software.
 *      Full touch screen control of all parameters.
 *      150/300 kSamples per second
 *      supports AC Measurement with (passive) external attenuator circuit.
 *      3 external circuits detected by software - no attenuator, passive attenuator /1, /10, /100, active attenuator.
 *      Automatic trigger, range and offset value selection
 *      1120 Byte data buffer - 3.5 * displays
 *      Min, max, average and peak to peak display
 *      Period and frequency display
 *      All settings can be changed during measurement
 *      Gesture (swipe) control of timebase and chart
 *      All AVR ADC input channels selectable
 *      Touch trigger level select
 *      1.1 Volt internal reference. 5 Volt (VCC) also usable.
 *
 *      The code can also be used as an example of C++/Assembler coding
 *      and non trivial interrupt handling routine
 *
 */

/*
 * SIMPLE EXTERNAL ATTENUATOR CIRCUIT (configured by connecting pin 10 to ground)
 *
 * Safety circuit and AC/DC switch
 * 3 resistors   2 diodes   1 capacitor   1 switch
 *
 *                ADC INPUT_0  1.1 Volt         ADC INPUT_1 11 Volt        ADC INPUT_2 11 Volt
 *                      /\                         /\                         /\
 *                      |                          |                          |
 *                      |                          +-----| 220k |----+        |
 *                      +-----| >4 M |----+        +-----| 220k |----+        +-----| 10 k |----+
 *                      |                 |        |                 |        |                 |
 *                      _                 |        _                 |        _                 |
 *                     | |                |       | |                |       | |                |
 *                     | | 10 k           |       | | 1 M            |       | | 1 M            |
 *                     | |                |       | |                |       | |                |
 *                      -                 |        -                 |        -                 |
 *                      |                 |        |                 |        |                 |
 *                      +----+            |        +----+            |        +----+            |
 *                      |    |            |        |    |            |        |    |            |
 *                      |    = C 0.1uF    |        |    = C 0.1uF    |        |    = C 0.1uF    |
 *                      |    |            |        |    |            |        |    |            |
 *                      O    O            |        O    O            |        O    O            |
 *                     DC   AC            |       DC   AC            |       DC   AC            |
 *                                        |                          |                          |
 *                       +----------------+--------------------------+--------------------------+
 *                       |
 *                       O
 *           AC/DC      /
 *           Switch    /
 *                   O/    O----------+
 *                AC |     DC         |
 *                   |                |
 *   VREF-| 100k |---+------| 100k |--+
 *                   |                |
 *                   +--------||------+-GND
 *                          33 uF
 *
 */

/*
 * since 5.0 / 1024.0 = 0,004883 Volt is the resolution of the ADC
 * depending of scale factor:
 * 1. The output values for 2 adjacent display (min/max) values can be identical
 * 2. The voltage picker value may not reflect the real sample value (e.g. shown for min/max)
 */

//#define DEBUG
#include <Arduino.h>

#include "SimpleTouchScreenDSO.h"
#include "FrequencyGenerator.h"
#include "TouchLib.h"
#include "BlueDisplay.h"
#include "BlueSerial.h"
#include "digitalWriteFast.h"
#define digitalToggleFast(P) BIT_SET(* &PINB, __digitalPinToBit(P))
#include <avr/interrupt.h>

/**
 * PINS (optional)
 * 2    Attenuator range control 0
 * 3    Attenuator range control 1
 * 4    AC / DC for attenuator
 * 5    AC / DC relais 1
 * 6    AC / DC relais 2
 * 7    Debug output
 * 8    Attenuator detect input with internal pullup - bit 0
 * 9    Attenuator detect input with internal pullup - bit 1    11-> no attenuator attached, 10-> simple (channel 0-2) attenuator attached, 11-> active (channel 0-1) attenuator attached
 * 10   TIMER1 - Frequency generator output
 * 11   Timer2 - Square wave for VEE (-5V) generation
 * 12
 * 13
 */

// Data buffer size (must be small enough to leave appr. 7 % (144 Byte) for stack
#define DATABUFFER_SIZE (3*DISPLAY_WIDTH + DISPLAY_WIDTH/2) //1120
// Acquisition start values
#define TIMEBASE_INDEX_START_VALUE 7 // 2ms - shows 50 Hz
#define CONTROL_PORT PORTD
#define CONTROL_DDR  DDRD
#define ATTENUATOR_MASK 0x0C // Bit 2+3
#define CONTROL_MASK    0x7C // Bit 2-6
#define ATTENUATOR_0_PIN 2
#define ATTENUATOR_1_PIN 3
#define AC_DC_PIN        4
#define AC_DC_RELAIS_PIN_1 5
#define AC_DC_RELAIS_PIN_2 6

#define OUTPUT_MASK_PORTB   0X0C
#define ATTENUATOR_DETECT_PIN_0 8 // PortB0
#define ATTENUATOR_DETECT_PIN_1 9
#define TIMER_1_OUTPUT_PIN 10 // Frequency generation OC1B TIMER1
#define VEE_PIN 11 // OC2A TIMER2 Square wave for VEE (-5V) generation
#define INFO_UPPER_MARGIN (1 + TEXT_SIZE_11_ASCEND)
#define INFO_LEFT_MARGIN 0

/**********************
 * Buttons
 *********************/
uint8_t TouchButtonStartStop;

uint8_t TouchButtonAutoTriggerOnOff;

// the warning "only initialized variables can be placed into program memory area"
// is because of a known bug of the gnu avr complier version
const char AutoTriggerButtonStringAuto[] PROGMEM = "Trigger auto";
const char AutoTriggerButtonStringManual[] PROGMEM = "Trigger man";
const char AutoTriggerButtonStringFree[] PROGMEM = "Trigger free";

uint8_t TouchButtonAutoOffsetOnOff;
const char AutoOffsetButtonStringAuto[] PROGMEM = "Offset auto";
const char AutoOffsetButtonString0[] PROGMEM = "Offset 0V";

uint8_t TouchButtonADCReference;
const char ReferenceButtonVCC[] PROGMEM = "Ref VCC";
const char ReferenceButton1_1V[] PROGMEM = "Ref 1.1V";

uint8_t TouchButtonAcDc;
const char AcDcButtonDC[] PROGMEM = "DC";
const char AcDcButtonAC[] PROGMEM = "AC";

#define ADC_DIRECT_CHANNEL_BUTTONS 3
uint8_t TouchButtonChannels[ADC_DIRECT_CHANNEL_BUTTONS];
uint8_t TouchButtonChannelSelect;
uint8_t TouchButtonChannelMode;

#define ADC_TEMPERATURE_CHANNEL 8
#define ADC_1_1_VOLT_CHANNEL 0x0E
char ChannelSelectButtonString[] = "Ch  ";
#define CHANNEL_STRING_INDEX 3 // the index of the channel number in char array
const char Channel3ButtonString[] PROGMEM = "Ch 3";

const char ChannelDivBy1ButtonString[] PROGMEM = "\xF7" "1";
const char ChannelDivBy10ButtonString[] PROGMEM = "\xF7" "10";
const char ChannelDivBy100ButtonString[] PROGMEM = "\xF7" "100";
const char * const ChannelDivByButtonStrings[] = { ChannelDivBy1ButtonString, ChannelDivBy10ButtonString,
        ChannelDivBy100ButtonString };

uint8_t TouchButtonSettings;
uint8_t TouchButtonBack;

uint8_t TouchButtonSingleshot;
#define SINGLESHOT_PPRINT_VALUE_X (37 * TEXT_SIZE_11_WIDTH)

uint8_t TouchButtonAutoRangeOnOff;
const char AutoRangeButtonStringAuto[] PROGMEM = "Range auto";
const char AutoRangeButtonStringManual[] PROGMEM = "Range man";

uint8_t TouchButtonSlope;
char SlopeButtonString[] = "Slope A";
// the index of the slope indicator in char array
#define SLOPE_STRING_INDEX 6

uint8_t TouchButtonChartHistoryOnOff;

/*
 * Slider for trigger level and voltage picker
 */
#define SLIDER_TLEVEL_VALUE_X (24 * TEXT_SIZE_11_WIDTH)
#define SLIDER_VPICKER_VALUE_X (19 * TEXT_SIZE_11_WIDTH)

uint8_t TouchSliderTriggerLevel;

uint8_t TouchSliderVoltagePicker;
uint8_t LastPickerValue;

/*****************************
 * Timebase stuff
 *****************************/
// ADC HW prescaler values
#define PRESCALE4    2 // is noisy
#define PRESCALE8    3 // is reasonable
#define PRESCALE16   4
#define PRESCALE32   5
#define PRESCALE64   6
#define PRESCALE128  7

#define PRESCALE_MIN_VALUE PRESCALE4
#define PRESCALE_MAX_VALUE PRESCALE128
#define PRESCALE_START_VALUE PRESCALE128

/*
 * Don't need to use timer for timebase, since the ADC is also driven by clock and with a few delay times of the ISR
 * one can get reproducible timings just with the ADC conversion timing!
 *
 * Different Acquisition modes depending on Timebase:
 * Mode ultrafast  10us - ADC free running - one loop for read and store 10 bit => needs double buffer space - interrupts blocked for duration of loop
 * Mode fast      201us - ADC free running - one loop for read pre process 10 -> 8 Bit and store - interrupts blocked for duration of loop
 * mode isr without delay 496us   - ADC generates Interrupts - because of ISR initial delay for push just start next conversion immediately
 * mode isr with delay    1,2,5ms - ADC generates Interrupts - busy delay, then start next conversion to match 1,2,5 timebase scale
 * mode isr with multiple read >=10ms - ADC generates Interrupts - to avoid excessive busy delays, start one ore more intermediate conversion just for delay purposes
 */

#define HORIZONTAL_GRID_COUNT 6
/**
 * Formula for Grid Height is:
 * 5V Reference, 10 bit Resolution => 1023/5 = 204.6 Pixel per Volt
 * 1 Volt per Grid -> 204,6 pixel. With scale (shift) 2 => 51.15 pixel.
 * 0.5 Volt -> 102.3 pixel with scale (shift) 1 => 51.15 pixel
 * 0.2 Volt -> 40.96 pixel
 * 1.1V Reference 1023/1.1 = 930 Pixel per Volt
 * 0.2 Volt -> 186 pixel with scale (shift) 2 => 46.5 pixel
 * 0.1 Volt -> 93 pixel with scale (shift) 1 => 46.5 pixel
 * 0.05 Volt -> 46.5 pixel
 */
#define HORIZONTAL_GRID_HEIGHT_1_1V_SHIFT8 11904 // 46.5*256 for 0.05 to 0.2 Volt/div for 6 divs per screen
#define HORIZONTAL_GRID_HEIGHT_2V_SHIFT8 6554 // 25.6*256 for 0.05 to 0.2 Volt/div for 10 divs per screen
#define ADC_CYCLES_PER_CONVERSION 13
#define TIMING_GRID_WIDTH 31 // with 31 instead of 32 the values fit better to 1-2-5 timebase scale
#define TIMEBASE_NUMBER_OF_ENTRIES 15 // the number of different timebases provided
#define TIMEBASE_NUMBER_OF_FAST_PRESCALE 8 // the number of prescale values not equal slowest possible prescale (PRESCALE128)
#define TIMEBASE_INDEX_ULTRAFAST_MODES 0 // first mode is ultrafast
#define TIMEBASE_INDEX_FAST_MODES 4 // first 5 modes are fast free running modes
#define TIMEBASE_NUMBER_OF_XSCALE_CORRECTION 4  // number of timebase which are simulated by display XSale factor
#define TIMEBASE_INDEX_MILLIS 6 // min index to switch to ms instead of us display
#define TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE 11 // min index where chart is drawn while buffer is filled (11 => 50ms)
// the delay between ADC end of conversion and first start of delay loop in ISR
#define ISR_ZERO_DELAY_MICROS 3
#define ISR_DELAY_MICROS 6 // delay from interrupt to delay code
#define ADC_CONVERSION_AS_DELAY_MICROS 112 // prescaler 128 => 104us per conversion + 8us for 1 clock delay because of manual restarting
/*
 * For prescale 4 is: 13*0.25 = 3.25us per conversion
 * 8->6.5us, 16->13us ,32->2*13=26 for 1ms Range, 64->51, 128->8*13=104us per conversion
 *
 * Resolution of TimebaseDelayValues is 1/4 microsecond
 * +1 -10 and -24 are manual correction values for the 1/4 microsecond resolution
 */
const uint16_t TimebaseDelayValues[TIMEBASE_NUMBER_OF_ENTRIES] = { 0, 0, 0, 0, 0, 3 - ISR_ZERO_DELAY_MICROS, //
        ((16 - ADC_CYCLES_PER_CONVERSION) * 2 - ISR_DELAY_MICROS) * 4 + 1, // =1   |   1 -   3  | 1ms Range
        ((16 - ADC_CYCLES_PER_CONVERSION) * 4 - ISR_DELAY_MICROS) * 4 - 10, // =14  |  13 -  27 | 2ms
        ((20 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, // =176 | 172 - 203 | 5ms
        ((40 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, // =816 | 812 - 843 | 10ms
        ((81 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, // =2128|
        ((202 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, // 50ms Range
        ((403 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, //
        ((806 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24, //
        ((uint16_t) ((2016 - ADC_CYCLES_PER_CONVERSION) * 8 - ISR_DELAY_MICROS) * 4 - 24), // 64048
        };
const uint8_t xScaleForTimebase[TIMEBASE_NUMBER_OF_XSCALE_CORRECTION] = { 10, 10, 4, 2 }; // for GUI and frequency
const uint8_t PrescaleValueforTimebase[TIMEBASE_NUMBER_OF_FAST_PRESCALE] = { PRESCALE4, PRESCALE8, PRESCALE8, PRESCALE8, PRESCALE8,
PRESCALE16, PRESCALE32, PRESCALE64 };
// for 31 grid
const uint16_t TimebaseDivPrintValues[TIMEBASE_NUMBER_OF_ENTRIES] PROGMEM = { 10, 20, 50, 101, 201, 496, 1, 2, 5, 10, 20, 50, 100,
        200, 500 };
// exact values for 31 grid - for period and frequency
const float TimebaseDivExactValues[TIMEBASE_NUMBER_OF_ENTRIES] PROGMEM
= { 100.75/*(31*13*0,25)*/, 201.5, 201.5, 201.5, 201.5 /*(31*13*0,5)*/, 496 /*(31*16*1)*/, 992 /*(31*16*2)*/, 1984 /*(31*16*4)*/,
        4960 /*(31*20*8)*/, 9920 /*(31*40*8)*/, 20088 /*(31*81*8)*/, 50096 /*(31*202*8)*/, 99944 /*(31*403*8)*/,
        199888 /*(31*806*8)*/, 499968 /*(31*2016*8)*/};

/****************************************
 * Automatic triggering and range stuff
 */
#define TRIGGER_WAIT_NUMBER_OF_SAMPLES 3300 // Number of samples (<=112us) used for detecting the trigger condition
// States of tTriggerStatus
#define TRIGGER_START 0 // No trigger condition met
#define TRIGGER_BEFORE_THRESHOLD 1 // slope condition met, wait to go beyond threshold hysteresis
#define TRIGGER_OK 2 // Trigger condition met
#define TRIGGER_HYSTERESIS_MANUAL 2 // value for effective trigger hysteresis in manual trigger mode
#define TRIGGER_HYSTERESIS_MIN 0x08 // min value for automatic effective trigger hysteresis
#define ADC_CYCLES_PER_CONVERSION 13
#define SCALE_CHANGE_DELAY_MILLIS 2000

#define ADC_MAX_CONVERSION_VALUE (1024 -1) // 10 bit
#define ATTENUATOR_FACTOR 10
#define DISPLAY_AC_ZERO_OFFSET_GRID_COUNT (-3)

/******************************
 * Measurement control values
 *****************************/
struct MeasurementControlStruct MeasurementControl;

/*
 * Display control
 * while running switch between upper info line on/off
 * while stopped switch between chart / t+info line and gui
 */

DisplayControlStruct DisplayControl;

// Union to speed up the combination of low and high bytes to a word
// it is not optimal since the compiler still generates 2 unnecessary moves
// but using  -- value = (high << 8) | low -- gives 5 unnecessary instructions
union Myword {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } byte;
    uint16_t Word;
    uint8_t * BytePointer;
};

// if max value > DISPLAY_USAGE the ValueShift is incremented
#define DISPLAY_USAGE DISPLAY_HEIGHT
#define DISPLAY_VALUE_FOR_ZERO (DISPLAY_HEIGHT - 1)

/***********************
 *   Loop control
 ***********************/
// last sample of millis() in loop as reference for loop duration
uint32_t MillisLastLoop = 0;
uint32_t MillisSinceLastInfoOutput = 0;
#define MILLIS_BETWEEN_INFO_OUTPUT 1000

/***************
 * Debug stuff
 ***************/
#ifdef DEBUG
#define DEBUG_PIN PORTD7 // PIN 7
#define DEBUG_PORT PORTD
#define DEBUG_PORT_INPUT PIND
inline void toggleDebug(void) {
//    digitalToggleFast(DEBUG_PIN)
    DEBUG_PORT_INPUT = (1 << DEBUG_PIN);
}
inline void setDebug(void) {
    DEBUG_PORT |= (1 << DEBUG_PIN);
}
inline void resetDebug(void) {
    DEBUG_PORT &= ~(1 << DEBUG_PIN);
}

void printDebugData(void);

uint16_t DebugValue1;
uint16_t DebugValue2;
uint16_t DebugValue3;
uint16_t DebugValue4;
#endif

/***********************************************************************************
 * Put those variables at the end of data section (.data + .bss + .noinit)
 * adjacent to stack in order to have stack overflows running into DataBuffer array
 * *********************************************************************************/
/*
 * a string buffer for any purpose...
 */
char StringBuffer[50] __attribute__((section(".noinit")));
// safety net - array overflow will overwrite only DisplayBuffer

/*
 * Data buffer
 */
struct DataBufferStruct {
    uint8_t DisplayBuffer[DISPLAY_WIDTH];
    uint8_t * DataBufferNextInPointer;
    uint8_t * DataBufferNextDrawPointer; // pointer to DataBuffer - for draw-while-acquire mode
    uint16_t DataBufferNextDrawIndex; // index in DisplayBuffer - for draw-while-acquire mode
    // to detect end of acquisition in interrupt service routine
    uint8_t * DataBufferEndPointer;
    // Used to synchronize ISR with main loop
    bool DataBufferFull;
    // AcqusitionSize is DISPLAY_WIDTH except on last acquisition before stop then it is DATABUFFER_SIZE
    uint16_t AcquisitionSize;
    // Pointer for horizontal scrolling
    uint8_t * DataBufferDisplayStart;
    uint8_t DataBuffer[DATABUFFER_SIZE];
} DataBufferControl __attribute__((section(".noinit")));

/*******************************************************************************************
 * Function declaration section
 *******************************************************************************************/
// Measurement section
void startAcquisition(void);
void acquireDataFast(void);

// Measurement auto control stuff (trigger, range + offset)
void computeAutoTrigger(void);
void setLevelAndHysteresis(int aRawTriggerValue, int aRawTriggerHysteresis);
void setInputRange(uint8_t aShiftValue, uint8_t aActiveAttenuatorValue);
bool changeRange(int8_t aChangeAmount);
bool checkRAWValuesForClippingAndChangeRange(void);
void computeAutoRange(void);
void computeAutoOffset(void);
void setOffsetAutomatic(bool aNewState);

// Attenuator support stuff
void setAttenuator(uint8_t aNewValue);
void setACMode(bool aNewMode);
uint16_t getAttenuatorFactor(void);

// GUI initialization and drawing stuff
void createGUI(void);
void drawStartGui(void);
void drawDSOSettingsPageGui(void);
void activatePartOfGui(void);
void redrawDisplay(void);

// GUI handler section
void TouchUpHandler(struct XYPosition * const aTochPosition);
void longTouchDownHandler(struct XYPosition * const aTochPosition);
void swipeEndHandler(struct Swipe * const aSwipeInfo);

// BUTTON handler section
void doAcDc(uint8_t aTheTouchedButton, int16_t aValue);
void doBack(uint8_t aTheTouchedButton, int16_t aValue);
void doTriggerAutoManualFree(uint8_t aTheTouchedButton, int16_t aValue);
void doRangeMode(uint8_t aTheTouchedButton, int16_t aValue);
void doTriggerSlope(uint8_t aTheTouchedButton, int16_t aValue);
void doAutoOffsetOnOff(uint8_t aTheTouchedButton, int16_t aValue);
void doADCReference(uint8_t aTheTouchedButton, int16_t aValue);
void doChannelSelect(uint8_t aTheTouchedButton, int16_t aValue);
void doTriggerSingleshot(uint8_t aTheTouchedButton, int16_t aValue);
void doStartStop(uint8_t aTheTouchedButton, int16_t aValue);
void doChartHistory(uint8_t aTheTouchedButton, int16_t aValue);
void doTriggerLevel(uint8_t aTheTouchedSlider, const int16_t aValue);
void doVoltagePicker(uint8_t aTheTouchedSlider, const int16_t aValue);
void doShowSettingsPage(uint8_t aTheTouchedButton, int16_t aValue);

// Button caption section
void setChannelButtonsCaption(void);
void setReferenceButtonCaption(void);
void setACModeButtonCaption(void);
void setTriggerAutoOnOffButtonCaption(void);
void setAutoOffsetButtonCaption(void);
void setAutoRangetButtonCaption(void);
void setSlopeButtonCaption(void);

// Graphical output section
bool scrollChart(int8_t aScrollAmount);
void clearTriggerLine(uint8_t aTriggerLevelDisplayValue);
void drawTriggerLine(void);
void drawGridLinesWithHorizLabelsAndTriggerLine(void);
void clearHorizontalGridLinesAndHorizontalLineLabels(void);
void drawMinMaxLines(void);
void drawDataBuffer(uint8_t *aByteBuffer, uint16_t aColor, uint16_t aClearBeforeColor);
void clearDisplayedChart(uint8_t * aDisplayBufferPtr);
void drawRemainingDataBufferValues(void);

// Text output section
void printVCCAndTemperature(void);
void clearInfo(void);
void printInfo(void);

// Utility section
bool changeTimeBaseValue(int8_t aChangeValue, bool doOutput);
void computeMicrosPerPeriod(void);
uint8_t getDisplayFromRawValue(uint16_t aRawValue);
uint16_t getRawFromDisplayValue(uint8_t aDisplayValue);
float getFloatFromDisplayValue(uint8_t aDisplayValue);

//Hardware support section
float getTemperature(void);
void setVCCValue(void);
void setChannel(uint8_t aChannel);
void setPrescaleFactor(uint8_t aFactor);
void setReference(uint8_t aReference);
void initTimer2(void);

/*******************************************************************************************
 * Program code starts here
 * Setup section
 *******************************************************************************************/

void initDisplay(void) {
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_LONG_TOUCH_ENABLE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0xD1, 0x21E7); // Ascending in UTF16
    BlueDisplay1.setCharacterMapping(0xD2, 0x21E9); // Descending in UTF16
//    BlueDisplay1.setCharacterMapping(0xF8, 0x2103); // Degree Celsius in UTF16
    BlueDisplay1.setButtonsTouchTone(TONE_PROP_BEEP, 80);
    createGUI();
    initFrequencyPage();
}

void setup() {
//    pinMode(ATTENUATOR_0_PIN, OUTPUT);
//    pinMode(ATTENUATOR_1_PIN, OUTPUT);
//    pinMode(AC_DC_PIN, OUTPUT);
    DDRD = (DDRD & ~CONTROL_MASK) | CONTROL_MASK;
//    pinMode(TIMER_1_OUTPUT_PIN, OUTPUT);
//    pinMode(VEE_PIN, OUTPUT);
    DDRB = (DDRD & ~OUTPUT_MASK_PORTB) | OUTPUT_MASK_PORTB;
    pinMode(ATTENUATOR_DETECT_PIN_0, INPUT_PULLUP);
    pinMode(ATTENUATOR_DETECT_PIN_1, INPUT_PULLUP);
#ifdef DEBUG
    pinMode(DEBUG_PIN, OUTPUT);
#endif

    // Shutdown SPI and TWI, enable all timers, USART and ADC
    PRR = (1 << PRSPI) | (1 << PRTWI);
    // Disable  digital input on all ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D | ADC4D | ADC5D;

    initSimpleSerial(HC_05_BAUD_RATE, false);

    // initialize values
    MeasurementControl.isRunning = false;
    MeasurementControl.TriggerSlopeRising = true;
    MeasurementControl.TriggerMode = TRIGGER_MODE_AUTO;
    MeasurementControl.isSingleShotMode = false;

    MeasurementControl.TimebaseIndex = 0;
    changeTimeBaseValue(TIMEBASE_INDEX_START_VALUE, false);

    MeasurementControl.RawDSOReadingACZero = 0x200;
    /*
     * setChannel() sets:
     * ShiftValue
     * ADCInputMUXChannel
     * ADCInputMUXChannelChar
     *
     * And if AttenuatorDetected == true also:
     * ChannelHasAttenuator
     * isACModeANDChannelHasAttenuator
     * AttenuatorValue
     * ADCReference
     */
    MeasurementControl.ChannelHasActiveAttenuator = false;
    MeasurementControl.AttenuatorValue = 0; // set direct input as default
    uint8_t tAttenuatorType = !digitalReadFast(ATTENUATOR_DETECT_PIN_0);
    tAttenuatorType |= (!digitalReadFast(ATTENUATOR_DETECT_PIN_1)) << 1;
    MeasurementControl.AttenuatorType = tAttenuatorType;
    uint8_t tChannel = 0;
    if (tAttenuatorType == ATTENUATOR_TYPE_SIMPLE_ATTENUATOR) {
        tChannel = 1;
    } else if (tAttenuatorType >=  ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        initTimer2(); // start timer2 for generating VEE (negative Voltage for external hardware)
    }
    setChannel(tChannel);
    /*
     * setChannel calls setInputRange(2,2) and this sets:
     * OffsetValue
     * HorizontalGridSizeShift8
     * HorizontalGridVoltage
     */

    MeasurementControl.RangeAutomatic = true;
    MeasurementControl.OffsetAutomatic = false;

    DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];

    DisplayControl.EraseColor = COLOR_BACKGROUND_DSO;
    DisplayControl.showHistory = false;
    DisplayControl.DisplayPage = DISPLAY_PAGE_START;
    DisplayControl.showInfoMode = INFO_MODE_SHORT_INFO;

    setACMode(false);
    initFrequency();
    initDisplay();
    drawStartGui();
    registerSimpleConnectCallback(&initDisplay);
    registerSimpleResizeAndReconnectCallback(&redrawDisplay);
    registerSwipeEndCallback(&swipeEndHandler);
    registerTouchUpCallback(&TouchUpHandler);
    registerLongTouchDownCallback(&longTouchDownHandler, 900);

    FeedbackTone(true);
    delay(400);
    setVCCValue();
    FeedbackTone(true);


}

/************************************************************************
 * main loop - 32 microseconds
 ************************************************************************/
// noreturn saves program space!
void __attribute__((noreturn)) loop(void) {
    uint32_t tMillis, tMillisOfLoop;
    bool tOutputInfo = true;

    for (;;) {
        tMillis = millis();
        tMillisOfLoop = tMillis - MillisLastLoop;
        MillisLastLoop = tMillis;
        MillisSinceLastInfoOutput += tMillisOfLoop;
        if (MillisSinceLastInfoOutput > MILLIS_BETWEEN_INFO_OUTPUT) {
            MillisSinceLastInfoOutput = 0;
            tOutputInfo = true;
        }

        if (MeasurementControl.isRunning) {
            if (MeasurementControl.TimebaseFastFreerunningMode) {
                /*
                 * Fast mode here  <= 201us/div
                 */
                acquireDataFast();
            }
            if (DataBufferControl.DataBufferFull) {
                /*
                 * Data (from InterruptServiceRoutine) is ready
                 */
                if (tOutputInfo) {
                    tOutputInfo = false;
                    // refresh grid
                    drawGridLinesWithHorizLabelsAndTriggerLine();
                    if (DisplayControl.showInfoMode != INFO_MODE_NO_INFO) {
                        computeMicrosPerPeriod();
                        printInfo();
                    }
                }

                MeasurementControl.ValueAverage = (MeasurementControl.IntegrateValueForAverage
                        + (DataBufferControl.AcquisitionSize / 2)) / DataBufferControl.AcquisitionSize;
                if (MeasurementControl.StopRequested) {
                    MeasurementControl.StopRequested = false;
                    MeasurementControl.isRunning = false;
                    if (MeasurementControl.ADCReference != DEFAULT) {
                        // get new VCC Value
                        setVCCValue();
                    }
                    if (MeasurementControl.isSingleShotMode) {
                        MeasurementControl.isSingleShotMode = false;
                        // Clear single shot character
                        BlueDisplay1.drawChar(INFO_LEFT_MARGIN + SINGLESHOT_PPRINT_VALUE_X, INFO_UPPER_MARGIN + TEXT_SIZE_11_HEIGHT,
                                ' ', TEXT_SIZE_11, COLOR_BLACK, COLOR_BACKGROUND_DSO);
                    }
                    //DisplayControl.showInfoMode = INFO_MODE_SHORT_INFO;
                    redrawDisplay();
                } else {
                    /*
                     * Normal loop-> process data, draw new chart, and start next acquisition
                     */
                    uint8_t tLastTriggerDisplayValue = DisplayControl.TriggerLevelDisplayValue;
                    computeAutoTrigger();
                    computeAutoRange();
                    computeAutoOffset();
                    // handle trigger line
                    DisplayControl.TriggerLevelDisplayValue = getDisplayFromRawValue(MeasurementControl.RawTriggerLevel);
                    if (tLastTriggerDisplayValue != DisplayControl.TriggerLevelDisplayValue) {
                        clearTriggerLine(tLastTriggerDisplayValue);
                        drawTriggerLine();
                    }

                    if (!DisplayControl.DrawWhileAcquire) {
                        // normal mode => clear old chart and draw new data
                        drawDataBuffer(&DataBufferControl.DataBuffer[0], COLOR_DATA_RUN, DisplayControl.EraseColor);
                    }
                    startAcquisition();
                }
            } else {
                /*
                 * Acquisition still ongoing
                 */
                if (DisplayControl.DrawWhileAcquire) {
                    drawRemainingDataBufferValues();
                    MeasurementControl.RawValueMin = MeasurementControl.ValueMinForISR;
                    MeasurementControl.RawValueMax = MeasurementControl.ValueMaxForISR;
                    if (checkRAWValuesForClippingAndChangeRange()) {
                        // range changed set new values for ISR to avoid another range change by the old values
                        MeasurementControl.ValueMinForISR = 42;
                        MeasurementControl.ValueMaxForISR = 42;
                    }
                }
            }
#ifdef DEBUG
//            DebugValue1 = MeasurementControl.ShiftValue;
//            DebugValue2 = MeasurementControl.RawValueMin;
//            DebugValue3 = MeasurementControl.RawValueMax;
//            printDebugData();
#endif
            if (MeasurementControl.isSingleShotMode && MeasurementControl.searchForTrigger && tOutputInfo) {
                tOutputInfo = false;
                // output actual values for single shot every second
                MeasurementControl.ValueAverage = MeasurementControl.ValueBeforeTrigger;
            }
        }
        if (tOutputInfo && DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            tOutputInfo = false;
            printVCCAndTemperature();
        }
        /*
         * GUI handling
         */
        checkAndHandleEvents();
    } // for(;;)
}
/* Main loop end */

/************************************************************************
 * Measurement section
 ************************************************************************/
/*
 * prepares all variables for new acquisition
 * switches between fast an interrupt mode depending on TIMEBASE_FAST_MODES
 * sets ADC status register including prescaler
 */
void startAcquisition(void) {
    DataBufferControl.AcquisitionSize = DISPLAY_WIDTH;
    DataBufferControl.DataBufferEndPointer = &DataBufferControl.DataBuffer[DISPLAY_WIDTH - 1];
    if (MeasurementControl.isSingleShotMode) {
        MeasurementControl.StopRequested = true;
    }
    if (MeasurementControl.StopRequested) {
        DataBufferControl.AcquisitionSize = DATABUFFER_SIZE;
        DataBufferControl.DataBufferEndPointer = &DataBufferControl.DataBuffer[DATABUFFER_SIZE - 1];
    }
    /*
     * setup new interrupt cycle only if not to be stopped
     */
    DataBufferControl.DataBufferNextInPointer = &DataBufferControl.DataBuffer[0];
    DataBufferControl.DataBufferNextDrawPointer = &DataBufferControl.DataBuffer[0];
    DataBufferControl.DataBufferNextDrawIndex = 0;
// start with waiting for triggering condition
    MeasurementControl.IntegrateValueForAverage = 0;
    MeasurementControl.TriggerSampleCount = 0;
    MeasurementControl.TriggerStatus = TRIGGER_START;
    MeasurementControl.searchForTrigger = true;
    DataBufferControl.DataBufferFull = false;
//  MeasurementControl.IntegrateCount = 0;

    uint8_t tTimebaseIndex = MeasurementControl.TimebaseIndex;
    if (tTimebaseIndex <= TIMEBASE_INDEX_FAST_MODES) {
        MeasurementControl.TimebaseFastFreerunningMode = true;
    } else {
        MeasurementControl.TimebaseFastFreerunningMode = false;
    }

    /*
     * get hardware prescale value
     */
    uint8_t tTimebaseHWValue = PRESCALE_MAX_VALUE;
    if (tTimebaseIndex < TIMEBASE_NUMBER_OF_FAST_PRESCALE) {
        tTimebaseHWValue = PrescaleValueforTimebase[tTimebaseIndex];
    }

#ifdef TIMING_DEBUG
//   resetTimingDebug();
#endif
    /*
     * Start acquisition in free running mode for trigger detection
     */
    //  ADCSRB = 0; // free running mode  - is default
    if (MeasurementControl.TimebaseFastFreerunningMode) {
        // NO Interrupt in FastMode
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | tTimebaseHWValue);
    } else {
        //  enable ADC interrupt, start with free running mode,
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | tTimebaseHWValue | (1 << ADIE));
    }
}

/*
 * Fast ADC read routine for timebase <= 201us/div
 */
void acquireDataFast(void) {
    /**********************************
     * wait for triggering condition
     **********************************/
    Myword tUValue;
    uint8_t tTriggerStatus = TRIGGER_START;
    uint16_t i;
    uint16_t tValueOffset = MeasurementControl.OffsetValue;

    /*
     * Wait for trigger for max. 10 screens
     */
// start the first conversion and clear bit to recognize next conversion has finished
    ADCSRA |= (1 << ADIF) | (1 << ADSC);
    // if trigger condition not met it should run forever in single shot mode
    for (i = TRIGGER_WAIT_NUMBER_OF_SAMPLES; i != 0 || MeasurementControl.isSingleShotMode; --i) {
        // wait for free running conversion to finish
        while (bit_is_clear(ADCSRA, ADIF)) {
            ;
        }
        // Get value
        tUValue.byte.LowByte = ADCL;
        tUValue.byte.HighByte = ADCH;
        // without "| (1 << ADSC)" it does not work - undocumented feature???
        ADCSRA |= (1 << ADIF) | (1 << ADSC); // clear bit to recognize next conversion has finished

        /*
         * detect trigger slope
         */
        if (MeasurementControl.TriggerSlopeRising) {
            if (tTriggerStatus == TRIGGER_START) {
                // rising slope - wait for value below 1. threshold
                if (tUValue.Word < MeasurementControl.TriggerLevelLower) {
                    tTriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // rising slope - wait for value to rise above 2. threshold
                if (tUValue.Word > MeasurementControl.TriggerLevelUpper) {
                    break;
                }
            }
        } else {
            if (tTriggerStatus == TRIGGER_START) {
                // falling slope - wait for value above 1. threshold
                if (tUValue.Word > MeasurementControl.TriggerLevelUpper) {
                    tTriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // falling slope - wait for value to go below 2. threshold
                if (tUValue.Word < MeasurementControl.TriggerLevelLower) {
                    break;
                }
            }
        }
    }
    MeasurementControl.searchForTrigger = false; // for single shot mode

    /********************************
     * read a buffer of data
     ********************************/
// setup for min, max, average
    uint16_t tValueMax = tUValue.Word;
    uint16_t tValueMin = tUValue.Word;
    uint8_t tIndex = MeasurementControl.TimebaseIndex;
    uint16_t tLoopCount = DataBufferControl.AcquisitionSize;
    uint8_t *DataPointer = &DataBufferControl.DataBuffer[0];
    uint8_t *DataPointerFast = &DataBufferControl.DataBuffer[0];
    uint32_t tIntegrateValue = 0;

    cli();
    if (tIndex <= TIMEBASE_INDEX_ULTRAFAST_MODES) {
        if (DATABUFFER_SIZE / 2 < DISPLAY_WIDTH) {
            // No space for 2 times DISPLAY_WIDTH values
            // I hope the compiler will remove the code ;-)
            tLoopCount = DATABUFFER_SIZE / 2;
        } else if (tLoopCount > DISPLAY_WIDTH) {
            tLoopCount = tLoopCount / 2;
        }
        /*
         * do very fast reading without processing
         */
        for (i = tLoopCount; i != 0; --i) {
            uint8_t tLow, tHigh;
            while (bit_is_clear(ADCSRA, ADIF)) {
                ;
            }
            tLow = ADCL;
            tHigh = ADCH;
            ADCSRA |= (1 << ADIF) | (1 << ADSC);
            *DataPointerFast++ = tLow;
            *DataPointerFast++ = tHigh;
        }
        sei();
        DataPointerFast = &DataBufferControl.DataBuffer[0];
    }
    for (i = tLoopCount; i != 0; --i) {
        if (tIndex <= TIMEBASE_INDEX_ULTRAFAST_MODES) {
            // get values from ultrafast buffer
            tUValue.byte.LowByte = *DataPointerFast++;
            tUValue.byte.HighByte = *DataPointerFast++;
        } else {
            // get values from adc
            // wait for free running conversion to finish
            // ADCSRA here is E5
            while (bit_is_clear(ADCSRA, ADIF)) {
                ;
            }
            // ADCSRA here is F5
            // duration: get Value included min 1,2 micros
            tUValue.byte.LowByte = ADCL;
            tUValue.byte.HighByte = ADCH;
            // without "| (1 << ADSC)" it does not work - undocumented feature???
            ADCSRA |= (1 << ADIF) | (1 << ADSC); // clear bit to recognize next conversion has finished
            //ADCSRA here is E5
        }
        /*
         * process value
         */

        tIntegrateValue += tUValue.Word;
        // get max and min for display and automatic triggering - needs 0,4 microseconds
        if (tUValue.Word > tValueMax) {
            tValueMax = tUValue.Word;
        } else if (tUValue.Word < tValueMin) {
            tValueMin = tUValue.Word;
        }

        /***************************************************
         * transform 10 bit value in order to fit on screen
         ***************************************************/
        if (tUValue.Word < tValueOffset) {
            tUValue.Word = 0;
        } else {
            tUValue.Word -= tValueOffset;
        }
        tUValue.Word = tUValue.Word >> MeasurementControl.ShiftValue;
        // Byte overflow? This can happen if autorange is disabled.
        if (tUValue.byte.HighByte > 0) {
            tUValue.byte.LowByte = 0xFF;
        }
        // now value is a byte and fits to screen
        *DataPointer++ = DISPLAY_VALUE_FOR_ZERO - tUValue.byte.LowByte;
    }
// enable interrupt
    sei();
    if (tIndex == 0) {
        // set remaining of buffer to zero
        for (i = tLoopCount; i != 0; --i) {
            *DataPointer++ = 0;
        }
    }
    ADCSRA &= ~(1 << ADATE); // Disable auto-triggering
    MeasurementControl.RawValueMax = tValueMax;
    MeasurementControl.RawValueMin = tValueMin;
    if (tIndex == 0 && tLoopCount > DISPLAY_WIDTH) {
        // compensate for half sample count in last measurement in fast mode
        tIntegrateValue *= 2;
    }
    MeasurementControl.IntegrateValueForAverage = tIntegrateValue;
    DataBufferControl.DataBufferFull = true;
}

/*
 * Interrupt service routine for adc interrupt
 * used only for "slow" mode because ISR overhead is to much for fast mode
 * app. 7 microseconds + 2 for push + 2 for pop
 */

ISR(ADC_vect) {
    if (MeasurementControl.TimebaseDelay == 0) {
        // 3 + 8 Pushes + in + eor =24 cycles
        // + 4+ cycles for jump to isr
        // +  4 for load compare and branch
        // gives 32 cycles = 2 micros
        ADCSRA |= (1 << ADSC);
    }

    Myword tUValue;
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;

    if (MeasurementControl.searchForTrigger) {
        bool tTriggerFound = false;
        /*
         * Trigger detection here
         */
        uint8_t tTriggerStatus = MeasurementControl.TriggerStatus;

        if (MeasurementControl.TriggerSlopeRising) {
            if (tTriggerStatus == TRIGGER_START) {
                // rising slope - wait for value below 1. threshold
                if (tUValue.Word < MeasurementControl.TriggerLevelLower) {
                    MeasurementControl.TriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // rising slope - wait for value to rise above 2. threshold
                if (tUValue.Word > MeasurementControl.TriggerLevelUpper) {
                    // start reading into buffer
                    tTriggerFound = true;
                }
            }
        } else {
            if (tTriggerStatus == TRIGGER_START) {
                // falling slope - wait for value above 1. threshold
                if (tUValue.Word > MeasurementControl.TriggerLevelUpper) {
                    MeasurementControl.TriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // falling slope - wait for value to go below 2. threshold
                if (tUValue.Word < MeasurementControl.TriggerLevelLower) {
                    // start reading into buffer
                    tTriggerFound = true;
                }
            }
        }

        if (!tTriggerFound) {
            // two consecutive if's save one register push and pop
            if (MeasurementControl.isSingleShotMode) {
                // no timeout for SingleShotMode -> return
                MeasurementControl.ValueBeforeTrigger = tUValue.Word;
                return;
            }
            /*
             * Trigger timeout handling
             */
            MeasurementControl.TriggerSampleCount++;
            if (MeasurementControl.TriggerSampleCount < MeasurementControl.TriggerTimeoutSampleCount) {
                /*
                 * Trigger condition not met and timeout not reached
                 */
                return;
            }
        }
        // Trigger found -> reset trigger flag and initialize max and min
        MeasurementControl.searchForTrigger = false;
        MeasurementControl.ValueMaxForISR = tUValue.Word;
        MeasurementControl.ValueMinForISR = tUValue.Word;
        // stop free running mode
        ADCSRA &= ~(1 << ADATE);

    }
    /*
     * read buffer data
     */
    // take only last value for chart
    bool tUseValue = true;
    if (MeasurementControl.TimebaseDelay != 0) {
        uint16_t tRemaining = MeasurementControl.TimebaseDelayRemaining;
        if (tRemaining > (ADC_CONVERSION_AS_DELAY_MICROS * 4)) {
            // instead of busy wait just start a new conversion, which lasts 112 micros ( 13 + 1 (delay) clock cycles at 8 micros per cycle )
            MeasurementControl.TimebaseDelayRemaining -= (ADC_CONVERSION_AS_DELAY_MICROS * 4);
            tUseValue = false;
        } else {
            // busy wait for remaining micros
            // delayMicroseconds() needs 2 registers more (=1/4 microsecond call overhead)
            // here we have 1/4 microseconds resolution
            __asm__ __volatile__ (
                    "1: sbiw %0,1" "\n\t" // 2 cycles
                    "brne 1b" : : "r" (MeasurementControl.TimebaseDelayRemaining)// 2 cycles
            );
            // restore initial value
            MeasurementControl.TimebaseDelayRemaining = MeasurementControl.TimebaseDelay;
        }
        // 5 micros from Interrupt till here
        // start the next conversion since in this mode adc is not free running
        ADCSRA |= (1 << ADSC);
    }

    /*
     * do further processing of raw data
     */
    // get max and min for display and automatic triggering
    if (tUValue.Word > MeasurementControl.ValueMaxForISR) {
        MeasurementControl.ValueMaxForISR = tUValue.Word;
    } else if (tUValue.Word < MeasurementControl.ValueMinForISR) {
        MeasurementControl.ValueMinForISR = tUValue.Word;
    }

    if (tUseValue) {
// detect end of buffer
        uint8_t * tDataBufferPointer = DataBufferControl.DataBufferNextInPointer;
        if (tDataBufferPointer > DataBufferControl.DataBufferEndPointer) {
            // stop acquisition
            ADCSRA &= ~(1 << ADIE); // disable ADC interrupt
            // copy max and min values of measurement for display purposes
            MeasurementControl.RawValueMin = MeasurementControl.ValueMinForISR;
            MeasurementControl.RawValueMax = MeasurementControl.ValueMaxForISR;
            /*
             * signal to main loop that acquisition ended
             * Main loop is responsible to start a new acquisition via call of startAcquisition();
             */
            DataBufferControl.DataBufferFull = true;
        } else {
            /*
             * c code (see line below) needs 5 register more (to push and pop)#
             * so do it with assembler and 1 additional register (for high byte :-()
             */
            //  MeasurementControl.IntegrateValueForAverage += tUValue.Word;
            __asm__ (
                    /* could use __tmp_reg__ as synonym for r24 */
                    /* add low byte */
                    "ld     r24, Z  ; \n"
                    "add    r24, %[lowbyte] ; \n"
                    "st     Z+, r24 ; \n"
                    /* add high byte with carry */
                    "ld     r24, Z  ; \n"
                    "adc    r24, %[highbyte] ; \n"
                    "st     Z+, r24 ; \n"
                    /* add carry */
                    "ld     r24, Z  ; \n"
                    "adc    r24, __zero_reg__ ; \n"
                    "st     Z+, r24 ; \n"
                    /* add carry */
                    "ld     r24, Z  ; \n"
                    "adc    r24, __zero_reg__ ; \n"
                    "st     Z+, r24 ; \n"
                    :/*no output*/
                    : [lowbyte] "r" (tUValue.byte.LowByte), [highbyte] "r" (tUValue.byte.HighByte), "z" (&MeasurementControl.IntegrateValueForAverage)
                    : "r24"
            );
            /***************************************************
             * transform 10 bit value in order to fit on screen
             ***************************************************/
            if (tUValue.Word < MeasurementControl.OffsetValue) {
                tUValue.Word = 0;
            } else {
                tUValue.Word = tUValue.Word - MeasurementControl.OffsetValue;
            }
            tUValue.Word = tUValue.Word >> MeasurementControl.ShiftValue;
            // Byte overflow? This can happen if autorange is disabled.
            if (tUValue.byte.HighByte > 0) {
                tUValue.byte.LowByte = 0xFF;
            }
            // store display byte value
            *tDataBufferPointer++ = DISPLAY_VALUE_FOR_ZERO - tUValue.byte.LowByte;
            // prepare for next
            DataBufferControl.DataBufferNextInPointer = tDataBufferPointer;
        }
    }
}

/***********************************************************************
 * Measurement auto control stuff (trigger, range + offset)
 ***********************************************************************/
/*
 * sets hysteresis, shift and offset such that graph are from bottom to DISPLAY_USAGE
 * If old values are reasonable don't change them to avoid jitter
 */
void computeAutoTrigger(void) {
    if (!MeasurementControl.TriggerMode == TRIGGER_MODE_AUTO) {
        return;
    }
    uint16_t tPeakToPeak = MeasurementControl.RawValueMax - MeasurementControl.RawValueMin;
// middle between min and max
    uint16_t tTriggerValue = MeasurementControl.RawValueMin + (tPeakToPeak / 2);

    /*
     * set effective hysteresis to quarter delta and clip to reasonable value
     */
    uint8_t TriggerHysteresis = tPeakToPeak / 4;
    if (TriggerHysteresis < TRIGGER_HYSTERESIS_MIN) {
        TriggerHysteresis = TRIGGER_HYSTERESIS_MIN;
    }
    setLevelAndHysteresis(tTriggerValue, TriggerHysteresis);
}

void setLevelAndHysteresis(int aRawTriggerValue, int aRawTriggerHysteresis) {
    MeasurementControl.RawTriggerLevel = aRawTriggerValue;
    if (MeasurementControl.TriggerSlopeRising) {
        MeasurementControl.TriggerLevelUpper = aRawTriggerValue;
        MeasurementControl.TriggerLevelLower = aRawTriggerValue - aRawTriggerHysteresis;
    } else {
        MeasurementControl.TriggerLevelLower = aRawTriggerValue;
        MeasurementControl.TriggerLevelUpper = aRawTriggerValue + aRawTriggerHysteresis;
    }
}

/*
 * Clears and sets grid and label too.
 * sets:
 * OffsetValue
 * HorizontalGridSizeShift8
 * HorizontalGridVoltage
 */
void setInputRange(uint8_t aShiftValue, uint8_t aActiveAttenuatorValue) {
    MeasurementControl.ShiftValue = aShiftValue;
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        setAttenuator(aActiveAttenuatorValue);
    }
    if (MeasurementControl.isACMode) {
        // Adjust zero offset for small display ranges
        uint16_t tNewValueOffsetForACMode = MeasurementControl.RawDSOReadingACZero / 2 + MeasurementControl.RawDSOReadingACZero / 4;
        if (aShiftValue == 1) {
            tNewValueOffsetForACMode = MeasurementControl.RawDSOReadingACZero / 2;
        } else if (aShiftValue == 2) {
            tNewValueOffsetForACMode = 0;
        }
        MeasurementControl.OffsetValue = tNewValueOffsetForACMode;
    } else if (!MeasurementControl.OffsetAutomatic) {
        MeasurementControl.OffsetValue = 0;
    }

    if (MeasurementControl.isRunning) {
        clearHorizontalGridLinesAndHorizontalLineLabels();
    }
    float tNewGridVoltage;
    uint8_t tFactor = 1 << aShiftValue;
    if (MeasurementControl.ADCReference == DEFAULT) {
        /*
         * 5 Volt reference
         */
        if (aShiftValue == 0) {
            // formula for 5V and 0.2V / div and shift 0 is: ((1023/5V)*0.2) * 256/2^shift = 52377.6 / 5
            MeasurementControl.HorizontalGridSizeShift8 = (52377.6 / MeasurementControl.VCC);
            tNewGridVoltage = 0.2;
        } else {
            // formula for 5V and 0.5V / div and shift 1 is: ((1023/5V)*0.5) * 256/2^shift = (130944/2) / 5
            MeasurementControl.HorizontalGridSizeShift8 = (65472.0 / MeasurementControl.VCC);
            tNewGridVoltage = 0.5 * aShiftValue;
        }
    } else {
        /*
         * 1.1 Volt reference
         */
        if (MeasurementControl.ChannelHasActiveAttenuator) {
            MeasurementControl.HorizontalGridSizeShift8 = HORIZONTAL_GRID_HEIGHT_2V_SHIFT8;
        } else {
            MeasurementControl.HorizontalGridSizeShift8 = HORIZONTAL_GRID_HEIGHT_1_1V_SHIFT8;
        }
        tNewGridVoltage = 0.05 * tFactor;
    }
    MeasurementControl.HorizontalGridVoltage = tNewGridVoltage * getAttenuatorFactor();
    if (MeasurementControl.isRunning) {
        drawGridLinesWithHorizLabelsAndTriggerLine();
    }
}

/*
 * used by swipe handler
 * input Range is ShiftValue + 3 * AttenuatorValue
 */
bool changeRange(int8_t aChangeAmount) {
    bool tRetValue = true;
    int8_t tNewValue = 0;
    if (MeasurementControl.AttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        tNewValue = MeasurementControl.AttenuatorValue * 3;
    }
    tNewValue += MeasurementControl.ShiftValue + aChangeAmount;
    if (tNewValue < 0) {
        tNewValue = 0;
        tRetValue = false;
    }
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        if (tNewValue > 8) {
            tNewValue = 8;
            tRetValue = false;
        }
    } else {
        if (tNewValue > 2) {
            tNewValue = 2;
            tRetValue = false;
        }
    }
    setInputRange(tNewValue % 3, tNewValue / 3);
    return tRetValue;
}

/*
 * makes only sense if external attenuator attached
 * returns true, if range was changed
 */
bool checkRAWValuesForClippingAndChangeRange(void) {
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        if (MeasurementControl.RangeAutomatic) {
            // Check for clipping (check ADC_MAX_CONVERSION_VALUE and also 0 if AC mode)
            if ((MeasurementControl.RawValueMax == ADC_MAX_CONVERSION_VALUE
                    || (MeasurementControl.isACMode && MeasurementControl.RawValueMin == 0))
                    && MeasurementControl.AttenuatorValue < 2) {
                setInputRange(0, MeasurementControl.AttenuatorValue + 1);
                return true;
            }
        }
    }
    return false;
}

void computeAutoRange(void) {
    if (MeasurementControl.RangeAutomatic) {
        //First check for clipping - check ADC_MAX_CONVERSION_VALUE and also zero if AC mode
        if (checkRAWValuesForClippingAndChangeRange()) {
            return;
        }

        // get relevant peak2peak value
        int16_t tPeakToPeak = MeasurementControl.RawValueMax;
        if (MeasurementControl.isACMode) {
            tPeakToPeak -= MeasurementControl.RawDSOReadingACZero;
            // check for zero offset error with tPeakToPeak < 0
            if (tPeakToPeak < 0
                    || ((int16_t) MeasurementControl.RawDSOReadingACZero - (int16_t) MeasurementControl.RawValueMin)
                            > tPeakToPeak) {
                //difference between zero and min is greater than difference between zero and max => min determines the range
                tPeakToPeak = MeasurementControl.RawDSOReadingACZero - MeasurementControl.RawValueMin;
            }
            // since tPeakToPeak must fit in half of display
            tPeakToPeak *= 2;
        } else if (MeasurementControl.OffsetAutomatic) {
            // Value min(display) is NOT fixed at zero
            tPeakToPeak -= MeasurementControl.RawValueMin;
        }

        /*
         * set automatic range
         */
        uint8_t tOldValueShift = MeasurementControl.ShiftValue;
        uint8_t tNewValueShift = 0;
        uint8_t tNewAttenuatorValue = MeasurementControl.AttenuatorValue;

        bool tAttChanged = false; // no change
        if (tPeakToPeak >= DISPLAY_USAGE * 2) {
            tNewValueShift = 2;
        } else if (tPeakToPeak >= DISPLAY_USAGE) {
            tNewValueShift = 1;
        } else if (MeasurementControl.AttenuatorValue > 0 && MeasurementControl.AttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
            /*
             * only max value is relevant for attenuator switching!
             */
            if (MeasurementControl.RawValueMax < ((DISPLAY_USAGE - (DISPLAY_USAGE / 10)) * 4) / ATTENUATOR_FACTOR) {
                // more than 10 percent below theoretical threshold -> switch attenuator to higher resolution
                tNewAttenuatorValue--;
                tNewValueShift = 2;
                tAttChanged = true;
            }
        }

        if (tNewValueShift < tOldValueShift || tAttChanged) {
            /*
             * wait n-milliseconds before switch to higher resolution (lower index)
             */
            uint32_t tActualMillis = millis();
            if (tActualMillis - MeasurementControl.TimestampLastRangeChange > SCALE_CHANGE_DELAY_MILLIS) {
                MeasurementControl.TimestampLastRangeChange = tActualMillis;
                setInputRange(tNewValueShift, tNewAttenuatorValue);
            }
        } else if (tNewValueShift > tOldValueShift) {
            setInputRange(tNewValueShift, tNewAttenuatorValue);
        } else {
            // reset "delay"
            MeasurementControl.TimestampLastRangeChange = millis();
        }
    }
}

/**
 * compute offset based on center value in order display values at center of screen
 */
void computeAutoOffset(void) {
    if (MeasurementControl.OffsetAutomatic) {
        uint16_t tValueMiddleY = MeasurementControl.RawValueMin
                + ((MeasurementControl.RawValueMax - MeasurementControl.RawValueMin) / 2);

        uint16_t tRawValuePerGrid = MeasurementControl.HorizontalGridSizeShift8 >> (8 - MeasurementControl.ShiftValue);
        uint16_t tLinesPerHalfDisplay = 4;
        if (MeasurementControl.HorizontalGridSizeShift8 > 10000) {
            tLinesPerHalfDisplay = 2;
        }
// take next multiple of HorizontalGridSize which is smaller than tValueMiddleY
        int16_t tNumberOfGridLinesToSkip = (tValueMiddleY / tRawValuePerGrid) - tLinesPerHalfDisplay; // adjust to bottom of display (minus 3 lines)
        if (tNumberOfGridLinesToSkip < 0) {
            tNumberOfGridLinesToSkip = 0;
        }
// avoid jitter by not changing number if its delta is only 1
        if (abs(MeasurementControl.OffsetGridCount - tNumberOfGridLinesToSkip) > 1 || tNumberOfGridLinesToSkip == 0) {
            MeasurementControl.OffsetValue = tNumberOfGridLinesToSkip * tRawValuePerGrid;
            MeasurementControl.OffsetGridCount = tNumberOfGridLinesToSkip;
        }
    }
}

/*
 * returns false if auto offset could not be enabled because of ac mode
 */
void setOffsetAutomatic(bool aNewState) {
    if (!aNewState) {
// disable auto offset
        MeasurementControl.OffsetValue = 0;
    }
    MeasurementControl.OffsetAutomatic = aNewState;
    setAutoOffsetButtonCaption();
}

/***********************************************************************
 * Attenuator support stuff
 ***********************************************************************/

void setAttenuator(uint8_t aNewValue) {
    MeasurementControl.AttenuatorValue = aNewValue;
    uint8_t tPortValue = CONTROL_PORT;
    tPortValue &= ~ATTENUATOR_MASK;
    tPortValue |= ((aNewValue << 2) & ATTENUATOR_MASK);
    CONTROL_PORT = tPortValue;
}

/*
 * No AC mode for channels without attenuator
 */
void setACMode(bool aNewMode) {
    if (MeasurementControl.isRunning) {
        clearHorizontalGridLinesAndHorizontalLineLabels();
    }
    MeasurementControl.storeForACMode = aNewMode;
    MeasurementControl.isACMode = aNewMode;
    digitalWriteFast(AC_DC_PIN, aNewMode);
    uint8_t tRelaisPin;
    if (aNewMode) {
        // no OffsetAutomatic for AC mode
        setOffsetAutomatic(false);
        tRelaisPin = AC_DC_RELAIS_PIN_1;
    } else {
        tRelaisPin = AC_DC_RELAIS_PIN_2;
    }
    // power latching relay
    digitalWriteFast(tRelaisPin, HIGH);
    setACModeButtonCaption();
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        // hide/show offset
        drawDSOSettingsPageGui();
    }
    if (MeasurementControl.isRunning) {
        drawGridLinesWithHorizLabelsAndTriggerLine();
    }
    // Wait for latching relay to switch - 1ms is working - (2 does not work reliable after reset so take 4)
    delay(4);
    // No need for power any more
    digitalWriteFast(tRelaisPin, LOW);
}

uint16_t getAttenuatorFactor(void) {
    uint16_t tRetValue = 1;
    if (MeasurementControl.ChannelHasACDC) {
        for (int i = 0; i < MeasurementControl.AttenuatorValue; ++i) {
            tRetValue *= ATTENUATOR_FACTOR;
        }
    }
    return tRetValue;
}

/***********************************************************************
 * GUI initialization and drawing stuff
 ***********************************************************************/

void createGUI(void) {
    BlueDisplay1.setButtonsGlobalFlags(USE_UP_EVENTS_FOR_BUTTONS); // since swipe recognition needs it
// Button for Singleshot (and settings/back)
    TouchButtonSingleshot = BlueDisplay1.createButtonPGM(BUTTON_WIDTH_3_POS_3, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_GUI_CONTROL, PSTR("Single"), TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doTriggerSingleshot);

    TouchButtonBack = BlueDisplay1.createButtonPGM(BUTTON_WIDTH_3_POS_3, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_GUI_CONTROL,
            PSTR("Back"), TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doBack);

// big start stop button
    TouchButtonStartStop = BlueDisplay1.createButtonPGM(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3,
            2 * BUTTON_HEIGHT_4 + BUTTON_DEFAULT_SPACING, COLOR_GUI_CONTROL, PSTR("Start/Stop"), TEXT_SIZE_11,
            BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doStartStop);

// Button for settings page
    TouchButtonSettings = BlueDisplay1.createButtonPGM(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4, COLOR_GUI_CONTROL, PSTR("Settings"), TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doShowSettingsPage);

    /*
     * History
     */
// Button for chart history (erase color)
    TouchButtonChartHistoryOnOff = BlueDisplay1.createButtonPGM(0, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, PSTR("History"),
    TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doChartHistory);

    /*
     * Settings trigger, slope, range and offset
     */

// Button for auto trigger on off
    TouchButtonAutoTriggerOnOff = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_2, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doTriggerAutoManualFree);
    setTriggerAutoOnOffButtonCaption();

// Button for slope
    TouchButtonSlope = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doTriggerSlope);
    setSlopeButtonCaption();

// Button for range
    TouchButtonAutoRangeOnOff = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4, COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doRangeMode);
    setAutoRangetButtonCaption();

// Button for auto offset on off
    TouchButtonAutoOffsetOnOff = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4, COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doAutoOffsetOnOff);
    setAutoOffsetButtonCaption();

    /*
     * AC/DC, Channel + Reference
     */

// Button for AC / DC
    TouchButtonAcDc = BlueDisplay1.createButton(0, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_GUI_SOURCE_TIMEBASE, "", TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doAcDc);
    setACModeButtonCaption();

// Button for channel 0
    TouchButtonChannels[0] = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4, COLOR_GUI_SELECTED, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doChannelSelect);

// Button for channel 1
    TouchButtonChannels[1] = BlueDisplay1.createButton(DISPLAY_WIDTH - BUTTON_WIDTH_6, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4, COLOR_GUI_SELECTED, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 1, &doChannelSelect);

// Button for channel 2
    TouchButtonChannels[2] = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4, COLOR_GUI_SELECTED, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 2, &doChannelSelect);
    setChannelButtonsCaption();

// Button for channel select
    TouchButtonChannelSelect = BlueDisplay1.createButtonPGM(DISPLAY_WIDTH - BUTTON_WIDTH_6, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4, COLOR_GUI_SELECTED, Channel3ButtonString, TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 42,
            &doChannelSelect);

// Button for reference voltage switching
    TouchButtonADCReference = BlueDisplay1.createButton(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4, COLOR_GUI_SOURCE_TIMEBASE, "", TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doADCReference);
    setReferenceButtonCaption();

    /*
     * SLIDER
     */
// make slider slightly visible
// slider for voltage picker
    TouchSliderVoltagePicker = BlueDisplay1.createSlider(SLIDER_VPICKER_VALUE_X - TEXT_SIZE_11_WIDTH + 4, 0, 6, DISPLAY_HEIGHT,
    DISPLAY_HEIGHT, 0, COLOR_SLIDER, COLOR_BACKGROUND_DSO, TOUCHSLIDER_SHOW_BORDER | TOUCHSLIDER_VALUE_BY_CALLBACK,
            &doVoltagePicker);

// slider for trigger level
    TouchSliderTriggerLevel = BlueDisplay1.createSlider(SLIDER_TLEVEL_VALUE_X - 4, 0, 6, DISPLAY_HEIGHT, DISPLAY_HEIGHT, 0,
    COLOR_SLIDER, COLOR_BACKGROUND_DSO, TOUCHSLIDER_SHOW_BORDER | TOUCHSLIDER_VALUE_BY_CALLBACK, &doTriggerLevel);
}

void drawStartGui(void) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_START;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    BlueDisplay1.drawButton(TouchButtonStartStop);
    BlueDisplay1.drawButton(TouchButtonSingleshot);
    BlueDisplay1.drawButton(TouchButtonSettings);
}

void drawDSOSettingsPageGui(void) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_SETTINGS;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);

    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    if (MeasurementControl.ChannelHasACDC) {
        BlueDisplay1.drawButton(TouchButtonAcDc);
    }
    BlueDisplay1.drawButton(TouchButtonAutoTriggerOnOff);
    BlueDisplay1.drawButton(TouchButtonBack);
    BlueDisplay1.drawButton(TouchButtonSlope);
    BlueDisplay1.drawButton(TouchButtonAutoRangeOnOff);
    if (!MeasurementControl.isACMode) {
        BlueDisplay1.drawButton(TouchButtonAutoOffsetOnOff);
    }

    /*
     * Determine colors for channel buttons
     */
    int16_t tButtonColor;
    for (uint8_t i = 0; i < ADC_DIRECT_CHANNEL_BUTTONS; ++i) {
        if (i == MeasurementControl.ADCInputMUXChannel) {
            tButtonColor = COLOR_GUI_SELECTED;
        } else {
            tButtonColor = COLOR_GUI_NOT_SELECTED;
        }
        BlueDisplay1.setButtonColorAndDraw(TouchButtonChannels[i], tButtonColor);
    }
    if (MeasurementControl.ADCInputMUXChannel >= ADC_DIRECT_CHANNEL_BUTTONS) {
        tButtonColor = COLOR_GUI_SELECTED;
    } else {
        tButtonColor = COLOR_GUI_NOT_SELECTED;
    }
    BlueDisplay1.setButtonColorAndDraw(TouchButtonChannelSelect, tButtonColor);

    BlueDisplay1.drawButton(TouchButtonADCReference);
    BlueDisplay1.drawButton(TouchButtonChartHistoryOnOff);
    BlueDisplay1.drawButton(TouchButtonFrequency);
}

/*
 * activate elements if returning from settings screen or if starting acquisition
 */
void activatePartOfGui(void) {
// first deactivate all
    BlueDisplay1.deactivateAllButtons();

    BlueDisplay1.activateButton(TouchButtonStartStop);
    BlueDisplay1.activateButton(TouchButtonSingleshot);
    BlueDisplay1.activateButton(TouchButtonSettings);
    BlueDisplay1.activateButton(TouchButtonChartHistoryOnOff);

    BlueDisplay1.drawSliderBorder(TouchSliderVoltagePicker);
    BlueDisplay1.activateSlider(TouchSliderVoltagePicker);
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
        BlueDisplay1.drawSliderBorder(TouchSliderTriggerLevel);
        BlueDisplay1.activateSlider(TouchSliderTriggerLevel);
    }
}

void redrawDisplay() {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    if (MeasurementControl.isRunning) {
        if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
            BlueDisplay1.drawSliderBorder(TouchSliderTriggerLevel);
        }
        if (DisplayControl.DisplayPage >= DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPageGui();
        } else {
            activatePartOfGui();
        }
        // refresh grid
        drawGridLinesWithHorizLabelsAndTriggerLine();
        printInfo();
    } else {
        // analyze mode
        if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
            drawStartGui();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
            activatePartOfGui();
            drawGridLinesWithHorizLabelsAndTriggerLine();
            drawMinMaxLines();
            drawDataBuffer(&DataBufferControl.DataBuffer[0], COLOR_DATA_HOLD, DisplayControl.EraseColor);
            printInfo();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPageGui();
        } else {
            drawFrequencyGui();
        }
    }
}

/************************************************************************
 * GUI handler section
 ************************************************************************/
/*
 * Handler for "empty" touch
 * Use touch up in order not to interfere with long touch
 * Switch between upper info line short/long/off
 */
void TouchUpHandler(struct XYPosition * const aTochPosition) {
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
// Wrap display mode
        uint8_t tNewMode = DisplayControl.showInfoMode + 1;
        if (tNewMode > INFO_MODE_LONG_INFO) {
            tNewMode = INFO_MODE_NO_INFO;
        }
        DisplayControl.showInfoMode = tNewMode;
        // erase former (short) info line
        clearInfo();
        if (tNewMode != INFO_MODE_NO_INFO) {
            printInfo();
        } else if (!MeasurementControl.isRunning) {
            redrawDisplay();
        }
    }
}

/*
 * If stopped toggle between Start and Chart page
 * if running and Chart page show Settings page
 */
void longTouchDownHandler(struct XYPosition * const aTochPosition) {
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        if (MeasurementControl.isRunning) {
            // Show settings page
            drawDSOSettingsPageGui();
        } else {
            // clear screen and show only gui
            DisplayControl.DisplayPage = DISPLAY_PAGE_START;
            redrawDisplay();
        }
    } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
        // only chart (and voltage picker)
        DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
        // flag to avoid initial clearing of (not existent) picker and trigger level lines
        LastPickerValue = 0xFF;
        redrawDisplay();
    }
}

void swipeEndHandler(struct Swipe * const aSwipeInfo) {
    bool tOKBeep = true;

    if (aSwipeInfo->TouchDeltaAbsMax > 32) {
        int8_t tTouchDeltaXGrid = aSwipeInfo->TouchDeltaX / 32;
        if (MeasurementControl.isRunning) {
            if (aSwipeInfo->SwipeMainDirectionIsX) {
                /*
                 * Timebase -> use tTouchDeltaXGrid/2
                 */
                tTouchDeltaXGrid /= 2;
                if (tTouchDeltaXGrid == 0) {
                    tOKBeep = false;
                } else {
                    tOKBeep = changeTimeBaseValue(-tTouchDeltaXGrid, true);
                }
            } else {
                if (!MeasurementControl.RangeAutomatic) {
                    /*
                     * Range
                     */
                    tOKBeep = changeRange(aSwipeInfo->TouchDeltaY / 32);
                } else {
                    tOKBeep = false;
                }
            }
        } else {
            if (aSwipeInfo->SwipeMainDirectionIsX) {
                //if (aSwipeInfo.TouchStartY > BUTTON_HEIGHT_4_LINE_3) {
                /*
                 * Scroll
                 */
                tOKBeep = scrollChart(-tTouchDeltaXGrid);
//                } else {
//                    /*
//                     * X-Scale
//                     */
//                }
            } else {
                tOKBeep = false;
            }
        }
    } else {
        tOKBeep = false;
    }
    FeedbackTone(tOKBeep);
}

/************************************************************************
 * BUTTON handler section
 ************************************************************************/
/*
 * toggle between DC and AC mode
 */
void doAcDc(uint8_t aTheTouchedButton, int16_t aValue) {
    setACMode(!MeasurementControl.isACMode);
}

/*
 * toggle between automatic and manual trigger voltage value
 */
void doTriggerAutoManualFree(uint8_t aTheTouchedButton, int16_t aValue) {
    uint8_t tNewMode = MeasurementControl.TriggerMode + 1;
    if (tNewMode == TRIGGER_MODE_FREE) {
        // set TriggerTimeoutSampleCount to zero for free running
        MeasurementControl.TriggerTimeoutSampleCount = 0;
    } else if (tNewMode > TRIGGER_MODE_FREE) {
        tNewMode = TRIGGER_MODE_AUTO;
        // set TriggerTimeoutSampleCount to regular value
        MeasurementControl.TriggerMode = tNewMode;
        changeTimeBaseValue(0, false);
    }
    MeasurementControl.TriggerMode = tNewMode;
// use aTheTouchedButton only if needed 3 times ore more otherwise it increases program size
    setTriggerAutoOnOffButtonCaption();
}

void doRangeMode(uint8_t aTheTouchedButton, int16_t aValue) {
    MeasurementControl.RangeAutomatic = !MeasurementControl.RangeAutomatic;
    setAutoRangetButtonCaption();
}

/*
 * toggle between ascending and descending trigger slope
 */
void doTriggerSlope(uint8_t aTheTouchedButton, int16_t aValue) {
    MeasurementControl.TriggerSlopeRising = (!MeasurementControl.TriggerSlopeRising);
    setSlopeButtonCaption();
}

/*
 * toggle between auto and 0 Volt offset
 * No auto offset in AC Mode
 */
void doAutoOffsetOnOff(uint8_t aTheTouchedButton, int16_t aValue) {
    setOffsetAutomatic(!MeasurementControl.OffsetAutomatic);
}

/*
 * toggle between 5 and 1.1 Volt reference
 */
void doADCReference(uint8_t aTheTouchedButton, int16_t aValue) {
    uint8_t tNewReference = MeasurementControl.ADCReference;
    if (MeasurementControl.ADCReference == DEFAULT) {
        tNewReference = INTERNAL;
    } else {
        tNewReference = DEFAULT;
    }
    setReference(tNewReference);
    setReferenceButtonCaption();
    if (!MeasurementControl.RangeAutomatic) {
        // set new grid values
        setInputRange(MeasurementControl.ShiftValue, MeasurementControl.AttenuatorValue);
    }
}

/*
 * Cycle through all external and internal adc channels
 */
void doChannelSelect(uint8_t aTheTouchedButton, int16_t aValue) {
    if (aValue > 16) {
        // channel increment button here
        uint8_t tOldValue = MeasurementControl.ADCInputMUXChannel;
        // increment channel but only if still at channel 3
        if (tOldValue >= ADC_DIRECT_CHANNEL_BUTTONS) {
            aValue = tOldValue + 1;
        } else {
            aValue = ADC_DIRECT_CHANNEL_BUTTONS;
        }
    }
    setChannel(aValue);
}

/*
 * show gui of settings screen
 */
void doShowSettingsPage(uint8_t aTheTouchedButton, int16_t aValue) {
    drawDSOSettingsPageGui();
}

void doBack(uint8_t aTheTouchedButton, int16_t aValue) {
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_FREQUENCY) {
        drawDSOSettingsPageGui();
    } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        if (MeasurementControl.isRunning) {
            DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
            BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
            drawGridLinesWithHorizLabelsAndTriggerLine();
            printInfo();
            activatePartOfGui();
        } else {
            drawStartGui();
        }
    }
}
/*
 * set to singleshot mode and draw an indicating "S"
 */
void doTriggerSingleshot(uint8_t aTheTouchedButton, int16_t aValue) {
    MeasurementControl.isSingleShotMode = true;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
    drawGridLinesWithHorizLabelsAndTriggerLine();
// draw an S to indicate running single shot trigger
    BlueDisplay1.drawChar(INFO_LEFT_MARGIN + SINGLESHOT_PPRINT_VALUE_X, INFO_UPPER_MARGIN + TEXT_SIZE_11_HEIGHT, 'S', TEXT_SIZE_11,
    COLOR_BLACK, COLOR_INFO_BACKGROUND);

// prepare info output - at least 1 sec later
    MillisSinceLastInfoOutput = 0;
    MeasurementControl.RawValueMax = 0;
    MeasurementControl.RawValueMin = 0;

// Start a new single shot
    startAcquisition();
    MeasurementControl.isRunning = true;
}

void doStartStop(uint8_t aTheTouchedButton, int16_t aValue) {
    if (MeasurementControl.isRunning) {
        /*
         * Stop here
         * for the last measurement read full buffer size
         * Do this asynchronously to the interrupt routine in order to extend a running or started acquisition
         * stop single shot mode
         */
        DataBufferControl.DataBufferEndPointer = &DataBufferControl.DataBuffer[DATABUFFER_SIZE - 1];
//        if (DataBufferControl.DataBufferFull) {
//            // start a new acquisition
//            startAcquisition();
//        }
// in SingleShotMode Stop is always requested
        if (MeasurementControl.StopRequested && !MeasurementControl.isSingleShotMode) {
            // for stop requested 2 times -> stop immediately
            uint8_t* tEndPointer = DataBufferControl.DataBufferNextInPointer;
            DataBufferControl.DataBufferEndPointer = tEndPointer;
            // clear trailing buffer space not used
            memset(tEndPointer, 0xFF, ((uint8_t*) &DataBufferControl.DataBuffer[DATABUFFER_SIZE]) - ((uint8_t*) tEndPointer));
        }
// return to continuous  mode
        MeasurementControl.isSingleShotMode = false;
        MeasurementControl.StopRequested = true;
// AcquisitionSize is used in synchronous fast loop so we can set it here
        DataBufferControl.AcquisitionSize = DATABUFFER_SIZE;
        DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];
// no feedback tone, it kills the timing!
    } else {
// Start here
        BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
        DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
        //DisplayControl.showInfoMode = true;
        activatePartOfGui();
        drawGridLinesWithHorizLabelsAndTriggerLine();
        MeasurementControl.isSingleShotMode = false;
        startAcquisition();
        MeasurementControl.isRunning = true;
    }
}

/*
 * Toggle history mode
 */
void doChartHistory(uint8_t aTheTouchedButton, int16_t aValue) {
    DisplayControl.showHistory = !aValue;
    BlueDisplay1.setRedGreenButtonColor(aTheTouchedButton, DisplayControl.showHistory,
            (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));

    if (DisplayControl.showHistory) {
        DisplayControl.EraseColor = COLOR_DATA_HISTORY;
    } else {
        DisplayControl.EraseColor = COLOR_BACKGROUND_DSO;
        if (MeasurementControl.isRunning) {
            redrawDisplay();
        }
    }
}

/*
 * The value printed has a resolution of 0,00488 * scale factor
 */
void doTriggerLevel(uint8_t aTheTouchedSlider, int16_t aValue) {
// in auto-trigger mode show only computed value and do not modify it
    if (MeasurementControl.TriggerMode != TRIGGER_MODE_MANUAL) {
// already shown :-)
        return;
    }
// to get display value take -aValue and vice versa
    aValue = DISPLAY_VALUE_FOR_ZERO - aValue;
    if (DisplayControl.TriggerLevelDisplayValue == aValue) {
        return;
    }

// clear old trigger line
    clearTriggerLine(DisplayControl.TriggerLevelDisplayValue);
// store actual display value
    DisplayControl.TriggerLevelDisplayValue = aValue;

// modify trigger values
    uint16_t tLevel = getRawFromDisplayValue(aValue);
    setLevelAndHysteresis(tLevel, TRIGGER_HYSTERESIS_MANUAL);

// draw new line
    drawTriggerLine();
}

/*
 * The value printed has a resolution of 0,00488 * scale factor
 */
void doVoltagePicker(uint8_t aTheTouchedSlider, int16_t aValue) {
    char tVoltageBuffer[6];
    if (LastPickerValue == aValue) {
        return;
    }
    if (LastPickerValue != 0xFF) {
// clear old line
        uint16_t tYpos = DISPLAY_VALUE_FOR_ZERO - LastPickerValue;
        BlueDisplay1.drawLineRel(0, tYpos, DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
// restore grid at old y position
        for (uint16_t tXPos = TIMING_GRID_WIDTH - 1; tXPos < DISPLAY_WIDTH - 1; tXPos += TIMING_GRID_WIDTH) {
            BlueDisplay1.drawPixel(tXPos, tYpos, COLOR_TIMING_LINES);
        }
    }
// draw new line
    uint8_t tValue = DISPLAY_VALUE_FOR_ZERO - aValue;
    BlueDisplay1.drawLineRel(0, tValue, DISPLAY_WIDTH, 0, COLOR_DATA_PICKER);
    LastPickerValue = aValue;

    float tVoltage = getFloatFromDisplayValue(tValue);
    dtostrf(tVoltage, 4, 2, tVoltageBuffer);
    tVoltageBuffer[4] = 'V';
    tVoltageBuffer[5] = '\0';
    BlueDisplay1.drawText(INFO_LEFT_MARGIN + SLIDER_VPICKER_VALUE_X, INFO_UPPER_MARGIN + 13, tVoltageBuffer, 11, COLOR_BLACK,
    COLOR_INFO_BACKGROUND);
}
/************************************************************************
 * Button caption section
 ************************************************************************/

void setChannelButtonsCaption(void) {
    for (uint8_t i = 0; i < ADC_DIRECT_CHANNEL_BUTTONS; ++i) {
        if (MeasurementControl.AttenuatorType == ATTENUATOR_TYPE_SIMPLE_ATTENUATOR) {
            BlueDisplay1.setButtonCaptionPGM(TouchButtonChannels[i], ChannelDivByButtonStrings[i],
                    (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
        } else {
            ChannelSelectButtonString[CHANNEL_STRING_INDEX] = 0x30 + i;
            BlueDisplay1.setButtonCaption(TouchButtonChannels[i], ChannelSelectButtonString,
                    (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
        }
    }
}

void setReferenceButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.ADCReference == DEFAULT) {
        tCaption = ReferenceButtonVCC;
    } else {
        tCaption = ReferenceButton1_1V;
    }
    BlueDisplay1.setButtonCaptionPGM(TouchButtonADCReference, tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setACModeButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.isACMode) {
        tCaption = AcDcButtonAC;
    } else {
        tCaption = AcDcButtonDC;
    }
    BlueDisplay1.setButtonCaptionPGM(TouchButtonAcDc, tCaption, false); // false, since complete page is drawn after setting
}

void setTriggerAutoOnOffButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_AUTO) {
        tCaption = AutoTriggerButtonStringAuto;
    } else if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
        tCaption = AutoTriggerButtonStringManual;
    } else {
        tCaption = AutoTriggerButtonStringFree;
    }
    BlueDisplay1.setButtonCaptionPGM(TouchButtonAutoTriggerOnOff, tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setAutoOffsetButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.OffsetAutomatic) {
        tCaption = AutoOffsetButtonStringAuto;
    } else {
        tCaption = AutoOffsetButtonString0;
    }
    BlueDisplay1.setButtonCaptionPGM(TouchButtonAutoOffsetOnOff, tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setAutoRangetButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.RangeAutomatic) {
        tCaption = AutoRangeButtonStringAuto;
    } else {
        tCaption = AutoRangeButtonStringManual;
    }
    BlueDisplay1.setButtonCaptionPGM(TouchButtonAutoRangeOnOff, tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setSlopeButtonCaption(void) {
    uint8_t tChar;
    if (MeasurementControl.TriggerSlopeRising) {
        tChar = 'A';
    } else {
        tChar = 'D';
    }
    SlopeButtonString[SLOPE_STRING_INDEX] = tChar;
    BlueDisplay1.setButtonCaption(TouchButtonSlope, SlopeButtonString, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

/************************************************************************
 * Graphical output section
 ************************************************************************/
/*
 * returns true if display was scrolled
 */
bool scrollChart(int8_t aScrollAmount) {
    if (DisplayControl.DisplayPage != DISPLAY_PAGE_CHART) {
        return false;
    }
    bool tRetValue = true;
    /*
     * set start of display in data buffer
     */
    DataBufferControl.DataBufferDisplayStart += aScrollAmount * TIMING_GRID_WIDTH / DisplayControl.XScale;
// check against begin of buffer
    if (DataBufferControl.DataBufferDisplayStart < &DataBufferControl.DataBuffer[0]) {
        DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];
        tRetValue = false;
    } else {
        uint8_t * tMaxAddress = &DataBufferControl.DataBuffer[DATABUFFER_SIZE];
        if (MeasurementControl.TimebaseIndex <= TIMEBASE_INDEX_FAST_MODES) {
            // Only half of data buffer is filled
            tMaxAddress = &DataBufferControl.DataBuffer[DATABUFFER_SIZE / 2];
        }
        tMaxAddress = tMaxAddress - (DISPLAY_WIDTH / DisplayControl.XScale);
        if (DataBufferControl.DataBufferDisplayStart > tMaxAddress) {
            DataBufferControl.DataBufferDisplayStart = tMaxAddress;
            tRetValue = false;
        }
    }
    drawDataBuffer(DataBufferControl.DataBufferDisplayStart, COLOR_DATA_HOLD, COLOR_BACKGROUND_DSO);

    return tRetValue;
}

void clearTriggerLine(uint8_t aTriggerLevelDisplayValue) {
    BlueDisplay1.drawLineRel(0, aTriggerLevelDisplayValue, DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
    if (DisplayControl.showInfoMode != INFO_MODE_NO_INFO) {
// restore grid at old y position
        for (uint16_t tXPos = TIMING_GRID_WIDTH - 1; tXPos < DISPLAY_WIDTH - 1; tXPos += TIMING_GRID_WIDTH) {
            BlueDisplay1.drawPixel(tXPos, aTriggerLevelDisplayValue, COLOR_TIMING_LINES);
        }
    }
}

void drawTriggerLine(void) {
    if (MeasurementControl.TriggerMode != TRIGGER_MODE_FREE) {
        uint8_t tValue = DisplayControl.TriggerLevelDisplayValue;
        BlueDisplay1.drawLineRel(0, tValue, DISPLAY_WIDTH, 0, COLOR_TRIGGER_LINE);
    }
}

#define HORIZONTAL_LINE_LABELS_CAPION_X (DISPLAY_WIDTH - TEXT_SIZE_11_WIDTH * 4)

void clearHorizontalGridLinesAndHorizontalLineLabels(void) {
    if (MeasurementControl.isACMode) {
// Start at DISPLAY_HEIGHT/2 shift 8 minus 1/2 shift 8
        for (int32_t tYPosLoop = 0x8000; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // clear line
            BlueDisplay1.drawLineRel(0, tYPos, DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
            // clear label
            BlueDisplay1.fillRectRel(HORIZONTAL_LINE_LABELS_CAPION_X - TEXT_SIZE_11_WIDTH, tYPos - (TEXT_SIZE_11_HEIGHT / 2),
                    (5 * TEXT_SIZE_11_WIDTH) - 1, TEXT_SIZE_11_HEIGHT, COLOR_BACKGROUND_DSO);
            if (tYPos != DISPLAY_HEIGHT / 2) {
                BlueDisplay1.drawLineRel(0, DISPLAY_HEIGHT - tYPos, DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
                BlueDisplay1.fillRectRel(HORIZONTAL_LINE_LABELS_CAPION_X - TEXT_SIZE_11_WIDTH,
                DISPLAY_HEIGHT - tYPos - (TEXT_SIZE_11_HEIGHT / 2), (5 * TEXT_SIZE_11_WIDTH) - 1, TEXT_SIZE_11_HEIGHT,
                COLOR_BACKGROUND_DSO);
            }
        }
    } else {
        int8_t tCaptionOffset = TEXT_SIZE_11_HEIGHT;
// Start at (DISPLAY_VALUE_FOR_ZERO) shift 8) + 1/2 shift8 for better rounding
        for (int32_t tYPosLoop = 0xFF80; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // clear line
            BlueDisplay1.drawLineRel(0, tYPos, DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
            // clear label
            BlueDisplay1.fillRectRel(HORIZONTAL_LINE_LABELS_CAPION_X - TEXT_SIZE_11_WIDTH, tYPos - tCaptionOffset,
                    (5 * TEXT_SIZE_11_WIDTH) - 1, TEXT_SIZE_11_HEIGHT, COLOR_BACKGROUND_DSO);
            tCaptionOffset = (TEXT_SIZE_11_HEIGHT / 2);
        }
    }
}

/*
 * draws vertical timing + horizontal reference voltage lines
 */
void drawGridLinesWithHorizLabelsAndTriggerLine(void) {
// vertical (timing) lines
    for (uint16_t tXPos = TIMING_GRID_WIDTH - 1; tXPos < DISPLAY_WIDTH; tXPos += TIMING_GRID_WIDTH) {
        BlueDisplay1.drawLineRel(tXPos, 0, 0, DISPLAY_HEIGHT, COLOR_TIMING_LINES);
    }

    /*
     * drawHorizontalLineLabels
     */
    float tActualVoltage = 0;
    char tStringBuffer[6];
    uint8_t tPrecision = 2 - MeasurementControl.AttenuatorValue;
    uint8_t tLength = 2 + tPrecision;
    if (MeasurementControl.isACMode) {
        /*
         * draw from middle of screen to top and "mirror" lines for negative values
         */
// Start at DISPLAY_HEIGHT/2 shift 8
        for (int32_t tYPosLoop = 0x8000; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // horizontal line
            BlueDisplay1.drawLineRel(0, tYPos, DISPLAY_WIDTH, 0, COLOR_TIMING_LINES);
            dtostrf(tActualVoltage, tLength, tPrecision, tStringBuffer);
            // draw label over the line
            BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X, tYPos + (TEXT_SIZE_11_ASCEND / 2), tStringBuffer, 11,
            COLOR_HOR_REF_LINE_LABEL, COLOR_NO_BACKGROUND);
            if (tYPos != DISPLAY_HEIGHT / 2) {
                // line with negative value
                BlueDisplay1.drawLineRel(0, DISPLAY_HEIGHT - tYPos, DISPLAY_WIDTH, 0, COLOR_TIMING_LINES);
                dtostrf(-tActualVoltage, tLength, tPrecision, tStringBuffer);
                // draw label over the line
                BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X - TEXT_SIZE_11_WIDTH,
                DISPLAY_HEIGHT - tYPos + (TEXT_SIZE_11_ASCEND / 2), tStringBuffer, 11, COLOR_HOR_REF_LINE_LABEL,
                COLOR_NO_BACKGROUND);
            }
            tActualVoltage += MeasurementControl.HorizontalGridVoltage;
        }
    } else {
        if (MeasurementControl.OffsetAutomatic) {
            tActualVoltage = MeasurementControl.HorizontalGridVoltage * MeasurementControl.OffsetGridCount;
        }
// draw first caption over the line
        int8_t tCaptionOffset = 1;
// Start at (DISPLAY_VALUE_FOR_ZERO) shift 8) + 1/2 shift8 for better rounding
        for (int32_t tYPosLoop = 0xFF80; tYPosLoop > 0; tYPosLoop -= MeasurementControl.HorizontalGridSizeShift8) {
            uint16_t tYPos = tYPosLoop / 0x100;
            // horizontal line
            BlueDisplay1.drawLineRel(0, tYPos, DISPLAY_WIDTH, 0, COLOR_TIMING_LINES);
            dtostrf(tActualVoltage, tLength, tPrecision, tStringBuffer);
            // draw label over the line
            BlueDisplay1.drawText(HORIZONTAL_LINE_LABELS_CAPION_X, tYPos - tCaptionOffset, tStringBuffer, 11,
            COLOR_HOR_REF_LINE_LABEL, COLOR_NO_BACKGROUND);
            // draw next caption on the line
            tCaptionOffset = -(TEXT_SIZE_11_ASCEND / 2);
            tActualVoltage += MeasurementControl.HorizontalGridVoltage;
        }
    }
    drawTriggerLine();
}

/*
 * draws min, max lines
 */
void drawMinMaxLines(void) {
// draw max line
    uint8_t tValueDisplay = getDisplayFromRawValue(MeasurementControl.RawValueMax);
    if (tValueDisplay != 0) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
    }
// min line
    tValueDisplay = getDisplayFromRawValue(MeasurementControl.RawValueMin);
    if (tValueDisplay != DISPLAY_VALUE_FOR_ZERO) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
    }
}

void drawDataBuffer(uint8_t *aByteBuffer, uint16_t aColor, uint16_t aClearBeforeColor) {
    uint8_t tXScale = DisplayControl.XScale;
    uint8_t * tBufferPtr = aByteBuffer;
    if (tXScale > 1) {
// expand - show value several times
        uint8_t * tDisplayBufferPtr = &DataBufferControl.DisplayBuffer[0];
        uint8_t tXScaleCounter = tXScale;
        uint8_t tValue = *tBufferPtr++;
        for (uint16_t i = 0; i < sizeof(DataBufferControl.DisplayBuffer); ++i) {
            if (tXScaleCounter == 0) {
                tValue = *tBufferPtr++;
                tXScaleCounter = tXScale;
            }
            tXScaleCounter--;
            *tDisplayBufferPtr++ = tValue;
        }
        tBufferPtr = &DataBufferControl.DisplayBuffer[0];
    }
    BlueDisplay1.drawChartByteBuffer(0, 0, aColor, aClearBeforeColor, tBufferPtr, sizeof(DataBufferControl.DisplayBuffer));
}

void clearDisplayedChart(uint8_t * aDisplayBufferPtr) {
    BlueDisplay1.drawChartByteBuffer(0, 0, COLOR_BACKGROUND_DSO, COLOR_NO_BACKGROUND, aDisplayBufferPtr,
            sizeof(DataBufferControl.DisplayBuffer));
}

/*
 * Draws only one chart value - used for drawing while sampling
 */
void drawRemainingDataBufferValues(void) {
// check if end of display buffer reached - needed for last acquisition which uses the whole data buffer
    uint8_t tLastValueByte;
    uint8_t tNextValueByte;
    uint8_t tValueByte;
    uint16_t tBufferIndex = DataBufferControl.DataBufferNextDrawIndex;

    while (DataBufferControl.DataBufferNextDrawPointer < DataBufferControl.DataBufferNextInPointer && tBufferIndex < DISPLAY_WIDTH) {
        /*
         * clear old line
         */
        if (tBufferIndex < DISPLAY_WIDTH - 1) {
            // fetch next value and clear line in advance
            tValueByte = DataBufferControl.DisplayBuffer[tBufferIndex];
            tNextValueByte = DataBufferControl.DisplayBuffer[tBufferIndex + 1];
            BlueDisplay1.drawLineOneX(tBufferIndex, tValueByte, tNextValueByte, DisplayControl.EraseColor);
        }

        /*
         * get new value
         */
        tValueByte = *DataBufferControl.DataBufferNextDrawPointer++;
        DataBufferControl.DisplayBuffer[tBufferIndex] = tValueByte;

        if (tBufferIndex != 0 && tBufferIndex <= DISPLAY_WIDTH - 1) {
            // get last value and draw line
            tLastValueByte = DataBufferControl.DisplayBuffer[tBufferIndex - 1];
            BlueDisplay1.drawLineOneX(tBufferIndex - 1, tLastValueByte, tValueByte, COLOR_DATA_RUN);
        }
        tBufferIndex++;
    }
    DataBufferControl.DataBufferNextDrawIndex = tBufferIndex;
}

/************************************************************************
 * Text output section
 ************************************************************************/
void formatThousandSeparator(char * aThousandPosition) {
    char tNewChar;
    char tOldChar = *aThousandPosition;
//set separator for thousands
    *aThousandPosition-- = THOUSANDS_SEPARATOR;
    for (uint8_t i = 2; i > 0; i--) {
        tNewChar = *aThousandPosition;
        *aThousandPosition-- = tOldChar;
        tOldChar = tNewChar;
    }
}
/*
 * clear info line
 */
void clearInfo(void) {
    BlueDisplay1.fillRectRel(INFO_LEFT_MARGIN, 1, DISPLAY_WIDTH - INFO_LEFT_MARGIN, (2 * TEXT_SIZE_11) + TEXT_SIZE_11_DECEND,
    COLOR_BACKGROUND_DSO);
}
/*
 * Output info line
 * for documentation see line 33 of this file
 */
void printInfo(void) {
    if (DisplayControl.showInfoMode == INFO_MODE_NO_INFO) {
        return;
    }

    char tSlopeChar;
    char tTimebaseUnitChar;
    char tReferenceChar;
    char tMinStringBuffer[6];
    char tAverageStringBuffer[6];
    char tMaxStringBuffer[5];
    char tP2PStringBuffer[5];
    char tTriggerStringBuffer[6];
    float tRefMultiplier;

    float tVoltage;

    if (MeasurementControl.TriggerSlopeRising) {
        tSlopeChar = '\xD1';
    } else {
        tSlopeChar = '\xD2';
    }
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        tRefMultiplier = 2.0 / 1.1;
    } else {
        tRefMultiplier = 1;
    }
    if (MeasurementControl.ADCReference == DEFAULT) {
        tReferenceChar = '5';
        tRefMultiplier *= MeasurementControl.VCC / 1024.0;
        /*
         * Use 1023 to get 5V display for full scale reading
         * Better would be 5.0 / 1024.0; since the reading for value for 5V- 1LSB is also 1023,
         * but this implies that the maximum displayed value is 4.99(51171875) :-(
         */
    } else {
        tReferenceChar = '1';
        tRefMultiplier *= 1.1 / 1024.0;
    }
    tRefMultiplier *= getAttenuatorFactor();

    uint8_t tPrecision = 2 - MeasurementControl.AttenuatorValue;
    int16_t tACOffset = 0;
    if (MeasurementControl.isACMode) {
        tACOffset = MeasurementControl.RawDSOReadingACZero;
    }

// 2 kByte code size
    tVoltage = tRefMultiplier * ((int16_t) MeasurementControl.RawValueMin - tACOffset);
    dtostrf(tVoltage, 5, tPrecision, tMinStringBuffer);
    tVoltage = tRefMultiplier * ((int16_t) MeasurementControl.ValueAverage - tACOffset);
    dtostrf(tVoltage, 5, tPrecision, tAverageStringBuffer);
    tVoltage = tRefMultiplier * ((int16_t) MeasurementControl.RawValueMax - tACOffset);
// 4 since we need no sign
    dtostrf(tVoltage, 4, tPrecision, tMaxStringBuffer);
    tVoltage = tRefMultiplier * (MeasurementControl.RawValueMax - MeasurementControl.RawValueMin);
// 4 since we need no sign
    dtostrf(tVoltage, 4, tPrecision, tP2PStringBuffer);

    if (MeasurementControl.TriggerSlopeRising) {
        tVoltage = ((int16_t) MeasurementControl.TriggerLevelUpper - tACOffset);
    } else {
        tVoltage = ((int16_t) MeasurementControl.TriggerLevelLower - tACOffset);
    }
    tVoltage = tRefMultiplier * tVoltage;
    dtostrf(tVoltage, 5, tPrecision, tTriggerStringBuffer);

    uint16_t tTimebaseUnitsPerGrid;
    if (MeasurementControl.TimebaseIndex >= TIMEBASE_INDEX_MILLIS) {
        tTimebaseUnitChar = 'm';
    } else {
        tTimebaseUnitChar = '\xB5'; // micro
    }
    tTimebaseUnitsPerGrid = pgm_read_word(&TimebaseDivPrintValues[MeasurementControl.TimebaseIndex]);

    /*
     * Period and frequency
     */
    uint32_t tHertz = 0;
    if (MeasurementControl.PeriodMicros != 0) {
        tHertz = 1000000 / MeasurementControl.PeriodMicros;
    }

    if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        sprintf_P(StringBuffer, PSTR("%3u%cs %c Ch%c %s %s %s P2P%sV %sV %c"), tTimebaseUnitsPerGrid, tTimebaseUnitChar, tSlopeChar,
                MeasurementControl.ADCInputMUXChannelChar, tMinStringBuffer, tAverageStringBuffer, tMaxStringBuffer,
                tP2PStringBuffer, tTriggerStringBuffer, tReferenceChar);
        BlueDisplay1.drawText(INFO_LEFT_MARGIN, INFO_UPPER_MARGIN, StringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_INFO_BACKGROUND);
    } else {
        /*
         * Short version
         */
        sprintf_P(StringBuffer, PSTR("%sV %sV  %5luHz %3u%cs"), tAverageStringBuffer, tP2PStringBuffer, tHertz,
                tTimebaseUnitsPerGrid, tTimebaseUnitChar);
        if (tHertz >= 1000) {
            formatThousandSeparator(&StringBuffer[15]);
        }
        BlueDisplay1.drawText(INFO_LEFT_MARGIN, 1 + 12, StringBuffer, 18, COLOR_BLACK, COLOR_INFO_BACKGROUND);
    }

    if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        uint32_t tMicrosPeriod = MeasurementControl.PeriodMicros;
        char tPeriodUnitChar;
        if (tMicrosPeriod >= 50000l) {
            tMicrosPeriod = tMicrosPeriod / 1000;
            tPeriodUnitChar = 'm'; // milli
        } else {
            tPeriodUnitChar = '\xB5'; // micro
        }

        sprintf_P(StringBuffer, PSTR(" %5lu%cs  %5luHz"), tMicrosPeriod, tPeriodUnitChar, tHertz);
        if (tMicrosPeriod >= 1000) {
            formatThousandSeparator(&StringBuffer[2]);
        }
        if (tHertz >= 1000) {
            formatThousandSeparator(&StringBuffer[11]);
        }
        BlueDisplay1.drawText(INFO_LEFT_MARGIN, INFO_UPPER_MARGIN + TEXT_SIZE_11 + 2, StringBuffer, TEXT_SIZE_11, COLOR_BLACK,
        COLOR_INFO_BACKGROUND);
    }
}

/*
 * Show temperature and VCC voltage
 */
void printVCCAndTemperature(void) {
    if (!MeasurementControl.isRunning) {
        setVCCValue();
        dtostrf(MeasurementControl.VCC, 4, 2, &StringBuffer[30]);
        float tTemp = getTemperature();
        dtostrf(tTemp, 4, 1, &StringBuffer[40]);
        sprintf_P(StringBuffer, PSTR("%s Volt %s\xB0" "C"), &StringBuffer[30], &StringBuffer[40]);
        BlueDisplay1.drawText(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_11_DECEND + 3), StringBuffer, TEXT_SIZE_11,
        COLOR_BLACK, COLOR_BACKGROUND_DSO);
    }
}

/************************************************************************
 * Utility section
 ************************************************************************/
bool changeTimeBaseValue(int8_t aChangeValue, bool doOutput) {
    bool tRetValue = true;
    uint8_t tOldIndex = MeasurementControl.TimebaseIndex;

// positive value means increment timebase index!
    int8_t tNewIndex = tOldIndex + aChangeValue;
    if (tNewIndex < 0) {
        tNewIndex = 0;
        tRetValue = false;
    } else if (tNewIndex > TIMEBASE_NUMBER_OF_ENTRIES - 1) {
        tNewIndex = TIMEBASE_NUMBER_OF_ENTRIES - 1;
        tRetValue = false;
    }

    bool tStartNewAcquisition = false;
    DisplayControl.DrawWhileAcquire = (tNewIndex >= TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE);

    if (tOldIndex >= TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE && tNewIndex < TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE) {
// from draw while acquire to normal mode -> stop acquisition, clear old chart, and start a new one
        ADCSRA &= ~(1 << ADIE); // stop acquisition - disable ADC interrupt
        clearDisplayedChart(&DataBufferControl.DisplayBuffer[0]);
        tStartNewAcquisition = true;
    }

    if (tOldIndex < TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE && tNewIndex >= TIMEBASE_INDEX_DRAW_WHILE_ACQUIRE) {
// from normal to draw while acquire mode
        ADCSRA &= ~(1 << ADIE); // stop acquisition - disable ADC interrupt
        clearDisplayedChart(&DataBufferControl.DataBuffer[0]);
        tStartNewAcquisition = true;
    }

    MeasurementControl.TimebaseIndex = tNewIndex;
// delay handling - programmed delays between adc conversions
    MeasurementControl.TimebaseDelay = TimebaseDelayValues[tNewIndex];
    MeasurementControl.TimebaseDelayRemaining = TimebaseDelayValues[tNewIndex];

    uint16_t tTriggerTimeoutSampleCount = 0;
    if (MeasurementControl.TriggerMode != TRIGGER_MODE_FREE) {
        /*
         * Try to have the time for display as trigger timeout
         * don't go below 1/3 of a second (3200 samples) and above 3 seconds
         * formula is ms/div * 10 (divs) * 0,112 (millis per sample)
         *  => use TimebaseDivExactValues / 8 as trigger timeout
         */
        uint32_t tTriggerTimeoutSampleCount32 = pgm_read_float(&TimebaseDivExactValues[tNewIndex]);
        tTriggerTimeoutSampleCount32 = tTriggerTimeoutSampleCount32 >> 3;
        if (tTriggerTimeoutSampleCount32 > 50000) {
            tTriggerTimeoutSampleCount32 = 50000;
        }
        tTriggerTimeoutSampleCount = tTriggerTimeoutSampleCount32;
// wait at least 1/3 second
        if (tTriggerTimeoutSampleCount < TRIGGER_WAIT_NUMBER_OF_SAMPLES) {
            tTriggerTimeoutSampleCount = TRIGGER_WAIT_NUMBER_OF_SAMPLES;
        }
    }

    MeasurementControl.TriggerTimeoutSampleCount = tTriggerTimeoutSampleCount;

// reset xScale to regular value
    if (MeasurementControl.TimebaseIndex < TIMEBASE_NUMBER_OF_XSCALE_CORRECTION) {
        DisplayControl.XScale = xScaleForTimebase[tNewIndex];
    } else {
        DisplayControl.XScale = 1;
    }
    if (doOutput) {
        printInfo();
    }
    if (tStartNewAcquisition) {
        startAcquisition();
    }

    return tRetValue;
}

/*
 * Scan display buffer for trigger conditions.
 * Assume that first value is the first valid after a trigger (which does not hold for fast timebases)
 */
void computeMicrosPerPeriod(void) {
    /*
     * detect micros of period
     */
    uint8_t *DataPointer = &DataBufferControl.DataBuffer[0];
    uint8_t tValue;
    uint16_t tCount = 0;
    uint16_t tCountPosition = 0;
    uint16_t i = 0;
    uint8_t tTriggerStatus = TRIGGER_START;
    uint8_t tTriggerValueUpper = getDisplayFromRawValue(MeasurementControl.TriggerLevelUpper);
    uint8_t tTriggerValueLower = getDisplayFromRawValue(MeasurementControl.TriggerLevelLower);

    uint32_t tMicrosPerPeriod;
    for (; i < DISPLAY_WIDTH; ++i) {
        tValue = *DataPointer++;
//
// Display buffer contains inverted values !!!
//
        if (MeasurementControl.TriggerSlopeRising) {
            if (tTriggerStatus == TRIGGER_START) {
                // rising slope - wait for value below 1. threshold
                if (tValue > tTriggerValueLower) {
                    tTriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // rising slope - wait for value to rise above 2. threshold
                if (tValue < tTriggerValueUpper) {
                    // search for next slope
                    tCount++;
                    tCountPosition = i;
                    tTriggerStatus = TRIGGER_START;
                }
            }
        } else {
            if (tTriggerStatus == TRIGGER_START) {
                // falling slope - wait for value above 1. threshold
                if (tValue < tTriggerValueUpper) {
                    tTriggerStatus = TRIGGER_BEFORE_THRESHOLD;
                }
            } else {
                // falling slope - wait for value to go below 2. threshold
                if (tValue > tTriggerValueLower) {
                    // search for next slope
                    tCount++;
                    tCountPosition = i;
                    tTriggerStatus = TRIGGER_START;
                }
            }
        }
    }

// compute microseconds per period
    if (tCount == 0) {
        MeasurementControl.PeriodMicros = 0;
    } else {
        if (MeasurementControl.TimebaseIndex <= TIMEBASE_INDEX_FAST_MODES + 1) {
            tCountPosition++; // compensate for 1 measurement delay between trigger detection and acquisition
        }
        tMicrosPerPeriod = pgm_read_float(&TimebaseDivExactValues[MeasurementControl.TimebaseIndex]) * tCountPosition;
        MeasurementControl.PeriodMicros = tMicrosPerPeriod / (tCount * TIMING_GRID_WIDTH);
    }
}

uint8_t getDisplayFromRawValue(uint16_t aRawValue) {
    aRawValue -= MeasurementControl.OffsetValue;
    aRawValue >>= MeasurementControl.ShiftValue;
    aRawValue = DISPLAY_VALUE_FOR_ZERO - aRawValue;
    return aRawValue;
}

uint16_t getRawFromDisplayValue(uint8_t aDisplayValue) {
    aDisplayValue = DISPLAY_VALUE_FOR_ZERO - aDisplayValue;
    uint16_t aValue16 = aDisplayValue << MeasurementControl.ShiftValue;
    aValue16 = aValue16 + MeasurementControl.OffsetValue;
    return aValue16;
}
/*
 * computes corresponding voltage from display y position (DISPLAY_HEIGHT - 1 -> 0 Volt)
 */
float getFloatFromDisplayValue(uint8_t aDisplayValue) {
    float tFactor;
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        tFactor = 2.0 / 1.1;
    } else {
        tFactor = 1;
    }
    if (MeasurementControl.ADCReference == DEFAULT) {
        tFactor *= MeasurementControl.VCC / 1024.0;
    } else {
        tFactor *= 1.1 / 1024.0;
    }
    int16_t tRaw = getRawFromDisplayValue(aDisplayValue);
    if (MeasurementControl.isACMode) {
        tRaw -= MeasurementControl.RawDSOReadingACZero;
    }
// cannot multiply tRaw with getAttenuatorFactor() before since it can lead to 16 bit overflow
    tFactor *= tRaw;
    tFactor *= getAttenuatorFactor();
    return tFactor;
}

void FeedbackTone(bool isNoError) {
    if (isNoError) {
        BlueDisplay1.playTone(TONE_PROP_BEEP);
    } else {
        BlueDisplay1.playTone(TONE_PROP_BEEP2);
    }
}

/************************************************************************
 * Hardware support section
 ************************************************************************/
/*
 * take 64 samples with prescaler 128 from channel
 * This takes 13 ms (+ 10 ms optional delay)
 */
uint16_t getADCValue(uint8_t aChannel, uint8_t aReference) {
    uint8_t tOldADMUX = ADMUX;
    ADMUX = aChannel | (aReference << REFS0);
// Temperature channel also seem to need an initial delay
    delay(10);
    Myword tUValue;
    uint16_t tSum = 0; // uint16_t is sufficient for 64 samples
    uint8_t tOldADCSRA = ADCSRA;
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | (0 << ADIE) | PRESCALE128;
    for (int i = 0; i < 64; ++i) {
        // wait for free running conversion to finish
        while (bit_is_clear(ADCSRA, ADIF)) {
            ;
        }
        tUValue.byte.LowByte = ADCL;
        tUValue.byte.HighByte = ADCH;
        tSum += tUValue.Word;
    }
    ADCSRA = tOldADCSRA;
    ADMUX = tOldADMUX;

    tSum = (tSum + 32) >> 6;
    return tSum;
}

void setVCCValue(void) {
    float tVCC = getADCValue(ADC_1_1_VOLT_CHANNEL, DEFAULT);
    MeasurementControl.VCC = (1024 * 1.1) / tVCC;
}

float getTemperature(void) {
    float tTemp = (getADCValue(ADC_TEMPERATURE_CHANNEL, INTERNAL) - 317);
    return (tTemp / 1.22);
}

/*
 * setChannel() sets:
 * ShiftValue
 * ADCInputMUXChannel
 * ADCInputMUXChannelChar
 * ADCReference
 * tHasACDC
 * isACMode
 *
 * And by setInputRange():
 * OffsetValue
 * HorizontalGridSizeShift8
 * HorizontalGridVoltage
 *
 * And if attenuator detected also:
 * AttenuatorValue
 *
 * For active attenuator also:
 * ChannelHasAttenuator
 *  */
void setChannel(uint8_t aChannel) {
    MeasurementControl.ShiftValue = 2;
    if (aChannel == 16) {
        // do wrap around
        aChannel = 3;
        MeasurementControl.ADCInputMUXChannelChar = 0x33;
    }
    bool tIsACMode = false;
    uint8_t tHasACDC = false;
    uint8_t tReference = DEFAULT; // DEFAULT/1 -> VCC   INTERNAL/3 -> 1.1V

    if (MeasurementControl.AttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        if (aChannel <= MAX_ADC_CHANNEL_WITH_ACTIVE_ATTENUATOR) {
            MeasurementControl.ChannelHasActiveAttenuator = true;
            tHasACDC = true;
            // restore AC mode for this channels
            tIsACMode = MeasurementControl.storeForACMode;
            // use internal reference if attenuator is available
            tReference = INTERNAL;
        } else {
            MeasurementControl.ChannelHasActiveAttenuator = false;
            // protect input. Since ChannelHasActiveAttenuator = false it will not be changed by setInputRange()
            setAttenuator(3);
            // signal that no attenuator attached at channel
            MeasurementControl.AttenuatorValue = 0;
        }
    } else if (MeasurementControl.AttenuatorType == ATTENUATOR_TYPE_SIMPLE_ATTENUATOR) {
        if (aChannel <= MAX_ADC_CHANNEL_WITH_SIMPLE_ATTENUATOR) {
            MeasurementControl.AttenuatorValue = aChannel; // channel 0 has 10^0 attenuation factor etc.
            tHasACDC = true;
            // restore AC mode for this channels
            tIsACMode = MeasurementControl.storeForACMode;
            tReference = INTERNAL;
        }
    }
    MeasurementControl.isACMode = tIsACMode;
    MeasurementControl.ChannelHasACDC = tHasACDC;
    ADMUX = aChannel | (tReference << REFS0);

//the second parameter for active attenuator is only needed if ChannelHasActiveAttenuator == true
    setInputRange(2, 2);

//TODO Value on NANO V3 ?
    if (aChannel <= MAX_ADC_CHANNEL) {
// Standard AD channels
        MeasurementControl.ADCInputMUXChannelChar = 0x30 + aChannel;
    } else if (aChannel == MAX_ADC_CHANNEL + 1) {
        aChannel = ADC_TEMPERATURE_CHANNEL; // Temperature
        MeasurementControl.ADCInputMUXChannelChar = 'T';
    } else if (aChannel == ADC_TEMPERATURE_CHANNEL + 1) {
        aChannel = ADC_1_1_VOLT_CHANNEL; // 1.1 Reference
        MeasurementControl.ADCInputMUXChannelChar = 'R';
    } else if (aChannel == ADC_1_1_VOLT_CHANNEL + 1) {
        MeasurementControl.ADCInputMUXChannelChar = 'G'; // Ground
    }
    MeasurementControl.ADCInputMUXChannel = aChannel;

    if (aChannel >= ADC_DIRECT_CHANNEL_BUTTONS) {
// set channel number in caption
        ChannelSelectButtonString[CHANNEL_STRING_INDEX] = MeasurementControl.ADCInputMUXChannelChar;
        BlueDisplay1.setButtonCaption(TouchButtonChannelSelect, ChannelSelectButtonString, false);
    }

    /*
     * Refresh page if necessary
     */
    setReferenceButtonCaption();
    // check it here since it is also called by setup
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
        // manage AC/DC and auto offset buttons
        drawDSOSettingsPageGui();
    }

}

void setPrescaleFactor(uint8_t aFactor) {
    ADCSRA = (ADCSRA & ~0x07) | aFactor;
}

// DEFAULT/1 -> VCC   INTERNAL/3 -> 1.1V
void setReference(uint8_t aReference) {
    MeasurementControl.ADCReference = aReference;
    ADMUX = (ADMUX & ~0xC0) | (aReference << REFS0);
}

void initTimer2(void) {

    // initialization with 0 is essential otherwise timer will not work correctly!!!
    TCCR2A = 0; // set entire TCCR1A register to 0
    TCCR2B = 0; // same for TCCR1B

    TIMSK2 = 0; // no interrupts
    TCNT2 = 0; // init counter
    OCR2A = 125 - 1; // set compare match register for 1kHz

    TCCR2A = (1 << COM2A0| 1 << WGM21); // Toggle OC2A on compare match / CTC mode
    TCCR2B = (1 << CS20 | 1 << CS21); // Clock/32 => 4 us

}

#ifdef DEBUG
void printDebugData(void) {
    sprintf_P(StringBuffer, PSTR("%5d, 0x%04X, 0x%04X, 0x%04X"), DebugValue1, DebugValue2, DebugValue3, DebugValue4);
    BlueDisplay1.drawText(INFO_LEFT_MARGIN, INFO_UPPER_MARGIN + 2 * TEXT_SIZE_11_HEIGHT, StringBuffer, 11, COLOR_BLACK,
            COLOR_BACKGROUND_DSO);
}
#endif