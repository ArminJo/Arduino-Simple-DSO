/*
 *      Created on: 29.03.2012
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
 *      Supports AC Measurement with (passive) external attenuator circuit.
 *      3 different types of external attenuator detected by software.
 *        - no attenuator (pin 8+9 left open).
 *        - passive attenuator with /1, /10, /100 attenuation (pin 8 connected to ground).
 *        - active attenuator (pin 9 connected to ground).
 *      Automatic trigger, range and offset value selection
 *      External trigger as well as delayed trigger possible
 *      1120 Byte data buffer - 3 * display width
 *      Min, max, average and peak to peak display
 *      Period and frequency display
 *      All settings can be changed during measurement
 *      Gesture (swipe) control of timebase and chart
 *      All AVR ADC input channels selectable
 *      Touch trigger level select
 *      1.1 Volt internal reference. 5 Volt (VCC) also usable.
 *
 *      The code can also be used as an example of C++/Assembler coding
 *      and complex interrupt handling routine
 *
 */

/*
 * SIMPLE EXTERNAL ATTENUATOR CIRCUIT (configured by connecting pin 8 to ground)
 *
 * Attenuator circuit and AC/DC switch
 *
 *          D2    ADC INPUT_0  1.1 Volt         ADC INPUT_1 11 Volt        ADC INPUT_2 110 Volt
 *          /\          /\                         /\     ______              /\     ______
 *          |           |                          +-----| 220k |----+        +-----| 10 k |----------+
 *          _           |      ______              |      ______     |        |      ------           |
 *         | |          +-----| >4 M |----+        +-----| 220k |----+        |      _____            |
 *         | | 100 k    |      ------     |        |      ------     |        +-----| 5 M |-+ 2*5 M or|
 *         | |          _                 |        _                 |        _      -----  _ 3*3.3 M |
 *          -          | |                |       | |                |       | |           | |        |
 *          |          | | 10 k           |       | | 1 M            |       | | 1 M       | | 5 M    |
 *          O          | |                |       | |                |       | |           | |        |
 *   External_Trigger   -                 |        -                 |        -             -         |
 *                      |                 |        |                 |        |             |         |
 *                      +----+            |        +----+            |        +----+        +----+    |
 *                      |    |            |        |    |            |        |    |        |    |    |
 *                      |    = C 0.1uF    |        |    = C 0.1uF    |        |    = 0.1uF  |    = 0.1uF  400 Volt
 *                      |    |            |        |    |            |        |    |        |    |    |
 *                      O    O            |        O    O            |        O    O        O    O    |
 *                     DC   AC            |       DC   AC            |       DC   AC       DC   AC    |
 *          D3                            |                          |                     1000 V Range
 *          /\           +----------------+--------------------------+--------------------------------+
 *          |            |
 *          O            O
 *         / _________  /   AC/DC            D8/Mode       D10/Frequency generator
 *        /            /    Switch             /\             \/
 * GND--O/    O      O/    O----------+        |              |
 *     AC     DC  AC |     DC         |        |              |
 *         ______    |       ______   |        |              |
 *   VREF-| 100k |---+------| 100k |--+        |              \/
 *         ------    |       ------   |        |
 *                   +--------||------+-GND----+
 *                          33 uF
 *
 */

/*
 * Attention: since 5.0 / 1024.0 = 0,004883 Volt is the resolution of the ADC, depending of scale factor:
 * 1. The output values for 2 adjacent display (min/max) values can be identical
 * 2. The voltage picker value may not reflect the real sample value (e.g. shown for min/max)
 */

/*
 * PIN
 * 2    External trigger input
 * 3    AC (low)/ DC (for attenuator)
 * 4    Attenuator range control (for active attenuator)
 * 5    Attenuator range control (for active attenuator)
 * 6    AC / DC relais (for active attenuator)
 * 7    AC / DC relais (for active attenuator)
 * 8    Attenuator detect input with internal pullup - bit 0
 * 9    Attenuator detect input with internal pullup - bit 1  11-> no attenuator attached, 10-> simple (channel 0-2) attenuator attached, 0x-> active (channel 0-1) attenuator attached
 * 10   Timer1 16 bit - Frequency generator output
 * 11   Timer2  8 bit - Square wave for VEE (-5V) generation
 * 12   Not yet used
 * 13   Internal LED
 *
 * Timer0  8 bit - Arduino delay() and millis() functions
 *
 */

/*
 * IMPORTANT - do not use Arduino serial.* here otherwise the usart interrupt kills the timing.
 * -flto gives 1300 Bytes for SimpleTouchScreenDSO -- but disables debug output at .lss file
 */

//#define DEBUG
#include <Arduino.h>

#include "SimpleTouchScreenDSO.h"
#include "PageFrequencyGenerator.h"

#include "BlueDisplay.h"
#include "utility/digitalWriteFast/digitalWriteFast.h"
#define digitalToggleFast(P) BIT_SET(* &PINB, __digitalPinToBit(P))

/**********************
 * Buttons
 *********************/
BDButton TouchButtonSettingsPage;
BDButton TouchButtonFrequencyPage;
BDButton TouchButtonBack;
// global flag for page control. Is evaluated by calling loop or page and set by buttonBack handler
bool sBackButtonPressed;

BDButton TouchButtonStartStop;

BDButton TouchButtonTriggerMode;
BDButton TouchButtonTriggerDelay;

const char TriggerModeButtonStringAuto[] PROGMEM = "Trigger auto";
const char TriggerModeButtonStringManualTimeout[] PROGMEM = "Trigger man t";
const char TriggerModeButtonStringManual[] PROGMEM = "Trigger man";
const char TriggerModeButtonStringFree[] PROGMEM = "Trigger free";
const char TriggerModeButtonStringExtern[] PROGMEM = "Trigger ext";

BDButton TouchButtonAutoOffsetOnOff;
const char AutoOffsetButtonStringAuto[] PROGMEM = "Offset auto";
const char AutoOffsetButtonString0[] PROGMEM = "Offset 0V";

BDButton TouchButtonAutoRangeOnOff;
const char AutoRangeButtonStringAuto[] PROGMEM = "Range auto";
const char AutoRangeButtonStringManual[] PROGMEM = "Range man";

BDButton TouchButtonADCReference;
const char ReferenceButtonVCC[] PROGMEM = "Ref VCC";
const char ReferenceButton1_1V[] PROGMEM = "Ref 1.1V";
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
//#define INTERNAL1V1 2
#undef INTERNAL
#define INTERNAL 2
#else
#define INTERNAL 3
#endif

BDButton TouchButtonAcDc;
const char AcDcButtonDC[] PROGMEM = "DC";
const char AcDcButtonAC[] PROGMEM = "AC";

BDButton TouchButtonChannels[NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR];
BDButton TouchButtonChannelSelect;
BDButton TouchButtonChannelMode;

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

BDButton TouchButtonBack_Singleshot;
#define SINGLESHOT_PPRINT_VALUE_X (37 * TEXT_SIZE_11_WIDTH)

BDButton TouchButtonChartHistoryOnOff;

/*
 * Slider for trigger level and voltage picker
 */
BDSlider TouchSliderTriggerLevel;

BDSlider TouchSliderVoltagePicker;
uint8_t LastPickerValue;





// for 31 grid
const uint16_t TimebaseDivPrintValues[TIMEBASE_NUMBER_OF_ENTRIES] PROGMEM = { 10, 20, 50, 101, 201, 496, 1, 2, 5, 10, 20, 50, 100,
        200, 500 };
// exact values for 31 grid - for period and frequency
const float TimebaseExactDivValuesMicros[TIMEBASE_NUMBER_OF_ENTRIES] PROGMEM
= { 100.75/*(31*13*0,25)*/, 201.5, 201.5, 201.5, 201.5 /*(31*13*0,5)*/, 496 /*(31*16*1)*/, 992 /*(31*16*2)*/, 1984 /*(31*16*4)*/,
        4960 /*(31*20*8)*/, 9920 /*(31*40*8)*/, 20088 /*(31*81*8)*/, 50096 /*(31*202*8)*/, 99944 /*(31*403*8)*/,
        199888 /*(31*806*8)*/, 499968 /*(31*2016*8)*/};
/*
 * For prescale 4 is: 13*0.25 = 3.25us per conversion
 * 8->6.5us, 16->13us ,32->2*13=26 for 1ms Range, 64->51, 128->8*13=104us per conversion
 *
 * Resolution of TimebaseDelayValues is 1/4 microsecond
 */
const uint16_t TimebaseDelayValues[TIMEBASE_NUMBER_OF_ENTRIES] = { 0, 0, 0, 0, 0, 3 - ISR_ZERO_DELAY_MICROS, //
        ((16 - ADC_CYCLES_PER_CONVERSION) * 2 * 4) - ISR_DELAY_MICROS_TIMES_4, // 6 us needed =5  | 2us ADC clock / 1ms Range
        ((16 - ADC_CYCLES_PER_CONVERSION) * 4 * 4) - ISR_DELAY_MICROS_TIMES_4, // =29 | 4us ADC clock / 2ms
        ((20 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, // 56 us delay needed = 205 | 8us ADC clock / 5ms
        ((40 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, // 216 us needed = 845 | 10ms
        ((81 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, // 544 us needed = 2157 | 20 ms
        ((202 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, //1512 us needed = 6029 | 50ms Range
        ((403 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, //
        ((806 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4, //
        (((uint16_t) (2016 - ADC_CYCLES_PER_CONVERSION) * 8 * 4) - ISR_DELAY_MICROS_TIMES_4), // 16025 us needed = 64077 | 500ms
        };
const uint8_t xScaleForTimebase[TIMEBASE_NUMBER_OF_XSCALE_CORRECTION] = { 10, 5, 2, 2 }; // for GUI and frequency
const uint8_t PrescaleValueforTimebase[TIMEBASE_NUMBER_OF_FAST_PRESCALE] = { PRESCALE4, PRESCALE4, PRESCALE4, PRESCALE8, PRESCALE8,
PRESCALE16 /*496us*/, PRESCALE32, PRESCALE64 /*2ms*/};

/****************************************
 * Automatic triggering and range stuff
 */
#define TRIGGER_WAIT_NUMBER_OF_SAMPLES 3300 // Number of samples (<=112us) used for detecting the trigger condition
#define TRIGGER_HYSTERESIS_MANUAL 2 // value for effective trigger hysteresis in manual trigger mode
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

// if max value > DISPLAY_HEIGHT the ValueShift is incremented
#define DISPLAY_VALUE_FOR_ZERO (REMOTE_DISPLAY_HEIGHT - 1)

/***********************
 *   Loop control
 ***********************/
// last sample of millis() in loop as reference for loop duration
uint32_t sMillisLastLoop = 0;
uint16_t sMillisSinceLastInfoOutput = 0;
#define MILLIS_BETWEEN_INFO_OUTPUT 1000

bool sShowWelcomeOnce;

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
char sStringBuffer[SIZEOF_STRINGBUFFER] __attribute__((section(".noinit")));
// safety net - array overflow will overwrite only DisplayBuffer
DataBufferStruct DataBufferControl __attribute__((section(".noinit")));

/*******************************************************************************************
 * Function declaration section
 *******************************************************************************************/
// Measurement section
void startAcquisition(void);
void acquireDataFast(void);

// Measurement auto control stuff (trigger, range + offset)
void computeAutoTrigger(void);
void setTriggerLevelAndHysteresis(int aRawTriggerValue, int aRawTriggerHysteresis);
void setInputRange(uint8_t aShiftValue, uint8_t aActiveAttenuatorValue);
bool changeRange(int8_t aChangeAmount);
bool checkRAWValuesForClippingAndChangeRange(void);
void computeAutoRange(void);
void computeAutoOffset(void);
void resetOffset(void);
void setOffsetAutomatic(bool aNewState);

// Attenuator support stuff
void setAttenuator(uint8_t aNewValue);
void setACMode(bool aNewMode);
uint16_t getAttenuatorFactor(void);

// GUI initialization and drawing stuff
void createGUI(void);
void drawStartGui(void);
void drawDSOSettingsPage(void);
void activatePartOfGui(void);


// GUI handler section
void doTouchUp(struct TouchEvent * const aTochPosition);
void doLongTouchDown(struct TouchEvent * const aTochPosition);
void doSwipeEnd(struct Swipe * const aSwipeInfo);

// BUTTON handler section
void doAcDcMode(BDButton * aTheTouchedButton, int16_t aValue);

void doPromptForTriggerDelay(BDButton * aTheTouchedButton, int16_t aValue);
void doAutoOffsetOnOff(BDButton * aTheTouchedButton, int16_t aValue);
void doADCReference(BDButton * aTheTouchedButton, int16_t aValue);
void doChannelSelect(BDButton * aTheTouchedButton, int16_t aValue);
void doTriggerSingleshot(BDButton * aTheTouchedButton, int16_t aValue);
void doStartStop(BDButton * aTheTouchedButton, int16_t aValue);
void doShowFrequencyPage(BDButton * aTheTouchedButton, int16_t aValue);
void doShowSettingsPage(BDButton * aTheTouchedButton, int16_t aValue);
void doTriggerLevel(BDSlider * aTheTouchedSlider, uint16_t aValue);
void doVoltagePicker(BDSlider * aTheTouchedSlider, uint16_t aValue);

// Button caption section
void setChannelButtonsCaption(void);
void setReferenceButtonCaption(void);
void setACModeButtonCaption(void);
void setTriggerDelayCaption(void);
void setAutoOffsetButtonCaption(void);
void setSlopeButtonCaption(void);

// Graphical output section
bool scrollChart(int8_t aScrollAmount);
void clearHorizontalGridLinesAndHorizontalLineLabels(void);
void drawMinMaxLines(void);
void drawDataBuffer(uint8_t *aByteBuffer, uint16_t aColor, uint16_t aClearBeforeColor);
void clearDisplayedChart(uint8_t * aDisplayBufferPtr);
void drawRemainingDataBufferValues(void);

// Text output section
void printVCCAndTemperature(void);
void clearInfo(uint8_t aOldMode);
void printInfo(void);
void printfTriggerDelay(char * aDataBufferPtr, uint16_t aTriggerDelayMillisOrMicros);

//Hardware support section
float getTemperature(void);
void setVCCValue(void);
void setChannel(uint8_t aChannel, bool doGui);
inline void setPrescaleFactor(uint8_t aFactor);
void setReference(uint8_t aReference);
void initTimer2(void);

// Stack info
void initStackFreeMeasurement(void);
uint16_t getStackFreeMinimumBytes(void);
void printFreeStack(void);

/*******************************************************************************************
 * Program code starts here
 * Setup section
 *******************************************************************************************/

void initDisplay(void) {
    // first synchronize. Since a complete chart data can be missing, send minimum 320 byte
    for (int i = 0; i < 16; ++i) {
        BlueDisplay1.sendSync();
    }
    BlueDisplay1.setFlagsAndSize(
            BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_LONG_TOUCH_ENABLE | BD_FLAG_ONLY_TOUCH_MOVE_DISABLE,
            REMOTE_DISPLAY_WIDTH, REMOTE_DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0xD1, 0x21D1); // Ascending in UTF16
    BlueDisplay1.setCharacterMapping(0xD2, 0x21D3); // Descending in UTF16
    //    BlueDisplay1.setCharacterMapping(0xF8, 0x2103); // Degree Celsius in UTF16
    BlueDisplay1.setButtonsTouchTone(TONE_PROP_BEEP, 80);
    createGUI();
    initFrequencyGeneratorPage();
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
        sShowWelcomeOnce = true;
    }
}

void setup() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__)
    pinMode(ATTENUATOR_0_PIN, OUTPUT);
    pinMode(ATTENUATOR_1_PIN, OUTPUT);
    pinMode(AC_DC_RELAIS_PIN_1, OUTPUT);
    pinMode(AC_DC_RELAIS_PIN_2, OUTPUT);
#else
    DDRD = (DDRD & ~CONTROL_MASK) | CONTROL_MASK;
#endif
    pinMode(AC_DC_PIN, INPUT_PULLUP);
    //    pinMode(TIMER_1_OUTPUT_PIN, OUTPUT);
    //    pinMode(VEE_PIN, OUTPUT);
    DDRB = (DDRB & ~OUTPUT_MASK_PORTB) | OUTPUT_MASK_PORTB;
    pinMode(ATTENUATOR_DETECT_PIN_0, INPUT_PULLUP);
    pinMode(ATTENUATOR_DETECT_PIN_1, INPUT_PULLUP);
#ifdef DEBUG
    pinMode(DEBUG_PIN, OUTPUT);
#endif
    MeasurementControl.ACModeFromISR = 2;
    // Enable pin change interrupt for AC/CD switch
    PCMSK2 = (1 << PCINT19);
    PCICR = (1 << PCIE2);

    // Shutdown SPI and TWI, enable all timers, USART and ADC
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__)
    PRR0 = (1 << PRTWI)| (1 << PRTWI);
#else
    PRR = (1 << PRTWI) | (1 << PRTWI);
#endif
    // Disable  digital input on all ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D | ADC4D | ADC5D;

    initSimpleSerial(HC_05_BAUD_RATE, false);

    // initialize values
    MeasurementControl.isRunning = false;
    MeasurementControl.TriggerSlopeRising = true;
    MeasurementControl.TriggerMode = TRIGGER_MODE_AUTOMATIC;
    MeasurementControl.isSingleShotMode = false;

    MeasurementControl.TimebaseIndex = 0;
    changeTimeBaseValue(TIMEBASE_INDEX_START_VALUE, false);
    /*
     * changeTimeBaseValue() needs:
     * TimebaseIndex
     * TriggerMode
     * changeTimeBaseValue() sets:
     * DrawWhileAcquire
     * TimebaseDelay
     * TimebaseDelayRemaining
     * TriggerTimeoutSampleCount
     * XScale
     */
    MeasurementControl.RawDSOReadingACZero = 0x200;

    MeasurementControl.ChannelHasActiveAttenuator = false;
    MeasurementControl.AttenuatorValue = 0; // set direct input as default
    uint8_t tAttenuatorType = !digitalReadFast(ATTENUATOR_DETECT_PIN_0);
    tAttenuatorType |= (!digitalReadFast(ATTENUATOR_DETECT_PIN_1)) << 1;
    MeasurementControl.AttenuatorType = tAttenuatorType;
    uint8_t tStartChannel = 0;
    if (tAttenuatorType == ATTENUATOR_TYPE_FIXED_ATTENUATOR) {
        tStartChannel = 1;
    } else if (tAttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        initTimer2(); // start timer2 for generating VEE (negative Voltage for external hardware)
    }

    delay(100);
    setVCCValue();

    setChannel(tStartChannel, false);
    /*
     * setChannel() needs:
     * AttenuatorType
     * isACMode
     * VCC
     *
     * setChannel() sets:
     * ShiftValue = 2
     * ADCInputMUXChannel
     * ADCInputMUXChannelChar
     *
     * And if AttenuatorDetected == true also:
     * ChannelHasAttenuator
     * isACModeANDChannelHasAttenuator
     * AttenuatorValue
     * ADCReference
     *
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

    setACMode(!digitalReadFast(AC_DC_PIN));
    initFrequencyGenerator();

    // Register callback handler and check for connection
    BlueDisplay1.initCommunication(&initDisplay, NULL, &redrawDisplay);
    registerSwipeEndCallback(&doSwipeEnd);
    registerTouchUpCallback(&doTouchUp);
    registerLongTouchDownCallback(&doLongTouchDown, 900);

    BlueDisplay1.playFeedbackTone(false);
    delay(400);
    setVCCValue();
    BlueDisplay1.playFeedbackTone(false);

    initStackFreeMeasurement();
}

/************************************************************************
 * main loop - 32 microseconds
 ************************************************************************/
// noreturn saves program space!
void __attribute__((noreturn)) loop(void) {
    uint32_t tMillis, tMillisOfLoop;
    bool sDoInfoOutput = true; // quasi static since we never leave the loop

    for (;;) {
        if (BlueDisplay1.mConnectionEstablished) {
            tMillis = millis();
            tMillisOfLoop = tMillis - sMillisLastLoop;
            sMillisLastLoop = tMillis;
            sMillisSinceLastInfoOutput += tMillisOfLoop;
            if (sMillisSinceLastInfoOutput > MILLIS_BETWEEN_INFO_OUTPUT) {
                sMillisSinceLastInfoOutput = 0;
                sDoInfoOutput = true;
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
                    if (sDoInfoOutput) {
                        sDoInfoOutput = false;
                        if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
                            drawGridLinesWithHorizLabelsAndTriggerLine();
                            if (DisplayControl.showInfoMode != INFO_MODE_NO_INFO) {
                                computePeriodFrequency(); // compute values only once
                                printInfo();
                            }
                        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
                            // refresh buttons
                            drawDSOSettingsPage();
                        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_FREQUENCY) {
                            // refresh buttons
                            drawFrequencyGeneratorPage();
#ifndef AVR
                        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_MORE_SETTINGS) {
                            // refresh buttons
                            drawDSOMoreSettingsPage();
#endif
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
                            BlueDisplay1.drawChar(INFO_LEFT_MARGIN + SINGLESHOT_PPRINT_VALUE_X,
                            INFO_UPPER_MARGIN + TEXT_SIZE_11_HEIGHT, ' ', TEXT_SIZE_11, COLOR_BLACK, COLOR_BACKGROUND_DSO);
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
                if (MeasurementControl.isSingleShotMode && MeasurementControl.TriggerStatus != TRIGGER_STATUS_FOUND
                        && sDoInfoOutput) {
                    sDoInfoOutput = false;
                    /*
                     * single shot - output actual values every second
                     */
                    MeasurementControl.ValueAverage = MeasurementControl.ValueBeforeTrigger;
                    MeasurementControl.PeriodMicros = 0;
                    MeasurementControl.PeriodFirst = 0;
                    MeasurementControl.PeriodSecond = 0;
                    printInfo();
                }
                /*
                 * Handle milliseconds delay - no timer overflow proof
                 */
                if (MeasurementControl.TriggerStatus == TRIGGER_STATUS_FOUND_AND_WAIT_FOR_DELAY) {
                    if (MeasurementControl.TriggerDelayMillisEnd == 0) {
                        // initialize value since this can not be efficiently done in ISR
                        MeasurementControl.TriggerDelayMillisEnd = millis() + MeasurementControl.TriggerDelayMillisOrMicros;
                    } else if (millis() > MeasurementControl.TriggerDelayMillisEnd) {
                        /*
                         * Start acquisition
                         */
                        MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND_AND_NOW_GET_ONE_VALUE;
                        if (MeasurementControl.TimebaseFastFreerunningMode) {
                            // NO Interrupt in FastMode
                            ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | MeasurementControl.TimebaseHWValue);
                        } else {
                            //  enable ADC interrupt, start with free running mode,
                            ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | PRESCALE16 | (1 << ADIE));
                        }
                    }
                }
            } else {
                // Analyze mode here
                if (sDoInfoOutput && DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
                    sDoInfoOutput = false;
                    /*
                     * show VCC and Temp
                     */
                    printVCCAndTemperature();
                }
            }

            /*
             * Handle Sub-Pages
             */
            if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
                if (sBackButtonPressed) {
                    sBackButtonPressed = false;
                    DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
                    redrawDisplay();
                }
            } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_FREQUENCY) {
                if (sBackButtonPressed) {
                    sBackButtonPressed = false;
                    stopFrequencyGeneratorPage();
                    DisplayControl.DisplayPage = DISPLAY_PAGE_SETTINGS;
                    redrawDisplay();
                } else {
                    //not needed here, because is contains only checkAndHandleEvents()
                    // loopFrequencyGeneratorPage();
                }
            }
        } // BlueDisplay1.mConnectionEstablished

        /*
         * handle AC / DC request from ISR
         */
        if (MeasurementControl.ACModeFromISR < 2) {
            setACMode(MeasurementControl.ACModeFromISR);
            MeasurementControl.ACModeFromISR = 2;
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
    DataBufferControl.AcquisitionSize = REMOTE_DISPLAY_WIDTH;
    DataBufferControl.DataBufferEndPointer = &DataBufferControl.DataBuffer[REMOTE_DISPLAY_WIDTH - 1];
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
    MeasurementControl.IntegrateValueForAverage = 0;
    DataBufferControl.DataBufferFull = false;
    /*
     * Timebase
     */
    uint8_t tTimebaseIndex = MeasurementControl.TimebaseIndex;
    if (tTimebaseIndex <= TIMEBASE_INDEX_FAST_MODES) {
        MeasurementControl.TimebaseFastFreerunningMode = true;
    } else {
        MeasurementControl.TimebaseFastFreerunningMode = false;
    }
    /*
     * get hardware prescale value
     */
    if (tTimebaseIndex < TIMEBASE_NUMBER_OF_FAST_PRESCALE) {
        MeasurementControl.TimebaseHWValue = PrescaleValueforTimebase[tTimebaseIndex];
    } else {
        MeasurementControl.TimebaseHWValue = PRESCALE_MAX_VALUE;
    }

    MeasurementControl.TriggerStatus = TRIGGER_STATUS_START;
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_EXTERN && !MeasurementControl.TimebaseFastFreerunningMode) {
        /*
         * wait for external trigger with INT1 pin change interrupt - NO timeout
         */
        if (MeasurementControl.TriggerSlopeRising) {
            EICRA = (1 << ISC01) | (1 << ISC00);
        } else {
            EICRA = (1 << ISC01);
        }

        // clear interrupt bit
        EIFR = (1 << INTF0);
        // enable interrupt on next change
        EIMSK = (1 << INT0);
        return;
    } else {
        // start with waiting for triggering condition
        MeasurementControl.TriggerSampleCountDividedBy256 = 0;
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
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | MeasurementControl.TimebaseHWValue);
    } else {
        //  enable ADC interrupt, start with fast free running mode for trigger search.
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | PRESCALE16 | (1 << ADIE));
    }
}

/*
 * ISR for external trigger input
 */
ISR(INT0_vect) {
    if (MeasurementControl.TriggerDelay != TRIGGER_DELAY_NONE) {
        /*
         * Delay
         */
        if (MeasurementControl.TriggerDelay == TRIGGER_DELAY_MICROS) {
            delayMicroseconds(MeasurementControl.TriggerDelayMillisOrMicros - TRIGGER_DELAY_MICROS_ISR_ADJUST_COUNT);
        } else {
            MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND_AND_WAIT_FOR_DELAY;
            MeasurementControl.TriggerDelayMillisEnd = millis() + MeasurementControl.TriggerDelayMillisOrMicros;
            return;
        }
    }

    /*
     * Start acquisition in free running mode as for trigger detection
     */
    MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND_AND_NOW_GET_ONE_VALUE;
    if (MeasurementControl.TimebaseFastFreerunningMode) {
        // NO Interrupt in FastMode
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | MeasurementControl.TimebaseHWValue);
    } else {
        //  enable ADC interrupt
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | PRESCALE16 | (1 << ADIE));
    }
    /*
     * Disable interrupt on trigger pin
     */
    EIMSK = 0;
}

/*
 * Fast ADC read routine for timebase <= 201us/div
 */
void acquireDataFast(void) {
    /**********************************
     * wait for triggering condition
     **********************************/
    uint16_t tUValue;
    uint8_t tTriggerStatus = TRIGGER_STATUS_START;
    uint16_t i;
    uint16_t tValueOffset = MeasurementControl.OffsetValue;

    if (MeasurementControl.TriggerMode == TRIGGER_MODE_EXTERN) {
        if (MeasurementControl.TriggerSlopeRising) {
            EICRA = (1 << ISC01) | (1 << ISC00);
        } else {
            EICRA = (1 << ISC01);
        }
        EIFR = (1 << INTF0);
        uint16_t tTimeoutCounter = 0;
        /*
         * Wait 65536 loops for external trigger to happen (for interrupt bit to be set)
         */
        do {
            tTimeoutCounter--;
        } while (bit_is_clear(EIFR, INTF0) && tTimeoutCounter != 0);

    } else {
        // start the first conversion and clear bit to recognize next conversion has finished
        ADCSRA |= (1 << ADIF) | (1 << ADSC);
        /*
         * Wait for trigger for max. 10 screens e.g. < 20 ms
         * if trigger condition not met it should run forever in single shot mode
         */
        for (i = TRIGGER_WAIT_NUMBER_OF_SAMPLES; i != 0 || MeasurementControl.isSingleShotMode; --i) {
            // wait for free running conversion to finish
            while (bit_is_clear(ADCSRA, ADIF)) {
                ;
            }
            // Get value
            tUValue = ADCL;
            tUValue |= ADCH << 8;
            // without "| (1 << ADSC)" it does not work - undocumented feature???
            ADCSRA |= (1 << ADIF) | (1 << ADSC); // clear bit to recognize next conversion has finished

            /*
             * detect trigger slope
             */
            if (MeasurementControl.TriggerSlopeRising) {
                if (tTriggerStatus == TRIGGER_STATUS_START) {
                    // rising slope - wait for value below 1. threshold
                    if (tUValue < MeasurementControl.RawTriggerLevelHysteresis) {
                        tTriggerStatus = TRIGGER_STATUS_BEFORE_THRESHOLD;
                    }
                } else {
                    // rising slope - wait for value to rise above 2. threshold
                    if (tUValue > MeasurementControl.RawTriggerLevel) {
                        break;
                    }
                }
            } else {
                if (tTriggerStatus == TRIGGER_STATUS_START) {
                    // falling slope - wait for value above 1. threshold
                    if (tUValue > MeasurementControl.RawTriggerLevelHysteresis) {
                        tTriggerStatus = TRIGGER_STATUS_BEFORE_THRESHOLD;
                    }
                } else {
                    // falling slope - wait for value to go below 2. threshold
                    if (tUValue < MeasurementControl.RawTriggerLevel) {
                        break;
                    }
                }
            }
        }
    }
    /*
     * Only microseconds delay makes sense
     */
    if (MeasurementControl.TriggerDelay == TRIGGER_DELAY_MICROS) {
        delayMicroseconds(MeasurementControl.TriggerDelayMillisOrMicros - TRIGGER_DELAY_MICROS_POLLING_ADJUST_COUNT);
    }
    /*
     * Get first value
     */
    ADCSRA |= (1 << ADIF) | (1 << ADSC);
    while (bit_is_clear(ADCSRA, ADIF)) {
        ;
    }
    tUValue = ADCL;
    tUValue |= ADCH << 8;
    // without "| (1 << ADSC)" it does not work - undocumented feature???
    ADCSRA |= (1 << ADIF) | (1 << ADSC); // clear bit to recognize next conversion has finished

    MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND; // for single shot mode

    /********************************
     * read a buffer of data here
     ********************************/
    // setup for min, max, average
    uint16_t tValueMax = tUValue;
    uint16_t tValueMin = tUValue;
    uint8_t tIndex = MeasurementControl.TimebaseIndex;
    uint16_t tLoopCount = DataBufferControl.AcquisitionSize;
    uint8_t *DataPointer = &DataBufferControl.DataBuffer[0];
    uint8_t *DataPointerFast = &DataBufferControl.DataBuffer[0];
    uint32_t tIntegrateValue = 0;

    cli();
    if (tIndex <= TIMEBASE_INDEX_ULTRAFAST_MODES) {
        if (DATABUFFER_SIZE / 2 < REMOTE_DISPLAY_WIDTH) {
            // Not enough space for 2 times DISPLAY_WIDTH 16 bit-values
            // I hope the compiler will remove the code ;-)
            tLoopCount = DATABUFFER_SIZE / 2;
        } else if (tLoopCount > REMOTE_DISPLAY_WIDTH) {
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
            tUValue = *DataPointerFast++;
            tUValue |= *DataPointerFast++ << 8;
        } else {
            // get values from adc
            // wait for free running conversion to finish
            // ADCSRA here is E5
            while (bit_is_clear(ADCSRA, ADIF)) {
                ;
            }
            // ADCSRA here is F5
            // duration: get Value included min 1,2 micros
            tUValue = ADCL;
            tUValue |= ADCH << 8;
            // without "| (1 << ADSC)" it does not work - undocumented feature???
            ADCSRA |= (1 << ADIF) | (1 << ADSC);            // clear bit to recognize next conversion has finished
            //ADCSRA here is E5
        }
        /*
         * process value
         */

        tIntegrateValue += tUValue;
        // get max and min for display and automatic triggering - needs 0,4 microseconds
        if (tUValue > tValueMax) {
            tValueMax = tUValue;
        } else if (tUValue < tValueMin) {
            tValueMin = tUValue;
        }

        /***************************************************
         * transform 10 bit value in order to fit on screen
         ***************************************************/
        if (tUValue < tValueOffset) {
            tUValue = 0;
        } else {
            tUValue -= tValueOffset;
        }
        tUValue = tUValue >> MeasurementControl.ShiftValue;
        // Byte overflow? This can happen if autorange is disabled.
        if (tUValue >= 0X100) {
            tUValue = 0xFF;
        }
        // now value is a byte and fits to screen
        *DataPointer++ = DISPLAY_VALUE_FOR_ZERO - tUValue;
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
    if (tIndex == 0 && tLoopCount > REMOTE_DISPLAY_WIDTH) {
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
 * 7 cycles before entering
 * 4 cycles RETI
 * ADC is NOT free running, except for trigger phase, where ADC also runs faster with prescaler 16.
 * Not free running is needed for inserting delays for exact timebase.
 */
ISR(ADC_vect) {
    // 7++ for jump to ISR
    // 3 + 8 Pushes + in + eor = 24 cycles
    // 31++ cycles to get here
    if (MeasurementControl.TimebaseDelay == 0) {
        // 6/7 for load TimebaseDelay and compare
        // Only entered for 496 us timebase (prescaler 16). Used for introducing 3 microseconds delay.
        // 5 cycles for set bit ADSC
        // gives 42 cycles < 48 cycles/3 micros
        // time passed must be between >2 and <3 micros (
        ADCSRA |= (1 << ADSC);
    }

    // 38++ cycles to get here
    Myword tUValue;
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;

    if (MeasurementControl.TriggerStatus != TRIGGER_STATUS_FOUND) {
        if (MeasurementControl.TriggerStatus != TRIGGER_STATUS_FOUND_AND_NOW_GET_ONE_VALUE) {
            bool tTriggerFound = false;
            /*
             * Trigger detection here
             */
            uint8_t tTriggerStatus = MeasurementControl.TriggerStatus;

            if (MeasurementControl.TriggerSlopeRising) {
                if (tTriggerStatus == TRIGGER_STATUS_START) {
                    // rising slope - wait for value below 1. threshold
                    if (tUValue.Word < MeasurementControl.RawTriggerLevelHysteresis) {
                        MeasurementControl.TriggerStatus = TRIGGER_STATUS_BEFORE_THRESHOLD;
                    }
                } else {
                    // rising slope - wait for value to rise above 2. threshold
                    if (tUValue.Word > MeasurementControl.RawTriggerLevel) {
                        // start reading into buffer
                        tTriggerFound = true;
                    }
                }
            } else {
                if (tTriggerStatus == TRIGGER_STATUS_START) {
                    // falling slope - wait for value above 1. threshold
                    if (tUValue.Word > MeasurementControl.RawTriggerLevelHysteresis) {
                        MeasurementControl.TriggerStatus = TRIGGER_STATUS_BEFORE_THRESHOLD;
                    }
                } else {
                    // falling slope - wait for value to go below 2. threshold
                    if (tUValue.Word < MeasurementControl.RawTriggerLevel) {
                        // start reading into buffer
                        tTriggerFound = true;
                    }
                }
            }

            if (!tTriggerFound) {
                if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL || MeasurementControl.isSingleShotMode
                        || MeasurementControl.TriggerDelay != TRIGGER_DELAY_NONE) {
                    // no timeout for Manual Trigger or SingleShotMode or trigger delay mode -> return
                    MeasurementControl.ValueBeforeTrigger = tUValue.Word;
                    return;
                }
                /*
                 * Trigger timeout handling. Always timeout in free running mode
                 */
                if (MeasurementControl.TriggerMode != TRIGGER_MODE_FREE) {
                    MeasurementControl.TriggerSampleCountPrecaler++;
                    if (MeasurementControl.TriggerSampleCountPrecaler != 0) {
                        return;
                    } else {
                        MeasurementControl.TriggerSampleCountDividedBy256++;
                        if (MeasurementControl.TriggerSampleCountDividedBy256 < MeasurementControl.TriggerTimeoutSampleCount) {
                            /*
                             * Trigger condition not met and timeout not reached
                             */
                            return;
                        }
                    }
                }
            } else {
                /*
                 * Trigger found, check for delay
                 */
                if (MeasurementControl.TriggerDelay != TRIGGER_DELAY_NONE) {
                    if (MeasurementControl.TriggerDelay == TRIGGER_DELAY_MICROS) {
                        ADCSRA &= ~(1 << ADIE); // disable ADC interrupt
//                        ADCSRA |= (1 << ADIF); // clear ADC interrupts
                        TIMSK0 &= ~(1 << TOIE0); // disable timer0 interrupt to avoid jitter
                        sei();
                        // enable RX interrupt for GUI, otherwise we could miss input bytes at long delays (eg.50000us)
                        uint16_t tDelayMicros = MeasurementControl.TriggerDelayMillisOrMicros;
                        // delayMicroseconds(tDelayMicros); substituted by code below, to avoid additional register pushes
                        __asm__ __volatile__ (
                                "1: sbiw %0,1" "\n\t" // 2 cycles
                                "1: adiw %0,1" "\n\t"// 2 cycles
                                "1: sbiw %0,1" "\n\t"// 2 cycles
                                "1: adiw %0,1" "\n\t"// 2 cycles
                                "1: sbiw %0,1" "\n\t"// 2 cycles
                                "1: adiw %0,1" "\n\t"// 2 cycles
                                "1: sbiw %0,1" "\n\t"// 2 cycles
                                "brne .-16" : : "w" (tDelayMicros)// 16 cycles
                        );

                        cli();
                        TIMSK0 |= (1 << TOIE0); // enable timer0 interrupt
                        ADCSRA |= (1 << ADIE); // enable ADC interrupt
                        // get a new value since ADC is already free running here
                        tUValue.byte.LowByte = ADCL;
                        tUValue.byte.HighByte = ADCH;
                    } else {
                        // needs additional register pushes
                        // MeasurementControl.TriggerDelayMillisEnd = millis() + MeasurementControl.TriggerDelayMillisOrMicros;
                        MeasurementControl.TriggerDelayMillisEnd = 0; // signal main loop to initialize value
                        MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND_AND_WAIT_FOR_DELAY;
                        ADCSRA &= ~(1 << ADIE); // disable ADC interrupt -> Main loop will handle delay
                        return;
                    }
                }
            }
        }
        // External Trigger or trigger found and delay passed -> reset trigger flag and initialize max and min
        MeasurementControl.TriggerStatus = TRIGGER_STATUS_FOUND;
        MeasurementControl.ValueMaxForISR = tUValue.Word;
        MeasurementControl.ValueMinForISR = tUValue.Word;
        // stop free running mode and set final prescaler value
        ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADIF) | MeasurementControl.TimebaseHWValue | (1 << ADIE));
    }
    /*
     * read buffer data
     */
    // take only last value for chart
    bool tUseValue = true;
    // 50++ cycles to get here
    // skip for 496 us timebase
    if (MeasurementControl.TimebaseIndex >= TIMEBASE_INDEX_MILLIS) {
        uint16_t tRemaining = MeasurementControl.TimebaseDelayRemaining;
        // 58++ cycles to get here
        if (tRemaining > (ADC_CONVERSION_AS_DELAY_MICROS * 4)) {
            // instead of busy wait just start a new conversion, which lasts 112 micros ( 13 + 1 (delay) clock cycles at 8 micros per cycle )
            tRemaining -= (ADC_CONVERSION_AS_DELAY_MICROS * 4);
            tUseValue = false;
        } else {
            /*
             *  busy wait for remaining micros for fast timebases
             */
            // 63++ cycles to get here
            // delayMicroseconds() needs additional register pushes and formula is not as simple as for assembler loop
            // here we have 1/4 microseconds resolution
            __asm__ __volatile__ (
                    "1: sbiw %0,1" "\n\t" // 2 cycles
                    "brne .-4" : : "w" (MeasurementControl.TimebaseDelayRemaining)// 2 cycles
            );
            // restore initial value
            tRemaining = MeasurementControl.TimebaseDelay; // 8 cycles
        }
        // 72++ cycles if (tRemaining > (ADC_CONVERSION_AS_DELAY_MICROS * 4)) == true
        // 73++ cycles + (4*(TimebaseDelayRemaining-1) +3) cycles => 76 cycles minimum
        // start the next conversion since in this mode adc is not free running
        ADCSRA |= (1 << ADSC); // 5 cycles
        MeasurementControl.TimebaseDelayRemaining = tRemaining;
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

/*
 * Pin change interrupt for AC/DC pin
 * Bouncing is around 250-1000 micro seconds
 */
ISR(PCINT2_vect) {
    delayMicroseconds(1000);
    if (digitalReadFast(AC_DC_PIN) == 0) {
        MeasurementControl.ACModeFromISR = 1;
    } else {
        MeasurementControl.ACModeFromISR = 0;
    }
}

/***********************************************************************
 * Measurement auto control stuff (trigger, range + offset)
 ***********************************************************************/
/*
 * Clears and sets grid and label too.
 * sets:
 * OffsetValue
 * HorizontalGridSizeShift8
 * HorizontalGridVoltage
 * needs:
 * VCC
 */
void setInputRange(uint8_t aShiftValue, uint8_t aActiveAttenuatorValue) {
    MeasurementControl.ShiftValue = aShiftValue;
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        setAttenuator(aActiveAttenuatorValue);
    }
    resetOffset();

    if (MeasurementControl.isRunning) {
        clearHorizontalGridLinesAndHorizontalLineLabels();
    }
    float tNewGridVoltage;
    uint16_t tHorizontalGridSizeShift8;

    if (MeasurementControl.ADCReference == DEFAULT) {
        /*
         * 5 Volt reference
         */
        if (aShiftValue == 0) {
            // formula for 5V and 0.2V / div and shift 0 is: ((1023/5V)*0.2) * 256/2^shift = 52377.6 / 5
            tHorizontalGridSizeShift8 = (52377.6 / MeasurementControl.VCC);
            tNewGridVoltage = 0.2;
        } else {
            // formula for 5V and 0.5V / div and shift 1 is: ((1023/5V)*0.5) * 256/2^shift = (130944/2) / 5
            tHorizontalGridSizeShift8 = (65472.0 / MeasurementControl.VCC);
            tNewGridVoltage = 0.5 * aShiftValue;
        }
    } else {
        /*
         * 1.1 Volt reference
         */
        if (MeasurementControl.ChannelHasActiveAttenuator) {
            tHorizontalGridSizeShift8 = HORIZONTAL_GRID_HEIGHT_2V_SHIFT8;
        } else {
            tHorizontalGridSizeShift8 = HORIZONTAL_GRID_HEIGHT_1_1V_SHIFT8;
        }
        uint8_t tFactor = 1 << aShiftValue;
        tNewGridVoltage = 0.05 * tFactor;
    }
    MeasurementControl.HorizontalGridSizeShift8 = tHorizontalGridSizeShift8;
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
    bool isError = false;
    int8_t tNewValue = 0;
    if (MeasurementControl.AttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        tNewValue = MeasurementControl.AttenuatorValue * 3;
    }
    tNewValue += MeasurementControl.ShiftValue + aChangeAmount;
    if (tNewValue < 0) {
        tNewValue = 0;
        isError = true;
    }
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        if (tNewValue > 8) {
            tNewValue = 8;
            isError = true;
        }
    } else {
        if (tNewValue > 2) {
            tNewValue = 2;
            isError = true;
        }
    }
    setInputRange(tNewValue % 3, tNewValue / 3);
    return isError;
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
                    || (MeasurementControl.ChannelIsACMode && MeasurementControl.RawValueMin == 0))
                    && MeasurementControl.AttenuatorValue < 2) {
                setInputRange(0, MeasurementControl.AttenuatorValue + 1);
                return true;
            }
        }
    }
    return false;
}

void computeAutoRange(void) {
    if (!MeasurementControl.RangeAutomatic) {
        return;
    }
    //First check for clipping - check ADC_MAX_CONVERSION_VALUE and also zero if AC mode
    if (checkRAWValuesForClippingAndChangeRange()) {
        return;
    }

    // get relevant peak2peak value
    int16_t tPeakToPeak = MeasurementControl.RawValueMax;
    if (MeasurementControl.ChannelIsACMode) {
        tPeakToPeak -= MeasurementControl.RawDSOReadingACZero;
        // check for zero offset error with tPeakToPeak < 0
        if (tPeakToPeak < 0
                || ((int16_t) MeasurementControl.RawDSOReadingACZero - (int16_t) MeasurementControl.RawValueMin) > tPeakToPeak) {
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
    // ignore warnings since we know that now tPeakToPeak is positive
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
    if (tPeakToPeak >= REMOTE_DISPLAY_HEIGHT * 2) {
        tNewValueShift = 2;
    } else if (tPeakToPeak >= REMOTE_DISPLAY_HEIGHT) {
#pragma GCC diagnostic pop
        tNewValueShift = 1;
    } else if (MeasurementControl.AttenuatorValue > 0 && MeasurementControl.AttenuatorType >= ATTENUATOR_TYPE_ACTIVE_ATTENUATOR) {
        /*
         * only max value is relevant for attenuator switching!
         */
        if (MeasurementControl.RawValueMax < ((REMOTE_DISPLAY_HEIGHT - (REMOTE_DISPLAY_HEIGHT / 10)) * 4) / ATTENUATOR_FACTOR) {
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

/**
 * compute offset based on center value in order display values at center of screen
 */
void computeAutoOffset(void) {
    if (!MeasurementControl.OffsetAutomatic) {
        return;
    }
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
    if (abs(MeasurementControl.OffsetGridCount - tNumberOfGridLinesToSkip) > 1) {
        // avoid jitter by not changing number if its delta is only 1
        clearHorizontalGridLinesAndHorizontalLineLabels();
        MeasurementControl.OffsetValue = tNumberOfGridLinesToSkip * tRawValuePerGrid;
        MeasurementControl.OffsetGridCount = tNumberOfGridLinesToSkip;
        drawGridLinesWithHorizLabelsAndTriggerLine();
    } else if (tNumberOfGridLinesToSkip == 0) {
        MeasurementControl.OffsetValue = 0;
        MeasurementControl.OffsetGridCount = 0;
    }
}

/*
 * sets offset and button caption
 */
void setOffsetAutomatic(bool aNewState) {
    if (!aNewState) {
        // disable auto offset
        MeasurementControl.OffsetValue = 0;
    }
    MeasurementControl.OffsetAutomatic = aNewState;
    setAutoOffsetButtonCaption();
}

/*
 * sets offset to right value for AC or DC mode
 */
void resetOffset(void) {
    if (MeasurementControl.ChannelIsACMode) {
        // Adjust zero offset for small display ranges
        uint16_t tNewValueOffsetForACMode = MeasurementControl.RawDSOReadingACZero / 2 + MeasurementControl.RawDSOReadingACZero / 4;
        if (MeasurementControl.ShiftValue == 1) {
            tNewValueOffsetForACMode = MeasurementControl.RawDSOReadingACZero / 2;
        } else if (MeasurementControl.ShiftValue == 2) {
            tNewValueOffsetForACMode = 0;
        }
        MeasurementControl.OffsetValue = tNewValueOffsetForACMode;
    } else if (!MeasurementControl.OffsetAutomatic) {
        MeasurementControl.OffsetValue = 0;
    }
}

/***********************************************************************
 * Attenuator support stuff
 ***********************************************************************/

void setAttenuator(uint8_t aNewValue) {
    MeasurementControl.AttenuatorValue = aNewValue;
    uint8_t tPortValue = CONTROL_PORT;
    tPortValue &= ~ATTENUATOR_MASK;
    tPortValue |= ((aNewValue << ATTENUATOR_SHIFT) & ATTENUATOR_MASK);
    CONTROL_PORT = tPortValue;
}

/*
 * No AC mode for channels without attenuator
 */
void setACMode(bool aNewMode) {
    if (MeasurementControl.isRunning) {
        clearHorizontalGridLinesAndHorizontalLineLabels();
    }
    MeasurementControl.isACMode = aNewMode;
    MeasurementControl.ChannelIsACMode = aNewMode;
    uint8_t tRelaisPin;
    if (aNewMode) {
        // no OffsetAutomatic for AC mode
        setOffsetAutomatic(false);
        tRelaisPin = AC_DC_RELAIS_PIN_1;
    } else {
        tRelaisPin = AC_DC_RELAIS_PIN_2;
    }
// change power latching relay state
    digitalWriteFast(tRelaisPin, HIGH);

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

// Wait for latching relay to switch - 2ms does not work reliable after reset, so take 4ms.
    delay(4);
// No need for relay power any more
    digitalWriteFast(tRelaisPin, LOW);
}

uint16_t getAttenuatorFactor(void) {
    uint16_t tRetValue = 1;
    if (MeasurementControl.ChannelHasACDCSwitch) {
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
    TouchButtonBack_Singleshot.initPGM(BUTTON_WIDTH_3_POS_3, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256,
    COLOR_GUI_CONTROL, PSTR("Single"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTriggerSingleshot);

    TouchButtonBack.initPGM(BUTTON_WIDTH_3_POS_3, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256, COLOR_GUI_CONTROL, PSTR("Back"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doDefaultBackButton);

// big start stop button
    TouchButtonStartStop.initPGM(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_256_LINE_2, BUTTON_WIDTH_3,
            (2 * BUTTON_HEIGHT_4_256) + BUTTON_DEFAULT_SPACING, COLOR_GUI_CONTROL, PSTR("Start/Stop"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStartStop);

// Button for settings page
    TouchButtonSettingsPage.initPGM(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_256_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4_256, COLOR_GUI_CONTROL, PSTR("Settings"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doShowSettingsPage);

    /*
     * Settings Page
     * 1. column
     */
// Button for chart history (erase color)
    TouchButtonChartHistoryOnOff.initPGM(0, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256, COLOR_RED, PSTR("History"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | BUTTON_FLAG_TYPE_TOGGLE_RED_GREEN_MANUAL_REFRESH, 0, &doChartHistory);

// Button for delay
    TouchButtonTriggerDelay.init(0, BUTTON_HEIGHT_4_256_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256,
    COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doPromptForTriggerDelay);
    setTriggerDelayCaption();

// Button for AC / DC
    TouchButtonAcDc.init(0, BUTTON_HEIGHT_4_256_LINE_3, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256,
    COLOR_GUI_SOURCE_TIMEBASE, "", TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doAcDcMode);
    setACModeButtonCaption();

// Button for selecting this page. To be displayed in parent page
    TouchButtonFrequencyPage.initPGM(0, BUTTON_HEIGHT_4_256_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256, COLOR_RED,
            PSTR("Frequency\nGenerator"), TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doShowFrequencyPage);

    /*
     * 2. column
     */
// Button for auto trigger on off
    TouchButtonTriggerMode.init(BUTTON_WIDTH_3_POS_2, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256,
    COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTriggerMode);
    setTriggerModeButtonCaption();

// Button for slope
    TouchButtonSlope.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_256_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4_256,
    COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTriggerSlope);
    setSlopeButtonCaption();

// Button for range
    TouchButtonAutoRangeOnOff.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_256_LINE_3, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4_256, COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doRangeMode);
    setAutoRangeModeAndButtonCaption(true);

// Button for auto offset on off
    TouchButtonAutoOffsetOnOff.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_256_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4_256, COLOR_GUI_TRIGGER, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doAutoOffsetOnOff);
    setAutoOffsetButtonCaption();

    /*
     * 3. column
     */
// Button for channel 0
    TouchButtonChannels[0].init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_256_LINE_2, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4_256, BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doChannelSelect);

// Button for channel 1
    TouchButtonChannels[1].init(REMOTE_DISPLAY_WIDTH - BUTTON_WIDTH_6, BUTTON_HEIGHT_4_256_LINE_2, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4_256, BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doChannelSelect);

// Button for channel 2
    TouchButtonChannels[2].init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_256_LINE_3, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4_256, BUTTON_AUTO_RED_GREEN_FALSE_COLOR, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 2, &doChannelSelect);
    setChannelButtonsCaption();

// Button for channel select
    TouchButtonChannelSelect.initPGM(REMOTE_DISPLAY_WIDTH - BUTTON_WIDTH_6, BUTTON_HEIGHT_4_256_LINE_3, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_4_256, BUTTON_AUTO_RED_GREEN_FALSE_COLOR, Channel3ButtonString, TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 42,
            &doChannelSelect);

// Button for reference voltage switching
    TouchButtonADCReference.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_256_LINE_4, BUTTON_WIDTH_3,
    BUTTON_HEIGHT_4_256, COLOR_GUI_SOURCE_TIMEBASE, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doADCReference);
    setReferenceButtonCaption();

    /*
     * SLIDER
     */
// make slider slightly visible
// slider for voltage picker
    TouchSliderVoltagePicker.init(SLIDER_VPICKER_POS_X, 0, SLIDER_SIZE, REMOTE_DISPLAY_HEIGHT, REMOTE_DISPLAY_HEIGHT, 0, 0,
    COLOR_DATA_PICKER_SLIDER, FLAG_SLIDER_VALUE_BY_CALLBACK, &doVoltagePicker);
    TouchSliderVoltagePicker.setBarBackgroundColor(COLOR_DATA_PICKER_SLIDER);

// slider for trigger level
    TouchSliderTriggerLevel.init(SLIDER_TLEVEL_POS_X, 0, SLIDER_SIZE, REMOTE_DISPLAY_HEIGHT, REMOTE_DISPLAY_HEIGHT, 0, 0,
    COLOR_TRIGGER_SLIDER, FLAG_SLIDER_VALUE_BY_CALLBACK, &doTriggerLevel);
    TouchSliderTriggerLevel.setBarBackgroundColor( COLOR_TRIGGER_SLIDER);
}

void drawStartGui(void) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_START;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    TouchButtonStartStop.drawButton();
    TouchButtonBack_Singleshot.drawButton();
    TouchButtonSettingsPage.drawButton();
    if (sShowWelcomeOnce) {
        BlueDisplay1.drawTextPGM(10, BUTTON_HEIGHT_4_LINE_2 + 32, PSTR("Welcome to\nArduino DSO"), 32, COLOR_BLUE,
        COLOR_BACKGROUND_DSO);
        BlueDisplay1.drawTextPGM(10, BUTTON_HEIGHT_4_LINE_2 + (3 * 32), PSTR("300 kSamples/s"), 22, COLOR_BLUE,
        COLOR_BACKGROUND_DSO);
        sShowWelcomeOnce = false;
    }
}

void drawDSOSettingsPage(void) {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);

    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    if (MeasurementControl.ChannelHasACDCSwitch) {
        TouchButtonAcDc.drawButton();
    }
    TouchButtonTriggerMode.drawButton();
    TouchButtonBack.drawButton();
    TouchButtonSlope.drawButton();
    TouchButtonTriggerDelay.drawButton();
    TouchButtonAutoRangeOnOff.drawButton();
    if (!MeasurementControl.ChannelIsACMode) {
        TouchButtonAutoOffsetOnOff.drawButton();
    }

    /*
     * Determine colors for channel buttons
     */
    int16_t tButtonColor;
    for (uint8_t i = 0; i < NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR; ++i) {
        if (i == MeasurementControl.ADCInputMUXChannel) {
            tButtonColor = BUTTON_AUTO_RED_GREEN_TRUE_COLOR;
        } else {
            tButtonColor = BUTTON_AUTO_RED_GREEN_FALSE_COLOR;
        }
        TouchButtonChannels[i].setButtonColorAndDraw(tButtonColor);
    }
    if (MeasurementControl.ADCInputMUXChannel >= NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR) {
        tButtonColor = BUTTON_AUTO_RED_GREEN_TRUE_COLOR;
    } else {
        tButtonColor = BUTTON_AUTO_RED_GREEN_FALSE_COLOR;
    }
    TouchButtonChannelSelect.setButtonColorAndDraw(tButtonColor);

    TouchButtonADCReference.drawButton();
    TouchButtonChartHistoryOnOff.drawButton();
    TouchButtonFrequencyPage.drawButton();
// print minimum stacksize and enable new stack measurement
    printFreeStack();
    initStackFreeMeasurement();
}

/*
 * activate elements if returning from settings screen or if starting acquisition
 */
void activatePartOfGui(void) {
// first deactivate all
    BlueDisplay1.deactivateAllButtons();

    TouchButtonStartStop.activate();
    TouchButtonBack_Singleshot.activate();
    TouchButtonSettingsPage.activate();
    TouchButtonChartHistoryOnOff.activate();

    TouchSliderVoltagePicker.drawSlider();
    if (MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL_TIMEOUT || MeasurementControl.TriggerMode == TRIGGER_MODE_MANUAL) {
        TouchSliderTriggerLevel.drawSlider();
    }
}

void redrawDisplay() {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    if (MeasurementControl.isRunning) {
        /*
         * running mode
         */
        if (DisplayControl.DisplayPage >= DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPage();
        } else {
            activatePartOfGui();
            // refresh grid
            drawGridLinesWithHorizLabelsAndTriggerLine();
            printInfo();
        }

    } else {
        /*
         * analyze mode
         */
        if (DisplayControl.DisplayPage == DISPLAY_PAGE_START) {
            drawStartGui();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
            activatePartOfGui();
            drawGridLinesWithHorizLabelsAndTriggerLine();
            drawMinMaxLines();
            // draw from last scroll position
            drawDataBuffer(DataBufferControl.DataBufferDisplayStart, COLOR_DATA_HOLD, DisplayControl.EraseColor);
            printInfo();
        } else if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            drawDSOSettingsPage();
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
void doTouchUp(struct TouchEvent * const aTochPosition) {
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
}

/*
 * If stopped toggle between Start and Chart page
 * if running and Chart page show Settings page
 */
void doLongTouchDown(struct TouchEvent * const aTochPosition) {
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        if (MeasurementControl.isRunning) {
            // Show settings page
            drawDSOSettingsPage();
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

void doSwipeEnd(struct Swipe * const aSwipeInfo) {
    bool isError = false;

    if (aSwipeInfo->TouchDeltaAbsMax > 32) {
        int8_t tTouchDeltaXGrid = aSwipeInfo->TouchDeltaX / 32;
        if (MeasurementControl.isRunning) {
            if (aSwipeInfo->SwipeMainDirectionIsX) {
                /*
                 * Timebase -> use tTouchDeltaXGrid/2
                 */
                tTouchDeltaXGrid /= 2;
                if (tTouchDeltaXGrid == 0) {
                    isError = true;
                } else {
                    isError = changeTimeBaseValue(-tTouchDeltaXGrid, true);
                }
            } else {
                if (!MeasurementControl.RangeAutomatic) {
                    /*
                     * Range
                     */
                    isError = changeRange(aSwipeInfo->TouchDeltaY / 32);
                } else {
                    isError = true;
                }
            }
        } else {
            if (aSwipeInfo->SwipeMainDirectionIsX) {
                //if (aSwipeInfo.TouchStartY > BUTTON_HEIGHT_4_256_LINE_3) {
                /*
                 * Scroll
                 */
                isError = scrollChart(-tTouchDeltaXGrid);
                //                } else {
                //                    /*
                //                     * X-Scale
                //                     */
                //                }
            } else {
                isError = true;
            }
        }
    } else {
        isError = true;
    }
    BlueDisplay1.playFeedbackTone(isError);
}

/************************************************************************
 * BUTTON handler section
 ************************************************************************/
/*
 * toggle between DC and AC mode
 */
void doAcDcMode(BDButton * aTheTouchedButton, int16_t aValue) {
    setACMode(!MeasurementControl.ChannelIsACMode);
}

/*
 * Handler for number receive event - set delay to value
 */
void doSetTriggerDelay(float aValue) {
    uint8_t tTriggerDelayMode = TRIGGER_DELAY_NONE;
    if (aValue != NUMBER_INITIAL_VALUE_DO_NOT_SHOW) {
        uint32_t tTriggerDelay = aValue;
        if (tTriggerDelay != 0) {
            if (tTriggerDelay > __UINT16_MAX__) {
                tTriggerDelayMode = TRIGGER_DELAY_MILLIS;
                MeasurementControl.TriggerDelayMillisOrMicros = tTriggerDelay / 1000;
            } else {
                tTriggerDelayMode = TRIGGER_DELAY_MICROS;
                uint16_t tTriggerDelayMicros = tTriggerDelay;
                if (tTriggerDelayMicros > TRIGGER_DELAY_MICROS_ISR_ADJUST_COUNT) {
                    MeasurementControl.TriggerDelayMillisOrMicros = tTriggerDelayMicros;
                } else {
                    // delay to small -> disable delay
                    tTriggerDelayMode = TRIGGER_DELAY_NONE;
                }
            }
        }
    }
    MeasurementControl.TriggerDelay = tTriggerDelayMode;
    setTriggerDelayCaption();
}

/*
 * Request delay value as number
 */
void doPromptForTriggerDelay(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPromptPGM(&doSetTriggerDelay, PSTR("Trigger delay [\xB5s]"), NUMBER_INITIAL_VALUE_DO_NOT_SHOW);
}

/*
 * toggle between auto and 0 Volt offset
 * No auto offset in AC Mode
 */
void doAutoOffsetOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    setOffsetAutomatic(!MeasurementControl.OffsetAutomatic);
}

/*
 * toggle between 5 and 1.1 Volt reference
 */
void doADCReference(BDButton * aTheTouchedButton, int16_t aValue) {
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
void doChannelSelect(BDButton * aTheTouchedButton, int16_t aValue) {
    if (aValue > 20) {
        // channel increment button here "Ch 3" etc.
        uint8_t tOldValue = MeasurementControl.ADCInputMUXChannel;
        // if channel 3 is not selected increment channel, otherwise select channel 3
        if (tOldValue >= NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR) {
            aValue = tOldValue + 1;
        } else {
            aValue = NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR;
        }
    }
    setChannel(aValue, true);
}

/*
 * set to singleshot mode and draw an indicating "S"
 */
void doTriggerSingleshot(BDButton * aTheTouchedButton, int16_t aValue) {
    MeasurementControl.isSingleShotMode = true;
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
    DisplayControl.DisplayPage = DISPLAY_PAGE_CHART;
    DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];

    drawGridLinesWithHorizLabelsAndTriggerLine();
// draw an S to indicate running single shot trigger
    BlueDisplay1.drawChar(INFO_LEFT_MARGIN + SINGLESHOT_PPRINT_VALUE_X, INFO_UPPER_MARGIN + TEXT_SIZE_11_HEIGHT, 'S', TEXT_SIZE_11,
    COLOR_BLACK, COLOR_INFO_BACKGROUND);

// prepare info output - at least 1 sec later
    sMillisSinceLastInfoOutput = 0;
    MeasurementControl.RawValueMax = 0;
    MeasurementControl.RawValueMin = 0;

// Start a new single shot
    startAcquisition();
    MeasurementControl.isRunning = true;
}

void doStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    if (MeasurementControl.isRunning) {
        /*
         * Stop here
         * for the last measurement read full buffer size
         * Do this asynchronously to the interrupt routine by "StopRequested" in order to extend a running or started acquisition.
         * Stopping does not need to release the trigger condition except for TRIGGER_MODE_EXTERN since trigger always has a timeout.
         * Stop single shot mode by switching to regular mode (and then waiting for timeout)
         */
        DataBufferControl.DataBufferEndPointer = &DataBufferControl.DataBuffer[DATABUFFER_SIZE - 1];

        /*
         * simulate an external trigger event for TRIGGER_MODE_EXTERN - use cli() to avoid race conditions
         */
        cli();
        if (MeasurementControl.TriggerMode == TRIGGER_MODE_EXTERN && MeasurementControl.TriggerStatus != TRIGGER_STATUS_FOUND) {
            INT0_vect();
        }
        sei();

        // in SingleShotMode Stop is always requested
        if (MeasurementControl.StopRequested && !MeasurementControl.isSingleShotMode) {
            /*
             * Stop requested 2 times -> stop immediately
             */
            uint8_t* tEndPointer = DataBufferControl.DataBufferNextInPointer;
            DataBufferControl.DataBufferEndPointer = tEndPointer;
            // clear trailing buffer space not used
            memset(tEndPointer, 0xFF, ((uint8_t*) &DataBufferControl.DataBuffer[DATABUFFER_SIZE]) - ((uint8_t*) tEndPointer));
        }
        // return to continuous mode with stop requested
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

// TODO we have a lot of duplicate code here and in printInfo()
void PrintTriggerInfo(void) {
    float tVoltage;
    float tRefMultiplier;
    if (MeasurementControl.ChannelHasActiveAttenuator) {
        tRefMultiplier = 2.0 / 1.1;
    } else {
        tRefMultiplier = 1;
    }
    if (MeasurementControl.ADCReference == DEFAULT) {
        tRefMultiplier *= MeasurementControl.VCC / 1024.0;
        /*
         * Use 1023 to get 5V display for full scale reading
         * Better would be 5.0 / 1024.0; since the reading for value for 5V- 1LSB is also 1023,
         * but this implies that the maximum displayed value is 4.99(51171875) :-(
         */
    } else {
        tRefMultiplier *= 1.1 / 1024.0;
    }
    tRefMultiplier *= getAttenuatorFactor();

    int16_t tACOffset = 0;
    if (MeasurementControl.ChannelIsACMode) {
        tACOffset = MeasurementControl.RawDSOReadingACZero;
    }
    tVoltage = ((int16_t) MeasurementControl.RawTriggerLevel - tACOffset);

    tVoltage = tRefMultiplier * tVoltage;
    uint8_t tPrecision = 2 - MeasurementControl.AttenuatorValue;
    dtostrf(tVoltage, 5, tPrecision, sStringBuffer);
    sStringBuffer[5] = 'V';
    sStringBuffer[6] = '\0';

    uint8_t tYPos = TRIGGER_LEVEL_INFO_SHORT_Y;
    uint16_t tXPos = TRIGGER_LEVEL_INFO_SHORT_X;
    uint8_t tFontsize = FONT_SIZE_INFO_SHORT;
    if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        tXPos = TRIGGER_LEVEL_INFO_LONG_X;
        tYPos = TRIGGER_LEVEL_INFO_LONG_Y;
        tFontsize = FONT_SIZE_INFO_LONG;
    }
    BlueDisplay1.drawText(tXPos, tYPos, sStringBuffer, tFontsize, COLOR_BLACK, COLOR_INFO_BACKGROUND);
}

/*
 * The value printed has a resolution of 0,00488 * scale factor
 */
void doTriggerLevel(BDSlider * aTheTouchedSlider, uint16_t aValue) {
// in auto-trigger mode show only computed value and do not modify it
    if (MeasurementControl.TriggerMode != TRIGGER_MODE_MANUAL_TIMEOUT && MeasurementControl.TriggerMode != TRIGGER_MODE_MANUAL) {
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
    setTriggerLevelAndHysteresis(tLevel, TRIGGER_HYSTERESIS_MANUAL);

// draw new line
    drawTriggerLine();
    PrintTriggerInfo();
}

/*
 * The value printed has a resolution of 0,00488 * scale factor
 */
void doVoltagePicker(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    char tVoltageBuffer[6];
    if (LastPickerValue == aValue) {
        return;
    }
    if (LastPickerValue != 0xFF) {
        // clear old line
        uint16_t tYpos = DISPLAY_VALUE_FOR_ZERO - LastPickerValue;
        BlueDisplay1.drawLineRel(0, tYpos, REMOTE_DISPLAY_WIDTH, 0, COLOR_BACKGROUND_DSO);
        // restore grid at old y position
        for (uint16_t tXPos = TIMING_GRID_WIDTH - 1; tXPos < REMOTE_DISPLAY_WIDTH - 1; tXPos += TIMING_GRID_WIDTH) {
            BlueDisplay1.drawPixel(tXPos, tYpos, COLOR_GRID_LINES);
        }
    }
// draw new line
    uint8_t tValue = DISPLAY_VALUE_FOR_ZERO - aValue;
    BlueDisplay1.drawLineRel(0, tValue, REMOTE_DISPLAY_WIDTH, 0, COLOR_DATA_PICKER);
    LastPickerValue = aValue;

    float tVoltage = getFloatFromDisplayValue(tValue);
    dtostrf(tVoltage, 4, 2, tVoltageBuffer);
    tVoltageBuffer[4] = 'V';
    tVoltageBuffer[5] = '\0';

    uint16_t tYPos = SLIDER_VPICKER_INFO_SHORT_Y;
    if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        tYPos = SLIDER_VPICKER_INFO_LONG_Y;
    }
// print value
    BlueDisplay1.drawText(SLIDER_VPICKER_INFO_X, tYPos, tVoltageBuffer, FONT_SIZE_INFO_SHORT, COLOR_BLACK,
    COLOR_INFO_BACKGROUND);
}

void doShowFrequencyPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_FREQUENCY;
    startFrequencyGeneratorPage();
}

void doShowSettingsPage(BDButton * aTheTouchedButton, int16_t aValue) {
    DisplayControl.DisplayPage = DISPLAY_PAGE_SETTINGS;
    drawDSOSettingsPage();
}

/*
 * default handle for back button
 */
void doDefaultBackButton(BDButton * aTheTouchedButton, int16_t aValue) {
    sBackButtonPressed = true;
}

/************************************************************************
 * Button caption section
 ************************************************************************/

void setChannelButtonsCaption(void) {
    for (uint8_t i = 0; i < NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR; ++i) {
        if (MeasurementControl.AttenuatorType == ATTENUATOR_TYPE_FIXED_ATTENUATOR) {
            TouchButtonChannels[i].setCaptionPGM(ChannelDivByButtonStrings[i],
                    (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
        } else {
            ChannelSelectButtonString[CHANNEL_STRING_INDEX] = 0x30 + i;
            TouchButtonChannels[i].setCaption(ChannelSelectButtonString, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
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
    TouchButtonADCReference.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setACModeButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.ChannelIsACMode) {
        tCaption = AcDcButtonAC;
    } else {
        tCaption = AcDcButtonDC;
    }
    TouchButtonAcDc.setCaptionPGM(tCaption); // do not draw, since complete page is drawn after setting
}

void setTriggerDelayCaption(void) {
    strcpy_P(&sStringBuffer[20], PSTR("Trigger delay\n"));
    if (MeasurementControl.TriggerDelay != TRIGGER_DELAY_NONE) {
        printfTriggerDelay(&sStringBuffer[34], MeasurementControl.TriggerDelayMillisOrMicros);
    }
    sprintf(sStringBuffer, &sStringBuffer[20], MeasurementControl.TriggerDelayMillisOrMicros);
    TouchButtonTriggerDelay.setCaption(sStringBuffer, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

void setAutoOffsetButtonCaption(void) {
    const char * tCaption;
    if (MeasurementControl.OffsetAutomatic) {
        tCaption = AutoOffsetButtonStringAuto;
    } else {
        tCaption = AutoOffsetButtonString0;
    }
    TouchButtonAutoOffsetOnOff.setCaptionPGM(tCaption, (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS));
}

/************************************************************************
 * Graphical output section
 ************************************************************************/
/*
 * returns false if display was scrolled
 */
bool scrollChart(int8_t aScrollAmount) {
    if (DisplayControl.DisplayPage != DISPLAY_PAGE_CHART) {
        return true;
    }
    bool isError = false;
    /*
     * set start of display in data buffer
     */
    DataBufferControl.DataBufferDisplayStart += aScrollAmount * TIMING_GRID_WIDTH / DisplayControl.XScale;
// check against begin of buffer
    if (DataBufferControl.DataBufferDisplayStart < &DataBufferControl.DataBuffer[0]) {
        DataBufferControl.DataBufferDisplayStart = &DataBufferControl.DataBuffer[0];
        isError = true;
    } else {
        uint8_t * tMaxAddress = &DataBufferControl.DataBuffer[DATABUFFER_SIZE];
        if (MeasurementControl.TimebaseIndex <= TIMEBASE_INDEX_FAST_MODES) {
            // Only half of data buffer is filled
            tMaxAddress = &DataBufferControl.DataBuffer[DATABUFFER_SIZE / 2];
        }
        tMaxAddress = tMaxAddress - (REMOTE_DISPLAY_WIDTH / DisplayControl.XScale);
        if (DataBufferControl.DataBufferDisplayStart > tMaxAddress) {
            DataBufferControl.DataBufferDisplayStart = tMaxAddress;
            isError = true;
        }
    }
    drawDataBuffer(DataBufferControl.DataBufferDisplayStart, COLOR_DATA_HOLD, COLOR_BACKGROUND_DSO);

    return isError;
}


void clearHorizontalGridLinesAndHorizontalLineLabels(void) {
    if (DisplayControl.DisplayPage == DISPLAY_PAGE_CHART) {
        BlueDisplay1.clearDisplay(COLOR_BACKGROUND_DSO);
        printInfo();
    }
}

/*
 * draws min, max lines
 */
void drawMinMaxLines(void) {
// draw max line
    uint8_t tValueDisplay = getDisplayFromRawValue(MeasurementControl.RawValueMax);
    if (tValueDisplay != 0) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, REMOTE_DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
    }
// min line
    tValueDisplay = getDisplayFromRawValue(MeasurementControl.RawValueMin);
    if (tValueDisplay != DISPLAY_VALUE_FOR_ZERO) {
        BlueDisplay1.drawLineRel(0, tValueDisplay, REMOTE_DISPLAY_WIDTH, 0, COLOR_MAX_MIN_LINE);
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

    while (DataBufferControl.DataBufferNextDrawPointer < DataBufferControl.DataBufferNextInPointer
            && tBufferIndex < REMOTE_DISPLAY_WIDTH) {
        /*
         * clear old line
         */
        if (tBufferIndex < REMOTE_DISPLAY_WIDTH - 1) {
            // fetch next value and clear line in advance
            tValueByte = DataBufferControl.DisplayBuffer[tBufferIndex];
            tNextValueByte = DataBufferControl.DisplayBuffer[tBufferIndex + 1];
            BlueDisplay1.drawLineFastOneX(tBufferIndex, tValueByte, tNextValueByte, DisplayControl.EraseColor);
        }

        /*
         * get new value
         */
        tValueByte = *DataBufferControl.DataBufferNextDrawPointer++;
        DataBufferControl.DisplayBuffer[tBufferIndex] = tValueByte;

        if (tBufferIndex != 0 && tBufferIndex <= REMOTE_DISPLAY_WIDTH - 1) {
            // get last value and draw line
            tLastValueByte = DataBufferControl.DisplayBuffer[tBufferIndex - 1];
            BlueDisplay1.drawLineFastOneX(tBufferIndex - 1, tLastValueByte, tValueByte, COLOR_DATA_RUN);
        }
        tBufferIndex++;
    }
    DataBufferControl.DataBufferNextDrawIndex = tBufferIndex;
}

/************************************************************************
 * Text output section
 ************************************************************************/
/*
 * Move 2 digits to front and set thousand separator
 */
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
void clearInfo(uint8_t aOldMode) {
// +1 because I have seen artifacts otherwise
    uint8_t tHeight = FONT_SIZE_INFO_SHORT + 1;
    if (aOldMode == INFO_MODE_LONG_INFO) {
        tHeight = (2 * FONT_SIZE_INFO_LONG) + 1;
    }
    BlueDisplay1.fillRectRel(INFO_LEFT_MARGIN, 0, REMOTE_DISPLAY_WIDTH, tHeight, COLOR_BACKGROUND_DSO);
}

/*
 * prints a period to a 9 character string and sets the thousand separator
 */
void printfMicrosPeriod(char * aDataBufferPtr, uint32_t aPeriod) {

    char tPeriodUnitChar;
    uint16_t tPeriod;
    if (aPeriod >= 50000l) {
        tPeriod = aPeriod / 1000;
        tPeriodUnitChar = 'm'; // milli
    } else {
        tPeriod = aPeriod;
        tPeriodUnitChar = '\xB5'; // micro
    }
    sprintf_P(aDataBufferPtr, PSTR("  %5u%cs"), tPeriod, tPeriodUnitChar);
    if (tPeriod >= 1000) {
        formatThousandSeparator(aDataBufferPtr + 3);
    }
}

void printfTriggerDelay(char * aDataBufferPtr, uint16_t aTriggerDelayMillisOrMicros) {

    char tPeriodUnitChar;
    if (MeasurementControl.TriggerDelay == TRIGGER_DELAY_MILLIS) {
        tPeriodUnitChar = 'm'; // milli
    } else {
        tPeriodUnitChar = '\xB5'; // micro
    }
    sprintf_P(aDataBufferPtr, PSTR(" %5u%cs"), aTriggerDelayMillisOrMicros, tPeriodUnitChar);
    if (aTriggerDelayMillisOrMicros >= 1000) {
        formatThousandSeparator(aDataBufferPtr + 2);
    }
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
    if (MeasurementControl.ChannelIsACMode) {
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

    tVoltage = ((int16_t) MeasurementControl.RawTriggerLevel - tACOffset);

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
    uint32_t tHertz = MeasurementControl.FrequencyHertz;

    if (DisplayControl.showInfoMode == INFO_MODE_LONG_INFO) {
        /*
         * Long version
         */
        sprintf_P(sStringBuffer, PSTR("%3u%cs %c Ch%c %s %s %s P2P%sV %sV %c"), tTimebaseUnitsPerGrid, tTimebaseUnitChar,
                tSlopeChar, MeasurementControl.ADCInputMUXChannelChar, tMinStringBuffer, tAverageStringBuffer, tMaxStringBuffer,
                tP2PStringBuffer, tTriggerStringBuffer, tReferenceChar);
        BlueDisplay1.drawText(INFO_LEFT_MARGIN, FONT_SIZE_INFO_LONG_ASC, sStringBuffer, FONT_SIZE_INFO_LONG, COLOR_BLACK,
        COLOR_INFO_BACKGROUND);

        /*
         * 2. line - timing
         */
        // 8 Character
        sprintf_P(sStringBuffer, PSTR(" %5luHz"), tHertz);
        if (tHertz >= 1000) {
            formatThousandSeparator(&sStringBuffer[2]);
        }
        /*
         * period and pulse values
         * Just write the fixed format strings to buffer, so that they are concatenated
         */
        printfMicrosPeriod(&sStringBuffer[8], MeasurementControl.PeriodMicros);
        printfMicrosPeriod(&sStringBuffer[17], MeasurementControl.PeriodFirst);
        printfMicrosPeriod(&sStringBuffer[26], MeasurementControl.PeriodSecond);

        /*
         * Delay - 14 character including leading space
         */
        if (MeasurementControl.TriggerDelay != TRIGGER_DELAY_NONE) {
            strcpy_P(&sStringBuffer[35], PSTR(" del "));
            printfTriggerDelay(&sStringBuffer[40], MeasurementControl.TriggerDelayMillisOrMicros);
        }

        BlueDisplay1.drawText(INFO_LEFT_MARGIN, FONT_SIZE_INFO_LONG_ASC + FONT_SIZE_INFO_LONG, sStringBuffer, FONT_SIZE_INFO_LONG,
        COLOR_BLACK, COLOR_INFO_BACKGROUND);

    } else {
        /*
         * Short version
         */
#ifdef LOCAL_DISPLAY_EXISTS
        snprintf(sStringBuffer, sizeof sStringBuffer, "%6.*fV %6.*fV%s%4u%cs", tPrecision,
                getFloatFromRawValue(MeasurementControl.RawValueAverage), tPrecision,
                getFloatFromRawValue(tValueDiff), tBufferForPeriodAndFrequency, tUnitsPerGrid, tTimebaseUnitChar);
#else
#ifdef AVR

        sprintf_P(sStringBuffer, PSTR("%sV %sV  %5luHz %3u%cs"), tAverageStringBuffer, tP2PStringBuffer, tHertz,
                tTimebaseUnitsPerGrid, tTimebaseUnitChar);
        if (tHertz >= 1000) {
            formatThousandSeparator(&sStringBuffer[15]);
        }

#else
        snprintf(sStringBuffer, sizeof sStringBuffer, "%6.*fV %6.*fV  %6luHz %4u%cs", tPrecision,
                getFloatFromRawValue(MeasurementControl.RawValueAverage), tPrecision,
                getFloatFromRawValue(tValueDiff), MeasurementControl.FrequencyHertz, tUnitsPerGrid,
                tTimebaseUnitChar);
        if (MeasurementControl.FrequencyHertz >= 1000) {
            // set separator for thousands
            formatThousandSeparator(&sStringBuffer[16], &sStringBuffer[19]);
        }
#endif
#endif
        BlueDisplay1.drawText(INFO_LEFT_MARGIN, FONT_SIZE_INFO_SHORT_ASC, sStringBuffer, FONT_SIZE_INFO_SHORT, COLOR_BLACK,
        COLOR_INFO_BACKGROUND);
    }

}

/*
 * Show temperature and VCC voltage
 */
void printVCCAndTemperature(void) {
    if (!MeasurementControl.isRunning) {
        setVCCValue();
        dtostrf(MeasurementControl.VCC, 4, 2, &sStringBuffer[30]);
        float tTemp = getTemperature();
        dtostrf(tTemp, 4, 1, &sStringBuffer[40]);
        sprintf_P(sStringBuffer, PSTR("%s Volt %s\xB0" "C"), &sStringBuffer[30], &sStringBuffer[40]);
        BlueDisplay1.drawText(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_256_LINE_4 - (TEXT_SIZE_11_DECEND + 3), sStringBuffer,
        TEXT_SIZE_11, COLOR_BLACK, COLOR_BACKGROUND_DSO);
    }
}

/************************************************************************
 * Utility section
 ************************************************************************/

/*
 * initialize RAM between actual stack and actual heap start (__brkval) with pattern 0x5A
 */
void initStackFreeMeasurement(void) {
    extern unsigned int __heap_start;
    extern void * __brkval;
    uint8_t v;

    uint8_t * tHeapPtr = (uint8_t *) __brkval;
    if (tHeapPtr == 0) {
        tHeapPtr = (uint8_t *) &__heap_start;
    }

// Fill memory
    do {
        *tHeapPtr++ = 0x5A;
    } while (tHeapPtr < &v);
}

/*
 * Check for untouched patterns
 */
uint16_t getStackFreeMinimumBytes(void) {
    extern unsigned int __heap_start;
    extern void * __brkval;
    uint8_t v;

    uint8_t * tHeapPtr = (uint8_t *) __brkval;
    if (tHeapPtr == 0) {
        tHeapPtr = (uint8_t *) &__heap_start;
    }

// first search for first match, because malloc() and free() may be happened in between
    while (*tHeapPtr != 0x5A && tHeapPtr < &v) {
        tHeapPtr++;
    }
// then count untouched patterns
    uint16_t tStackFree = 0;
    while (*tHeapPtr == 0x5A && tHeapPtr < &v) {
        tHeapPtr++;
        tStackFree++;
    }
// word -> bytes
    return (tStackFree);
}

/*
 * Show minimum free space on stack
 * Needs 260 byte of FLASH
 */
void printFreeStack(void) {
    uint16_t tUntouchesBytesOnStack = getStackFreeMinimumBytes();
    sprintf_P(sStringBuffer, PSTR("%4u Bytes stack"), tUntouchesBytesOnStack);
    BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_256_LINE_4 - (TEXT_SIZE_11_DECEND + 3), sStringBuffer,
    TEXT_SIZE_11, COLOR_BLACK, COLOR_BACKGROUND_DSO);
}

bool changeTimeBaseValue(int8_t aChangeValue, bool doOutput) {
    bool IsError = false;
    uint8_t tOldIndex = MeasurementControl.TimebaseIndex;

// positive value means increment timebase index!
    int8_t tNewIndex = tOldIndex + aChangeValue;
    if (tNewIndex < 0) {
        tNewIndex = 0;
        IsError = true;
    } else if (tNewIndex > TIMEBASE_NUMBER_OF_ENTRIES - 1) {
        tNewIndex = TIMEBASE_NUMBER_OF_ENTRIES - 1;
        IsError = true;
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

    /*
     * Set trigger timeout. Used only for trigger modes with timeout.
     */
    uint16_t tTriggerTimeoutSampleCount;
    /*
     * Try to have the time for showing one display as trigger timeout
     * Don't go below 1/10 of a second (30 * 256 samples) and above 3 seconds (900)
     * 10 ms Range => timeout = 100 millisecond
     * 20 ms => 200ms, 50 ms => 500ms, 100 ms => 1s, 200 ms => 2s, 500 ms => 3s
     * Trigger is always running with ADC at free running mode at 1 us clock => 13 us per conversion
     * which gives 77k samples for 1 second
     */
    if (tNewIndex <= 9) {
        tTriggerTimeoutSampleCount = 30;
    } else if (tNewIndex >= 14) {
        tTriggerTimeoutSampleCount = 900;
    } else {
        tTriggerTimeoutSampleCount = pgm_read_word(&TimebaseDivPrintValues[tNewIndex]) * 3;
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

    return IsError;
}

uint8_t getDisplayFromRawValue(uint16_t aRawValue) {
    aRawValue -= MeasurementControl.OffsetValue;
    aRawValue >>= MeasurementControl.ShiftValue;
    if (DISPLAY_VALUE_FOR_ZERO > aRawValue) {
        return (DISPLAY_VALUE_FOR_ZERO - aRawValue);
    } else {
        return 0;
    }
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
    if (MeasurementControl.ChannelIsACMode) {
        tRaw -= MeasurementControl.RawDSOReadingACZero;
    }
// cannot multiply tRaw with getAttenuatorFactor() before since it can lead to 16 bit overflow
    tFactor *= tRaw;
    tFactor *= getAttenuatorFactor();
    return tFactor;
}

/************************************************************************
 * Hardware support section
 ************************************************************************/

void setVCCValue(void) {
    float tVCC = getADCValue(ADC_1_1_VOLT_CHANNEL, DEFAULT);
    MeasurementControl.VCC = (1024 * 1.1) / tVCC;
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
 * needs:
 * VCC
 *
 * And if attenuator detected also:
 * AttenuatorValue
 *
 * For active attenuator also:
 * ChannelHasAttenuator
 *  */
void setChannel(uint8_t aChannel, bool doGUI) {
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
        if (aChannel < NUMBER_OF_CHANNEL_WITH_ACTIVE_ATTENUATOR) {
            MeasurementControl.ChannelHasActiveAttenuator = true;
            tHasACDC = true;
            // restore AC mode for this channels
            tIsACMode = MeasurementControl.isACMode;
            // use internal reference if attenuator is available
            tReference = INTERNAL;
        } else {
            MeasurementControl.ChannelHasActiveAttenuator = false;
            // protect input. Since ChannelHasActiveAttenuator = false it will not be changed by setInputRange()
            setAttenuator(3);
            // signal that no attenuator attached at channel
            MeasurementControl.AttenuatorValue = 0;
        }
    } else if (MeasurementControl.AttenuatorType == ATTENUATOR_TYPE_FIXED_ATTENUATOR) {
        if (aChannel < NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR) {
            MeasurementControl.AttenuatorValue = aChannel; // channel 0 has 10^0 attenuation factor etc.
            tHasACDC = true;
            // restore AC mode for this channels
            tIsACMode = MeasurementControl.isACMode;
            tReference = INTERNAL;
        }
    }
    MeasurementControl.ChannelIsACMode = tIsACMode;
    MeasurementControl.ChannelHasACDCSwitch = tHasACDC;
    MeasurementControl.ADCReference = tReference;
    ADMUX = aChannel | (tReference << REFS0);

//the second parameter for active attenuator is only needed if ChannelHasActiveAttenuator == true
    setInputRange(2, 2);

    if (aChannel < NUMBER_OF_CHANNEL_WITH_FIXED_ATTENUATOR) {
        //reset caption of TouchButtonChannelSelect to "Ch 3"
        MeasurementControl.ADCInputMUXChannelChar = '3';
    } else if (aChannel <= MAX_ADC_CHANNEL) {
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

// set channel number in caption
    ChannelSelectButtonString[CHANNEL_STRING_INDEX] = MeasurementControl.ADCInputMUXChannelChar;
    if (doGUI) {
        // do not touch gui before Setup is done
        TouchButtonChannelSelect.setCaption(ChannelSelectButtonString);

        /*
         * Refresh page if necessary
         */
        setReferenceButtonCaption();
        // check it here since it is also called by setup
        if (DisplayControl.DisplayPage == DISPLAY_PAGE_SETTINGS) {
            // manage AC/DC and auto offset buttons
            drawDSOSettingsPage();
        }
    }
}

inline void setPrescaleFactor(uint8_t aFactor) {
    ADCSRA = (ADCSRA & ~0x07) | aFactor;
}

// DEFAULT/1 -> VCC   INTERNAL/3 -> 1.1V
void setReference(uint8_t aReference) {
    MeasurementControl.ADCReference = aReference;
    ADMUX = (ADMUX & ~0xC0) | (aReference << REFS0);
}

/*
 * Square wave for VEE (-5V) generation
 */
void initTimer2(void) {

// initialization with 0 is essential otherwise timer will not work correctly!!!
    TCCR2A = 0; // set entire TCCR1A register to 0
    TCCR2B = 0; // same for TCCR1B

    TIMSK2 = 0; // no interrupts
    TCNT2 = 0; // init counter
    OCR2A = 125 - 1; // set compare match register for 1kHz

    TCCR2A = (1 << COM2A0 | 1 << WGM21); // Toggle OC2A on compare match / CTC mode
    TCCR2B = (1 << CS20 | 1 << CS21); // Clock/32 => 4 us

}

#ifdef DEBUG
void printDebugData(void) {
    sprintf_P(sStringBuffer, PSTR("%5d, 0x%04X, 0x%04X, 0x%04X"), DebugValue1, DebugValue2, DebugValue3, DebugValue4);
    BlueDisplay1.drawText(INFO_LEFT_MARGIN, INFO_UPPER_MARGIN + 2 * TEXT_SIZE_11_HEIGHT, sStringBuffer, 11, COLOR_BLACK,
            COLOR_BACKGROUND_DSO);
}
#endif

/*
 * For the common parts of AVR and ARM development
 */
//#include "TouchDSOCommon.cpp"
