/*
 * TouchLib.h
 *
 * @date 01.09.2014
 * @author Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmail.com
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 * @version 1.5.0
 */

#ifndef TOUCHLIB_H_
#define TOUCHLIB_H_

#ifndef DO_NOT_NEED_BASIC_TOUCH
//#define DO_NOT_NEED_BASIC_TOUCH // outcommenting or better defining for the compiler with -DDO_NOT_NEED_BASIC_TOUCH saves 620 bytes FLASH and 36 bytes RAM
#endif

#include "BlueDisplay.h"

#ifdef LOCAL_DISPLAY_EXISTS
#include "ADS7846.h"
extern ADS7846 TouchPanel;
#endif

#define BUTTON_DEFAULT_SPACING 16
#define BUTTON_DEFAULT_SPACING_HALF 8
#define BUTTON_DEFAULT_SPACING_QUARTER 4
/*
 * Layout for 320 x 240 screen size
 */
#define LAYOUT_320_WIDTH 320
#define LAYOUT_320_HEIGHT 240
/*
 * WIDTHS
 */
#define BUTTON_WIDTH_2 152 // for 2 buttons horizontal - 19 characters
#define BUTTON_WIDTH_2_POS_2 (BUTTON_WIDTH_2 + BUTTON_DEFAULT_SPACING)
//
#define BUTTON_WIDTH_3 96 // for 3 buttons horizontal - 12 characters
#define BUTTON_WIDTH_3_POS_2 (BUTTON_WIDTH_3 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_3_POS_3 (DISPLAY_WIDTH - BUTTON_WIDTH_3)
//
#define BUTTON_WIDTH_4 68 // for 4 buttons horizontal - 8 characters
#define BUTTON_WIDTH_4_POS_2 (BUTTON_WIDTH_4 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_4_POS_3 (2*(BUTTON_WIDTH_4 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_4_POS_4 (LAYOUT_320_WIDTH - BUTTON_WIDTH_4)
//
#define BUTTON_WIDTH_5 51 // for 5 buttons horizontal 51,2  - 6 characters
#define BUTTON_WIDTH_5_POS_2 (BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_5_POS_3 (2*(BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_5_POS_4 (3*(BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_5_POS_5 (LAYOUT_320_WIDTH - BUTTON_WIDTH_5)
//
#define BUTTON_WIDTH_2_5 120 //  for 2 buttons horizontal plus one small with BUTTON_WIDTH_5 (118,5)- 15 characters
#define BUTTON_WIDTH_2_5_POS_2   (BUTTON_WIDTH_2_5 + BUTTON_DEFAULT_SPACING -1)
#define BUTTON_WIDTH_2_5_POS_2_5 (LAYOUT_320_WIDTH - BUTTON_WIDTH_5)
//
#define BUTTON_WIDTH_6 40 // for 6 buttons horizontal
#define BUTTON_WIDTH_6_POS_2 (BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_6_POS_3 (2*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_4 (3*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_5 (4*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_6 (LAYOUT_320_WIDTH - BUTTON_WIDTH_6)
//
#define BUTTON_WIDTH_8 33 // for 8 buttons horizontal
#define BUTTON_WIDTH_10 28 // for 10 buttons horizontal
/*
 * HEIGHTS
 */
#define BUTTON_HEIGHT_4 48 // for 4 buttons vertical
#define BUTTON_HEIGHT_4_LINE_2 (BUTTON_HEIGHT_4 + BUTTON_DEFAULT_SPACING)
#define BUTTON_HEIGHT_4_LINE_3 (2*(BUTTON_HEIGHT_4 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_4_LINE_4 (DISPLAY_HEIGHT - BUTTON_HEIGHT_4)
//
#define BUTTON_HEIGHT_5 35 // for 5 buttons vertical
#define BUTTON_HEIGHT_5_LINE_2 (BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING)
#define BUTTON_HEIGHT_5_LINE_3 (2*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_5_LINE_4 (3*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_5_LINE_5 (DISPLAY_HEIGHT - BUTTON_HEIGHT_5)
//
#define BUTTON_HEIGHT_6 26 // for 6 buttons vertical 26,66..

//
#define TOUCH_SWIPE_THRESHOLD 10  // threshold for swipe detection to suppress long touch handler calling
#define TOUCH_SWIPE_RESOLUTION_MILLIS 20

struct XYPosition {
    uint16_t PosX;
    uint16_t PosY;
};

struct Swipe {
    bool SwipeMainDirectionIsX; // true if TouchDeltaXAbs >= TouchDeltaYAbs
    uint8_t Filler;
    uint16_t TouchStartX;
    uint16_t TouchStartY;
    int16_t TouchDeltaX;
    int16_t TouchDeltaY;
    uint16_t TouchDeltaAbsMax; // max of TouchDeltaXAbs and TouchDeltaYAbs to easily decide if swipe is large enough to be accepted
};

struct Callback {
    uint16_t ObjectIndex;
#if (FLASHEND > 65535)
    void * Handler;
#else
    void * Handler;
    void * Handler_upperWord; // not used on 16 bit address cpu
#endif
    union ValueForHandler {
        uint16_t Int16Value;
        uint32_t Int32Value;
        float FloatValue;
    } ValueForHandler;
};

struct BluetoothEvent {
    uint8_t EventType;
    union EventData {
        unsigned char ByteArray[10]; // To copy data from input buffer
        struct XYPosition TouchPosition;
        struct XYSize DisplaySize;
        struct Callback CallbackInfo;
        struct Swipe SwipeInfo;
    } EventData;
};

#ifdef LOCAL_DISPLAY_EXISTS
extern struct BluetoothEvent localTouchEvent;
/*
 * helper variables
 */
extern bool sButtonTouched;
extern bool sAutorepeatButtonTouched;
extern bool sSliderTouched;
extern bool sNothingTouched;
extern bool sSliderIsMoveTarget;
extern volatile bool sDisableTouchUpOnce; // set normally by application if long touch action was made

void resetTouchFlags(void);
#endif

extern struct BluetoothEvent remoteTouchEvent;

#ifndef DO_NOT_NEED_BASIC_TOUCH
extern struct BluetoothEvent remoteTouchDownEvent;
extern bool sTouchIsStillDown;
extern struct XYPosition sDownPosition;
extern struct XYPosition sActualPosition;
extern struct XYPosition sUpPosition;
#endif

// can be one of the following:
//see also android.view.MotionEvent
#define EVENT_TAG_TOUCH_ACTION_DOWN 0x00
#define EVENT_TAG_TOUCH_ACTION_UP   0x01
#define EVENT_TAG_TOUCH_ACTION_MOVE 0x02
#define EVENT_TAG_TOUCH_ACTION_ERROR 0xFF
#define EVENT_TAG_CONNECTION_BUILD_UP 0x10
#define EVENT_TAG_RESIZE_ACTION 0x11
// Must be below 0x20 since it only sends 4 bytes data
#define EVENT_TAG_LONG_TOUCH_DOWN_CALLBACK_ACTION  0x18

#define EVENT_TAG_FIRST_CALLBACK_ACTION_CODE 0x20
#define EVENT_TAG_BUTTON_CALLBACK_ACTION  0x20
#define EVENT_TAG_SLIDER_CALLBACK_ACTION  0x21
#define EVENT_TAG_SWIPE_CALLBACK_ACTION  0x22
#define EVENT_TAG_NUMBER_CALLBACK 0x28

#define EVENT_TAG_NO_EVENT 0xFF

void checkAndHandleEvents(void);

void registerPeriodicTouchCallback(bool (*aPeriodicTouchCallback)(int const, int const), const uint32_t aCallbackPeriodMillis);
void setPeriodicTouchCallbackPeriod(const uint32_t aCallbackPeriod);

void registerLongTouchDownCallback(void (*aLongTouchCallback)(struct XYPosition * const), const uint16_t aLongTouchTimeoutMillis);

void registerSwipeEndCallback(void (*aSwipeEndCallback)(struct Swipe * const));

void registerConnectCallback(void (*aConnectCallback)(struct XYSize * const aMaxSize));
void registerSimpleConnectCallback(void (*aConnectCallback)(void));

void registerResizeAndReconnectCallback(void (*aResizeAndReconnectCallback)(struct XYSize * const aActualSize));
void registerSimpleResizeAndReconnectCallback(void (*aSimpleResizeAndReconnectCallback)(void));

#ifndef DO_NOT_NEED_BASIC_TOUCH
void registerTouchDownCallback(void (*aTouchDownCallback)(struct XYPosition * const aActualPositionPtr));
void registerTouchMoveCallback(void (*aTouchMoveCallback)(struct XYPosition * const aActualPositionPtr));
void registerTouchUpCallback(void (*aTouchUpCallback)(struct XYPosition * const aActualPositionPtr));
#endif

#ifdef LOCAL_DISPLAY_EXISTS
void handleLocalTouchUp(void);
void simpleTouchDownHandler(struct XYPosition * const aActualPositionPtr);
void simpleTouchHandlerOnlyForButtons(struct XYPosition * const aActualPositionPtr);
void simpleTouchDownHandlerOnlyForSlider(struct XYPosition * const aActualPositionPtr);
void simpleTouchMoveHandlerForSlider(struct XYPosition * const aActualPositionPtr);
#endif

void handleEvent(struct BluetoothEvent * aEvent);

#endif /* TOUCHLIB_H_ */
