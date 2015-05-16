/*
 * TouchLib.cpp
 *
 * @date 01.09.2014
 * @author Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmail.com
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 * @version 1.5.0
 */

#include <Arduino.h>
#include "TouchLib.h"
#include "BlueDisplay.h"
#include "BlueSerial.h"

#ifdef LOCAL_DISPLAY_EXISTS
#include "ADS7846.h"
ADS7846 TouchPanel;
#endif

#ifndef DO_NOT_NEED_BASIC_TOUCH
struct BluetoothEvent remoteTouchDownEvent; // to avoid overwriting of touch down events if CPU is busy and interrupt in not enabled
struct XYPosition sDownPosition;
struct XYPosition sActualPosition;
struct XYPosition sUpPosition;
#endif

#ifdef LOCAL_DISPLAY_EXISTS
/*
 * helper variables
 */bool sButtonTouched = false; // flag if autorepeat button was touched - to influence long button press handling
bool sAutorepeatButtonTouched = false;// flag if autorepeat button was touched - to influence long button press handling
bool sNothingTouched = false;// = !(sSliderTouched || sButtonTouched || sAutorepeatButtonTouched)
bool sSliderIsMoveTarget = false;// true if slider was touched by DOWN event

struct BluetoothEvent localTouchEvent;
#endif

bool sTouchIsStillDown = false;
bool sDisableTouchUpOnce = false;

struct BluetoothEvent remoteTouchEvent;

void (*sTouchDownCallback)(struct XYPosition * const) = NULL;
void (*sLongTouchDownCallback)(struct XYPosition * const) = NULL;
void (*sTouchMoveCallback)(struct XYPosition * const) = NULL;
void (*sTouchUpCallback)(struct XYPosition * const) = NULL;

void (*sSwipeEndCallback)(struct Swipe * const) = NULL;

void (*sConnectCallback)(struct XYSize * const) = NULL;
void (*sSimpleConnectCallback)(void) = NULL;
void (*sResizeAndReconnectCallback)(struct XYSize * const) = NULL;
void (*sSimpleResizeAndReconnectCallback)(void) = NULL;

void registerConnectCallback(void (*aConnectCallback)(struct XYSize * const aMaxSizePtr)) {
    sConnectCallback = aConnectCallback;
}

void registerSimpleConnectCallback(void (*aConnectCallback)(void)) {
    sSimpleConnectCallback = aConnectCallback;
}

void registerResizeAndReconnectCallback(void (*aResizeAndReconnectCallback)(struct XYSize * const aActualSizePtr)) {
    sResizeAndReconnectCallback = aResizeAndReconnectCallback;
}

void registerSimpleResizeAndReconnectCallback(void (*aSimpleResizeAndReconnectCallback)(void)) {
    sSimpleResizeAndReconnectCallback = aSimpleResizeAndReconnectCallback;
}

#ifndef DO_NOT_NEED_BASIC_TOUCH
void registerTouchDownCallback(void (*aTouchDownCallback)(struct XYPosition * const aActualPositionPtr)) {
    sTouchDownCallback = aTouchDownCallback;
}

void registerTouchMoveCallback(void (*aTouchMoveCallback)(struct XYPosition * const aActualPositionPtr)) {
    sTouchMoveCallback = aTouchMoveCallback;
}

void registerTouchUpCallback(void (*aTouchUpCallback)(struct XYPosition * const aActualPositionPtr)) {
    sTouchUpCallback = aTouchUpCallback;
}
#endif

/**
 * Register a callback routine which is only called after a timeout if screen is still touched
 */
void registerLongTouchDownCallback(void (*aLongTouchDownCallback)(struct XYPosition * const),
        const uint16_t aLongTouchDownTimeoutMillis) {
    sLongTouchDownCallback = aLongTouchDownCallback;
    BlueDisplay1.setLongTouchDownTimeout(aLongTouchDownTimeoutMillis);
}

/**
 * Register a callback routine which is called when touch goes up and swipe detected
 */
void registerSwipeEndCallback(void (*aSwipeEndCallback)(struct Swipe * const)) {
    sSwipeEndCallback = aSwipeEndCallback;
    // disable next end touch since we are already in a touch handler and don't want the end of this touch to be propagated
    if (sTouchIsStillDown) {
        sDisableTouchUpOnce = true;
    }
}

/**
 * Is called by thread main loops
 */
void checkAndHandleEvents(void) {
#ifdef USE_SIMPLE_SERIAL
#ifndef DO_NOT_NEED_BASIC_TOUCH
#ifdef LOCAL_DISPLAY_EXISTS
    resetTouchFlags();
    if (localTouchEvent.EventType != EVENT_TAG_NO_EVENT) {
        handleEvent(&localTouchEvent);
    }
#endif
    if (remoteTouchDownEvent.EventType != EVENT_TAG_NO_EVENT) {
        handleEvent(&remoteTouchDownEvent);
    }
#endif
    if (remoteTouchEvent.EventType != EVENT_TAG_NO_EVENT) {
        handleEvent(&remoteTouchEvent);
    }
#else
    serialEvent();
#endif
}

/**
 * Interprets the event type and manage the flags
 *
 * (Touch) Message has 7 bytes:
 * Gross message length in bytes
 * Function code
 * X Position LSB
 * X Position MSB
 * Y Position LSB
 * Y Position MSB
 * Sync Token
 */
void handleEvent(struct BluetoothEvent * aEvent) {
    uint8_t tEventType = aEvent->EventType;
    // avoid using event twice
    aEvent->EventType = EVENT_TAG_NO_EVENT;
#ifndef DO_NOT_NEED_BASIC_TOUCH
    if (tEventType == EVENT_TAG_TOUCH_ACTION_DOWN) {
        // must initialize all positions here!
        sDownPosition = aEvent->EventData.TouchPosition;
        sActualPosition = aEvent->EventData.TouchPosition;
        sTouchIsStillDown = true;
        if (sTouchDownCallback != NULL) {
            sTouchDownCallback(&aEvent->EventData.TouchPosition);
        }

    } else if (tEventType == EVENT_TAG_TOUCH_ACTION_MOVE) {
        if (sTouchMoveCallback != NULL) {
            sTouchMoveCallback(&aEvent->EventData.TouchPosition);
        }
        sActualPosition = aEvent->EventData.TouchPosition;

    } else if (tEventType == EVENT_TAG_TOUCH_ACTION_UP) {
        sUpPosition = aEvent->EventData.TouchPosition;
        sTouchIsStillDown = false;
#ifdef LOCAL_DISPLAY_EXISTS
        // may set sDisableTouchUpOnce
            handleLocalSwipeDetection();
#endif
            if (sDisableTouchUpOnce) {
                sDisableTouchUpOnce = false;
                return;
            }
            if (sTouchUpCallback != NULL) {
                sTouchUpCallback(&aEvent->EventData.TouchPosition);
            }

    } else if (tEventType == EVENT_TAG_TOUCH_ACTION_ERROR) {
        // try to reset touch state
        sUpPosition = aEvent->EventData.TouchPosition;
        sTouchIsStillDown = false;
    } else
#endif

    if (tEventType == EVENT_TAG_BUTTON_CALLBACK_ACTION) {
        sTouchIsStillDown = false; // to disable local touch up detection
        void (*tCallback)(uint16_t, int16_t) = (void (*)(uint16_t, int16_t)) aEvent->EventData.CallbackInfo.Handler;
        tCallback(aEvent->EventData.CallbackInfo.ObjectIndex, aEvent->EventData.CallbackInfo.ValueForHandler.Int16Value);

    } else if (tEventType == EVENT_TAG_SLIDER_CALLBACK_ACTION) {
        sTouchIsStillDown = false; // to disable local touch up detection
        void (*tCallback)(uint16_t, int16_t) = (void (*)(uint16_t, int16_t))aEvent->EventData.CallbackInfo.Handler;
        tCallback(aEvent->EventData.CallbackInfo.ObjectIndex, aEvent->EventData.CallbackInfo.ValueForHandler.Int16Value);

    } else if (tEventType == EVENT_TAG_NUMBER_CALLBACK) {
        void (*tCallback)(float) = (void (*)(float))aEvent->EventData.CallbackInfo.Handler;
        tCallback(aEvent->EventData.CallbackInfo.ValueForHandler.FloatValue);

    } else if (tEventType == EVENT_TAG_SWIPE_CALLBACK_ACTION) {
        sTouchIsStillDown = false;
        if (sSwipeEndCallback != NULL) {
            if (aEvent->EventData.SwipeInfo.SwipeMainDirectionIsX) {
                aEvent->EventData.SwipeInfo.TouchDeltaAbsMax = abs(aEvent->EventData.SwipeInfo.TouchDeltaX);
            } else {
                aEvent->EventData.SwipeInfo.TouchDeltaAbsMax = abs(aEvent->EventData.SwipeInfo.TouchDeltaY);
            }
            sSwipeEndCallback(&(aEvent->EventData.SwipeInfo));
        }

    } else if (tEventType == EVENT_TAG_LONG_TOUCH_DOWN_CALLBACK_ACTION) {
        if (sLongTouchDownCallback != NULL) {
            sLongTouchDownCallback(&(aEvent->EventData.TouchPosition));
        }
        sDisableTouchUpOnce = true;

    } else if (tEventType == EVENT_TAG_CONNECTION_BUILD_UP) {
        BlueDisplay1.setMaxDisplaySize(&aEvent->EventData.DisplaySize);
        if (sSimpleConnectCallback != NULL) {
            sSimpleConnectCallback();
        } else if (sConnectCallback != NULL) {
            sConnectCallback(&aEvent->EventData.DisplaySize);
        }
// also handle as resize
        tEventType = EVENT_TAG_RESIZE_ACTION;
    }
    if (tEventType == EVENT_TAG_RESIZE_ACTION) {
        BlueDisplay1.setActualDisplaySize(&aEvent->EventData.DisplaySize);
        if (sSimpleResizeAndReconnectCallback != NULL) {
            sSimpleResizeAndReconnectCallback();
        } else if (sResizeAndReconnectCallback != NULL) {
            sResizeAndReconnectCallback(&aEvent->EventData.DisplaySize);
        }
    }

}

#ifndef DO_NOT_NEED_BASIC_TOUCH
bool isTouchStillDown(void) {
    return sTouchIsStillDown;
}
#endif

#ifdef LOCAL_DISPLAY_EXISTS
void resetTouchFlags(void) {
    sButtonTouched = false;
    sAutorepeatButtonTouched = false;
    sNothingTouched = false;
}
/**
 * Called at Touch Up
 * Handle long callback delay and compute swipe info
 */
void handleLocalSwipeDetection(void) {
    if (sSwipeEndCallback != NULL && !sSliderIsMoveTarget) {
        if (abs(sDownPosition.PosX - sActualPosition.PosX) >= TOUCH_SWIPE_THRESHOLD
                || abs(sDownPosition.PosY - sActualPosition.PosY) >= TOUCH_SWIPE_THRESHOLD) {
            /*
             * Swipe recognized here
             * compute SWIPE data and call callback handler
             */
            struct Swipe tSwipeInfo;
            tSwipeInfo.TouchStartX = sDownPosition.PosX;
            tSwipeInfo.TouchStartY = sDownPosition.PosY;
            tSwipeInfo.TouchDeltaX = sUpPosition.PosX - sDownPosition.PosX;
            uint16_t tTouchDeltaXAbs = abs(tSwipeInfo.TouchDeltaX );
            tSwipeInfo.TouchDeltaY = sUpPosition.PosY - sDownPosition.PosY;
            uint16_t tTouchDeltaYAbs = abs(tSwipeInfo.TouchDeltaY );
            if (tTouchDeltaXAbs >= tTouchDeltaYAbs) {
                // X direction
                tSwipeInfo.SwipeMainDirectionIsX = true;
                tSwipeInfo.TouchDeltaAbsMax = tTouchDeltaXAbs;
            } else {
                tSwipeInfo.SwipeMainDirectionIsX = false;
                tSwipeInfo.TouchDeltaAbsMax = tTouchDeltaYAbs;
            }
            sSwipeEndCallback(&tSwipeInfo);
            sDisableTouchUpOnce = true;
        }
    }
    sSliderIsMoveTarget = false;
}

/**
 *
 * @param aActualPositionPtr
 * @return
 */
void simpleTouchDownHandler(struct XYPosition * const aActualPositionPtr) {
    if (TouchSlider::checkAllSliders(aActualPositionPtr->PosX, aActualPositionPtr->PosY)) {
        sSliderIsMoveTarget = true;
    } else {
        if (!TouchButton::checkAllButtons(aActualPositionPtr->PosX, aActualPositionPtr->PosY)) {
            sNothingTouched = true;
        }
    }
}

void simpleTouchHandlerOnlyForButtons(struct XYPosition * const aActualPositionPtr) {
    if (!TouchButton::checkAllButtons(aActualPositionPtr->PosX, aActualPositionPtr->PosY, true)) {
        sNothingTouched = true;
    }
}

void simpleTouchDownHandlerOnlyForSlider(struct XYPosition * const aActualPositionPtr) {
    if (TouchSlider::checkAllSliders(aActualPositionPtr->PosX, aActualPositionPtr->PosY)) {
        sSliderIsMoveTarget = true;
    } else {
        sNothingTouched = true;
    }
}

void simpleTouchMoveHandlerForSlider(struct XYPosition * const aActualPositionPtr) {
    if (sSliderIsMoveTarget) {
        TouchSlider::checkAllSliders(aActualPositionPtr->PosX, aActualPositionPtr->PosY);
    }
}
#endif
