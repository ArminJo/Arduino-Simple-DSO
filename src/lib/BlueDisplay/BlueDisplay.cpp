/*
 * BlueDisplay.cpp
 * C stub for Android BlueDisplay app and the local MI0283QT2 Display from Watterott.
 * It implements a few display test functions.
 *
 *  Created on: 12.09.2014
 * @author Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmail.com
 *      License: LGPL v3 (http://www.gnu.org/licenses/lgpl.html)
 * @version 1.0.0
 *
 */

#include <Arduino.h>
#include "BlueDisplay.h"

#include "BlueSerial.h"

/*
 * Internal functions
 */
const int FUNCTION_TAG_GLOBAL_SETTINGS = 0x08;
// Sub functions for GLOBAL_SETTINGS
const int SET_FLAGS_AND_SIZE = 0x00;
const int SET_CODEPAGE = 0x01;
const int SET_CHARACTER_CODE_MAPPING = 0x02;
const int SET_LONG_TOUCH_DOWN_TIMEOUT = 0x08;
// Sub functions for SET_FLAGS_AND_SIZE
const int BD_FLAG_FIRST_RESET_ALL = 0x01;
const int BD_FLAG_TOUCH_BASIC_DISABLE = 0x02;
const int BD_FLAG_TOUCH_MOVE_DISABLE = 0x04;
const int BD_FLAG_LONG_TOUCH_ENABLE = 0x08;
const int BD_FLAG_USE_MAX_SIZE = 0x10;

/*
 * Miscellaneous functions
 */
const int FUNCTION_TAG_GET_NUMBER = 0x0C;
const int FUNCTION_TAG_GET_TEXT = 0x0D;
const int FUNCTION_TAG_PLAY_TONE = 0x0E;

/*
 * Display functions
 */
const int FUNCTION_TAG_CLEAR_DISPLAY = 0x10;
const int FUNCTION_TAG_DRAW_DISPLAY = 0x11;
// 3 parameter
const int FUNCTION_TAG_DRAW_PIXEL = 0x14;
// 6 parameter
const int FUNCTION_TAG_DRAW_CHAR = 0x16;
// 5 parameter
const int FUNCTION_TAG_DRAW_LINE_REL = 0x20;
const int FUNCTION_TAG_DRAW_LINE = 0x21;
const int FUNCTION_TAG_DRAW_RECT_REL = 0x24;
const int FUNCTION_TAG_FILL_RECT_REL = 0x25;
const int FUNCTION_TAG_DRAW_RECT = 0x26;
const int FUNCTION_TAG_FILL_RECT = 0x27;

const int FUNCTION_TAG_DRAW_CIRCLE = 0x28;
const int FUNCTION_TAG_FILL_CIRCLE = 0x29;

const int LAST_FUNCTION_TAG_WITHOUT_DATA = 0x5F;

// Function with variable data size
const int FUNCTION_TAG_DRAW_STRING = 0x60;
const int FUNCTION_TAG_DEBUG_STRING = 0x61;

const int FUNCTION_TAG_GET_NUMBER_WITH_SHORT_PROMPT = 0x64;

const int FUNCTION_TAG_DRAW_PATH = 0x68;
const int FUNCTION_TAG_FILL_PATH = 0x69;
const int FUNCTION_TAG_DRAW_CHART = 0x6A;
const int FUNCTION_TAG_DRAW_CHART_WITHOUT_DIRECT_RENDERING = 0x6B;

/*
 * Button functions
 */
const int FUNCTION_TAG_BUTTON_DRAW = 0x40;
const int FUNCTION_TAG_BUTTON_DRAW_CAPTION = 0x41;
const int FUNCTION_TAG_BUTTON_SETTINGS = 0x42;
const int FUNCTION_TAG_BUTTON_SET_COLOR_AND_VALUE_AND_DRAW = 0x43;

// static functions
const int FUNCTION_TAG_BUTTON_ACTIVATE_ALL = 0x48;
const int FUNCTION_TAG_BUTTON_DEACTIVATE_ALL = 0x49;
const int FUNCTION_TAG_BUTTON_GLOBAL_SETTINGS = 0x4A;

// Function with variable data size
const int FUNCTION_TAG_BUTTON_CREATE = 0x70;
const int FUNCTION_TAG_BUTTON_CREATE_32 = 0x71;
const int FUNCTION_TAG_BUTTON_SET_CAPTION = 0x72;
const int FUNCTION_TAG_BUTTON_SET_CAPTION_AND_DRAW_BUTTON = 0x73;

// Flags for BUTTON_SETTINGS
const int BUTTON_FLAG_SET_COLOR_BUTTON = 0x00;
const int BUTTON_FLAG_SET_COLOR_BUTTON_AND_DRAW = 0x01;
const int BUTTON_FLAG_SET_COLOR_CAPTION = 0x02;
const int BUTTON_FLAG_SET_COLOR_CAPTION_AND_DRAW = 0x03;
const int BUTTON_FLAG_SET_VALUE = 0x04;
const int BUTTON_FLAG_SET_COLOR_AND_VALUE = 0x05;
const int BUTTON_FLAG_SET_COLOR_AND_VALUE_AND_DRAW = 0x06;
const int BUTTON_FLAG_SET_POSITION = 0x07;
const int BUTTON_FLAG_SET_ACTIVE = 0x10;
const int BUTTON_FLAG_RESET_ACTIVE = 0x11;

// Flags for BUTTON_GLOBAL_SETTINGS
const int USE_UP_EVENTS_FOR_BUTTONS = 0x01;

// Values for global button flag
const int BUTTON_FLAG_NO_BEEP_ON_TOUCH = 0x00;
const int BUTTON_FLAG_DO_BEEP_ON_TOUCH = 0x01;
const int BUTTONS_SET_BEEP_TONE = 0x02;
const int BUTTONS_SET_BEEP_TONE_VOLUME_FOR_NO_VOLUME_CHANGE = 0xFF;

// no valid button number
#define NO_BUTTON 0xFF
#define NO_SLIDER 0xFF

/*
 * Slider functions
 */
const int FUNCTION_TAG_SLIDER_CREATE = 0x50;
const int FUNCTION_TAG_SLIDER_DRAW = 0x51;
const int FUNCTION_TAG_SLIDER_SETTINGS = 0x52;
const int FUNCTION_TAG_SLIDER_DRAW_BORDER = 0x53;

// Flags for SLIDER_SETTINGS
const int SLIDER_FLAG_SET_COLOR_THRESHOLD = 0x00;
const int SLIDER_FLAG_SET_COLOR_BAR_BACKGROUND = 0x01;
const int SLIDER_FLAG_SET_COLOR_BAR = 0x02;
const int SLIDER_FLAG_SET_VALUE_AND_DRAW_BAR = 0x03;
const int SLIDER_FLAG_SET_POSITION = 0x04;
const int SLIDER_FLAG_SET_ACTIVE = 0x05;
const int SLIDER_FLAG_RESET_ACTIVE = 0x06;

// static slider functions
const int FUNCTION_TAG_SLIDER_ACTIVATE_ALL = 0x58;
const int FUNCTION_TAG_SLIDER_DEACTIVATE_ALL = 0x59;
const int FUNCTION_TAG_SLIDER_GLOBAL_SETTINGS = 0x5A;

// Flags for slider options
const int TOUCHSLIDER_SHOW_BORDER = 0x01;
const int TOUCHSLIDER_VALUE_BY_CALLBACK = 0x02; // if set value will be set by callback handler
const int TOUCHSLIDER_IS_HORIZONTAL = 0x04;

//-------------------- Constructor --------------------

BlueDisplay::BlueDisplay(void) {
    mReferenceDisplaySize.XWidth = DISPLAY_DEFAULT_WIDTH;
    mReferenceDisplaySize.YHeight = DISPLAY_DEFAULT_HEIGHT;
    return;
}

// One instance of BlueDisplay called BlueDisplay1
BlueDisplay BlueDisplay1;

void BlueDisplay::setFlagsAndSize(uint16_t aFlags, uint16_t aWidth, uint16_t aHeight) {
    mReferenceDisplaySize.XWidth = aWidth;
    mReferenceDisplaySize.YHeight = aHeight;
    if (USART_isBluetoothPaired()) {
        if (aFlags & BD_FLAG_FIRST_RESET_ALL) {
            // reset local buttons to be synchronized
            BlueDisplay1.resetAllButtons();
            BlueDisplay1.resetAllSliders();
        }
        sendUSARTArgs(FUNCTION_TAG_GLOBAL_SETTINGS, 4, SET_FLAGS_AND_SIZE, aFlags, aWidth, aHeight);
    }
}

/*
 * index is from android.media.ToneGenerator see also
 * http://www.syakazuka.com/Myself/android/sound_test.html
 */
void BlueDisplay::playTone(uint8_t aToneIndex) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_PLAY_TONE, 1, aToneIndex);
    }
}

void BlueDisplay::playTone(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_PLAY_TONE, 1, TONE_DEFAULT);
    }
}

void BlueDisplay::setCharacterMapping(uint8_t aChar, uint16_t aUnicodeChar) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_GLOBAL_SETTINGS, 3, SET_CHARACTER_CODE_MAPPING, aChar, aUnicodeChar);
    }
}

void BlueDisplay::setLongTouchDownTimeout(uint16_t aLongTouchTimeoutMillis) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_GLOBAL_SETTINGS, 2, SET_LONG_TOUCH_DOWN_TIMEOUT, aLongTouchTimeoutMillis);
    }
}

void BlueDisplay::clearDisplay(uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.clear(aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_CLEAR_DISPLAY, 1, aColor);
    }
}

// forces an rendering of the drawn bitmap
void BlueDisplay::drawDisplayDirect(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_DRAW_DISPLAY, 0);
    }
}

void BlueDisplay::drawPixel(uint16_t aXPos, uint16_t aYPos, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawPixel(aXPos, aYPos, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_DRAW_PIXEL, 3, aXPos, aYPos, aColor);
    }
}

void BlueDisplay::drawLine(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawLine(aXStart, aYStart, aXEnd, aYEnd, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_DRAW_LINE, aXStart, aYStart, aXEnd, aYEnd, aColor);
    }
}

void BlueDisplay::drawLineRel(uint16_t aXStart, uint16_t aYStart, uint16_t aXDelta, uint16_t aYDelta, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawLine(aXStart, aYStart, aXStart + aXDelta, aYStart + aYDelta, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_DRAW_LINE_REL, aXStart, aYStart, aXDelta, aYDelta, aColor);
    }
}

/**
 * Fast routine for drawing data charts
 * draws a line only from x to x+1
 * first pixel is omitted because it is drawn by preceding line
 * uses setArea instead if drawPixel to speed up drawing
 */
void BlueDisplay::drawLineOneX(uint16_t aXStart, uint16_t aYStart, uint16_t aYEnd, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawLineFastOneX(aXStart, aYStart, aYEnd, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_DRAW_LINE, aXStart, aYStart, aXStart + 1, aYEnd, aColor);
    }
}

void BlueDisplay::drawLineWithThickness(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, int16_t aThickness,
        uint8_t aThicknessMode, uint16_t aColor) {
    sendUSARTArgs(FUNCTION_TAG_DRAW_LINE, 6, aXStart, aYStart, aXEnd, aYEnd, aColor, aThickness);
}

void BlueDisplay::drawRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor,
        uint16_t aStrokeWidth) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawRect(aXStart, aYStart, aXEnd - 1, aYEnd - 1, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_DRAW_RECT, 6, aXStart, aYStart, aXEnd, aYEnd, aColor, aStrokeWidth);
    }
}

void BlueDisplay::drawRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, uint16_t aColor,
        uint16_t aStrokeWidth) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawRect(aXStart, aYStart, aXStart + aWidth - 1, aYStart + aHeight - 1, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_DRAW_RECT_REL, 6, aXStart, aYStart, aWidth, aHeight, aColor, aStrokeWidth);
    }
}

void BlueDisplay::fillRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.fillRect(aXStart, aYStart, aXEnd, aYEnd, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_FILL_RECT, aXStart, aYStart, aXEnd, aYEnd, aColor);
    }
}

void BlueDisplay::fillRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.fillRect(aXStart, aYStart, aXStart + aWidth - 1, aYStart + aHeight - 1, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_FILL_RECT_REL, aXStart, aYStart, aWidth, aHeight, aColor);
    }
}

void BlueDisplay::drawCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, uint16_t aColor, uint16_t aStrokeWidth) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.drawCircle(aXCenter, aYCenter, aRadius, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5Args(FUNCTION_TAG_DRAW_CIRCLE, aXCenter, aYCenter, aRadius, aColor, aStrokeWidth);
    }
}

void BlueDisplay::fillCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, uint16_t aColor) {
#ifdef LOCAL_DISPLAY_EXISTS
    LocalDisplay.fillCircle(aXCenter, aYCenter, aRadius, aColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_FILL_CIRCLE, 4, aXCenter, aYCenter, aRadius, aColor);
    }
}

/**
 * @return start x for next character / x + (TEXT_SIZE_11_WIDTH * size)
 */
uint16_t BlueDisplay::drawChar(uint16_t aPosX, uint16_t aPosY, char aChar, uint8_t aCharSize, uint16_t aFGColor,
        uint16_t aBGColor) {
    uint16_t tRetValue = 0;
#ifdef LOCAL_DISPLAY_EXISTS
    tRetValue = LocalDisplay.drawChar(aPosX, aPosY - getTextAscend(aCharSize), aChar, getLocalTextSize(aCharSize), aFGColor,
            aBGColor);
#endif
    if (USART_isBluetoothPaired()) {
        tRetValue = aPosX + getTextWidth(aCharSize);
        sendUSARTArgs(FUNCTION_TAG_DRAW_CHAR, 6, aPosX, aPosY, aCharSize, aFGColor, aBGColor, aChar);
    }
    return tRetValue;
}

/**
 * @param aXStart left position
 * @param aYStart upper position
 * @param aStringPtr is (const char *) to avoid endless casts for string constants
 * @return uint16_t start y for next line - next y Parameter
 */
uint16_t drawMLText(uint16_t aPosX, uint16_t aPosY, const char *aStringPtr, uint8_t aTextSize, uint16_t aColor, uint16_t aBGColor) {
    uint16_t tRetValue = 0;
#ifdef LOCAL_DISPLAY_EXISTS
    tRetValue = LocalDisplay.drawMLText(aPosX, aPosY - getTextAscend(aTextSize), (char *) aStringPtr, getLocalTextSize(aTextSize),
            aColor, aBGColor);
#endif
    if (USART_isBluetoothPaired()) {
        sendUSART5ArgsAndByteBuffer(FUNCTION_TAG_DRAW_STRING, aPosX, aPosY, aTextSize, aColor, aBGColor, strlen(aStringPtr),
                (uint8_t*) aStringPtr);
    }
    return tRetValue;
}

/**
 * @param aXStart left position
 * @param aYStart upper position
 * @param aStringPtr is (const char *) to avoid endless casts for string constants
 * @return uint16_t start x for next character - next x Parameter
 */
uint16_t BlueDisplay::drawText(uint16_t aPosX, uint16_t aPosY, const char *aStringPtr, uint8_t aTextSize, uint16_t aColor,
        uint16_t aBGColor) {
    uint16_t tRetValue = 0;
#ifdef LOCAL_DISPLAY_EXISTS
    tRetValue = LocalDisplay.drawText(aPosX, aPosY - getTextAscend(aTextSize), (char *) aStringPtr, getLocalTextSize(aTextSize),
            aColor, aBGColor);
#endif
    if (USART_isBluetoothPaired()) {
        tRetValue = aPosX + strlen(aStringPtr) * getTextWidth(aTextSize);
        sendUSART5ArgsAndByteBuffer(FUNCTION_TAG_DRAW_STRING, aPosX, aPosY, aTextSize, aColor, aBGColor, strlen(aStringPtr),
                (uint8_t*) aStringPtr);
    }
    return tRetValue;
}

/**
 * Output String as error log
 */
void BlueDisplay::debugMessage(const char *aStringPtr) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgsAndByteBuffer(FUNCTION_TAG_DEBUG_STRING, 0, strlen(aStringPtr), (uint8_t*) aStringPtr);
    }
}

uint16_t BlueDisplay::drawTextPGM(uint16_t aPosX, uint16_t aPosY, PGM_P aPGMString, uint8_t aTextSize, uint16_t aColor,
uint16_t aBGColor) {
    uint16_t tRetValue = 0;
    uint8_t tCaptionLengtht = strlen_P(aPGMString);
    if(tCaptionLengtht < STRING_BUFFER_STACK_SIZE) {
        char StringBuffer[STRING_BUFFER_STACK_SIZE];
        strcpy_P(StringBuffer, aPGMString);
#ifdef LOCAL_DISPLAY_EXISTS
        tRetValue = LocalDisplay.drawTextPGM(aPosX, aPosY - getTextAscend(aTextSize), aPGMString, getLocalTextSize(aTextSize), aColor, aBGColor);
#endif
        if (USART_isBluetoothPaired()) {
            tRetValue = aPosX + tCaptionLengtht * getTextWidth(aTextSize);
            sendUSART5ArgsAndByteBuffer(FUNCTION_TAG_DRAW_STRING, aPosX, aPosY, aTextSize, aColor, aBGColor, tCaptionLengtht,
            (uint8_t*) StringBuffer);
        }
    }
    return tRetValue;
}

/**
 * if aClearBeforeColor != 0 then previous line is cleared before
 */
void BlueDisplay::drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, uint16_t aColor, uint16_t aClearBeforeColor,
        uint8_t *aByteBuffer, uint16_t aByteBufferLength) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgsAndByteBuffer(FUNCTION_TAG_DRAW_CHART, 4, aXOffset, aYOffset, aColor, aClearBeforeColor, aByteBufferLength,
                aByteBuffer);
    }
}

/**
 * if aClearBeforeColor != 0 then previous line is cleared before
 * chart index is coded in the upper 4 bits of aYOffset
 */
void BlueDisplay::drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, uint16_t aColor, uint16_t aClearBeforeColor,
        uint8_t aChartIndex, bool aDoDrawDirect, uint8_t *aByteBuffer, uint16_t aByteBufferLength) {
    if (USART_isBluetoothPaired()) {
        aYOffset = aYOffset | ((aChartIndex & 0x0F) << 12);
        uint8_t tFunctionTag = FUNCTION_TAG_DRAW_CHART_WITHOUT_DIRECT_RENDERING;
        if (aDoDrawDirect) {
            tFunctionTag = FUNCTION_TAG_DRAW_CHART;
        }
        sendUSARTArgsAndByteBuffer(tFunctionTag, 4, aXOffset, aYOffset, aColor, aClearBeforeColor, aByteBufferLength, aByteBuffer);
    }
}

void BlueDisplay::setMaxDisplaySize(struct XYSize * const aMaxDisplaySizePtr) {
    mMaxDisplaySize.XWidth = aMaxDisplaySizePtr->XWidth;
    mMaxDisplaySize.YHeight = aMaxDisplaySizePtr->YHeight;
}

void BlueDisplay::setActualDisplaySize(struct XYSize * const aActualDisplaySizePrt) {
    mActualDisplaySize.XWidth = aActualDisplaySizePrt->XWidth;
    mActualDisplaySize.YHeight = aActualDisplaySizePrt->YHeight;
}

uint16_t BlueDisplay::getDisplayWidth(void) {
    return mReferenceDisplaySize.XWidth;
}

uint16_t BlueDisplay::getDisplayHeight(void) {
    return mReferenceDisplaySize.YHeight;
}

void BlueDisplay::getNumber(void (*aNumberHandler)(const float)) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_GET_NUMBER, 1, aNumberHandler);
    }
}

void BlueDisplay::getNumberWithShortPrompt(void (*aNumberHandler)(const float), const char *aShortPromptString) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgsAndByteBuffer(FUNCTION_TAG_GET_NUMBER_WITH_SHORT_PROMPT, 1, aNumberHandler, strlen(aShortPromptString),
                (uint8_t*) aShortPromptString);
    }
}

void BlueDisplay::getNumberWithShortPromptPGM(void (*aNumberHandler)(const float), const char *aPGMShortPromptString) {
    if (USART_isBluetoothPaired()) {
        uint8_t tShortPromptLengtht = strlen_P(aPGMShortPromptString);
        if (tShortPromptLengtht < STRING_BUFFER_STACK_SIZE) {
            char StringBuffer[STRING_BUFFER_STACK_SIZE];
            strcpy_P(StringBuffer, aPGMShortPromptString);
            sendUSARTArgsAndByteBuffer(FUNCTION_TAG_GET_NUMBER_WITH_SHORT_PROMPT, 1, aNumberHandler, tShortPromptLengtht,
                    (uint8_t*) StringBuffer);
        }
    }
}

void BlueDisplay::getText(void (*aTextHandler)(const char *)) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_GET_TEXT, 1, aTextHandler);
    }
}

/***************************************************************************************************************************************************
 *
 * BUTTONS
 *
 **************************************************************************************************************************************************/

uint8_t sButtonIndex = 0;
void BlueDisplay::resetAllButtons(void) {
    sButtonIndex = 0;
}

uint8_t BlueDisplay::createButton(const uint16_t aPositionX, const uint16_t aPositionY, const uint16_t aWidthX,
        const uint16_t aHeightY, const uint16_t aButtonColor, const char * aCaption, const uint8_t aCaptionSize,
        const uint8_t aFlags, const int16_t aValue, void (*aOnTouchHandler)(uint8_t, int16_t)) {
    uint8_t tButtonNumber = NO_BUTTON;
    if (USART_isBluetoothPaired()) {
        tButtonNumber = sButtonIndex++;
        sendUSARTArgsAndByteBuffer(FUNCTION_TAG_BUTTON_CREATE, 9, tButtonNumber, aPositionX, aPositionY, aWidthX, aHeightY,
                aButtonColor, aCaptionSize | (aFlags << 8), aValue, aOnTouchHandler, strlen(aCaption), aCaption);
    }
    return tButtonNumber;
}

uint8_t BlueDisplay::createButtonPGM(const uint16_t aPositionX, const uint16_t aPositionY, const uint16_t aWidthX,
        const uint16_t aHeightY, const uint16_t aButtonColor, PGM_P aPGMCaption, const uint8_t aCaptionSize, const uint8_t aFlags,
const int16_t aValue, void (*aOnTouchHandler)(uint8_t, int16_t)) {
    uint8_t tButtonNumber = NO_BUTTON;
    if (USART_isBluetoothPaired()) {
        uint8_t tCaptionLengtht = strlen_P(aPGMCaption);
        if(tCaptionLengtht < STRING_BUFFER_STACK_SIZE) {
            char StringBuffer[STRING_BUFFER_STACK_SIZE];
            strcpy_P(StringBuffer,aPGMCaption);
            tButtonNumber = sButtonIndex++;
            sendUSARTArgsAndByteBuffer(FUNCTION_TAG_BUTTON_CREATE, 9, tButtonNumber, aPositionX, aPositionY, aWidthX,aHeightY,
            aButtonColor, aCaptionSize | (aFlags << 8), aValue, aOnTouchHandler, tCaptionLengtht, StringBuffer);
        }
    }
    return tButtonNumber;
}

void BlueDisplay::drawButton(uint8_t aButtonNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_DRAW, 1, aButtonNumber);
    }
}

void BlueDisplay::drawButtonCaption(uint8_t aButtonNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_DRAW_CAPTION, 1, aButtonNumber);
    }
}

void BlueDisplay::setButtonCaption(uint8_t aButtonNumber, const char * aCaption, bool doDrawButton) {
    if (USART_isBluetoothPaired()) {
        uint8_t tFunctionCode = FUNCTION_TAG_BUTTON_SET_CAPTION;
        if (doDrawButton) {
            tFunctionCode = FUNCTION_TAG_BUTTON_SET_CAPTION_AND_DRAW_BUTTON;
        }
        sendUSARTArgsAndByteBuffer(tFunctionCode, 1, aButtonNumber, strlen(aCaption), aCaption);
    }
}

void BlueDisplay::setButtonCaptionPGM(uint8_t aButtonNumber, PGM_P aPGMCaption, bool doDrawButton) {
    if (USART_isBluetoothPaired()) {
        uint8_t tCaptionLengtht = strlen_P(aPGMCaption);
        if(tCaptionLengtht < STRING_BUFFER_STACK_SIZE) {
            char StringBuffer[STRING_BUFFER_STACK_SIZE];
            strcpy_P(StringBuffer,aPGMCaption);

            uint8_t tFunctionCode = FUNCTION_TAG_BUTTON_SET_CAPTION;
            if (doDrawButton) {
                tFunctionCode = FUNCTION_TAG_BUTTON_SET_CAPTION_AND_DRAW_BUTTON;
            }
            sendUSARTArgsAndByteBuffer(tFunctionCode, 1, aButtonNumber, tCaptionLengtht, StringBuffer);
        }
    }
}

void BlueDisplay::setButtonValue(uint8_t aButtonNumber, const int16_t aValue) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 3, aButtonNumber, BUTTON_FLAG_SET_VALUE, aValue);
    }
}

void BlueDisplay::setButtonColor(uint8_t aButtonNumber, const int16_t aButtonColor) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 3, aButtonNumber, BUTTON_FLAG_SET_COLOR_BUTTON, aButtonColor);
    }
}

void BlueDisplay::setButtonColorAndDraw(uint8_t aButtonNumber, const int16_t aButtonColor) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 3, aButtonNumber, BUTTON_FLAG_SET_COLOR_BUTTON_AND_DRAW, aButtonColor);
    }
}

/*
 * value 0 -> red, 1 -> green
 */
void BlueDisplay::setRedGreenButtonColor(uint8_t aButtonNumber, int16_t aValue, bool doDrawButton) {
    uint16_t tColor;
    if (aValue != 0) {
        // map to boolean
        aValue = true;
        tColor = COLOR_GREEN;
    } else {
        tColor = COLOR_RED;
    }
    if (USART_isBluetoothPaired()) {
        uint8_t tFunctionCode = BUTTON_FLAG_SET_COLOR_AND_VALUE;
        if (doDrawButton) {
            tFunctionCode = BUTTON_FLAG_SET_COLOR_AND_VALUE_AND_DRAW;
        }
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 4, aButtonNumber, tFunctionCode, tColor, aValue);
    }
}

void BlueDisplay::activateButton(uint8_t aButtonNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 2, aButtonNumber, BUTTON_FLAG_SET_ACTIVE);
    }
}

void BlueDisplay::deactivateButton(uint8_t aButtonNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_SETTINGS, 2, aButtonNumber, BUTTON_FLAG_RESET_ACTIVE);
    }
}

void BlueDisplay::setButtonsGlobalFlags(uint16_t aFlags) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_GLOBAL_SETTINGS, 1, aFlags);
    }
}

/*
 * aToneVolume: value in percent
 */
void BlueDisplay::setButtonsTouchTone(uint8_t aToneIndex, uint8_t aToneVolume) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_GLOBAL_SETTINGS, 3, BUTTONS_SET_BEEP_TONE, aToneIndex, aToneVolume);
    }
}

void BlueDisplay::activateAllButtons(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_ACTIVATE_ALL, 0);
    }
}

void BlueDisplay::deactivateAllButtons(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_BUTTON_DEACTIVATE_ALL, 0);
    }
}

/***************************************************************************************************************************************************
 *
 * SLIDER
 *
 **************************************************************************************************************************************************/

uint8_t sSliderIndex = 0;

void BlueDisplay::resetAllSliders(void) {
    sSliderIndex = 0;
}

/**
 * @brief initialization with all parameters except color
 * @param aPositionX determines upper left corner
 * @param aPositionY determines upper left corner
 * @param aBarWidth width of bar (and border) in pixel * #TOUCHSLIDER_OVERALL_SIZE_FACTOR
 * @param aBarLength size of slider bar in pixel
 * @param aThresholdValue value where color of bar changes from #TOUCHSLIDER_DEFAULT_BAR_COLOR to #TOUCHSLIDER_DEFAULT_BAR_THRESHOLD_COLOR
 * @param aInitalValue
 * @param aSliderColor
 * @param aBarColor
 * @param aOptions see #TOUCHSLIDER_SHOW_BORDER etc.
 * @param aCaption if NULL no caption is drawn
 * @param aOnChangeHandler - if NULL no update of bar is done on touch
 * @param aValueHandler Handler to convert actualValue to string - if NULL sprintf("%3d", mActualValue) is used
 *  * @return false if parameters are not consistent ie. are internally modified
 *  or if not enough space to draw caption or value.
 */
uint8_t BlueDisplay::createSlider(const uint16_t aPositionX, const uint16_t aPositionY, const uint8_t aBarWidth,
        const uint16_t aBarLength, const uint16_t aThresholdValue, const int16_t aInitalValue, const uint16_t aSliderColor,
        const uint16_t aBarColor, const uint8_t aOptions, void (*aOnChangeHandler)(const uint8_t, const int16_t)) {
    uint8_t tSliderNumber = NO_SLIDER;

    if (USART_isBluetoothPaired()) {
        tSliderNumber = sSliderIndex++;
        sendUSARTArgs(FUNCTION_TAG_SLIDER_CREATE, 11, tSliderNumber, aPositionX, aPositionY, aBarWidth, aBarLength, aThresholdValue,
                aInitalValue, aSliderColor, aBarColor, aOptions, aOnChangeHandler);
    }
    return tSliderNumber;
}

void BlueDisplay::drawSlider(uint8_t aSliderNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_DRAW, 1, aSliderNumber);
    }
}

void BlueDisplay::drawSliderBorder(uint8_t aSliderNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_DRAW_BORDER, 1, aSliderNumber);
    }
}

void BlueDisplay::setSliderActualValueAndDraw(uint8_t aSliderNumber, int16_t aActualValue) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_SETTINGS, 3, aSliderNumber, SLIDER_FLAG_SET_VALUE_AND_DRAW_BAR, aActualValue);
    }
}

void BlueDisplay::activateSlider(uint8_t aSliderNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_SETTINGS, 2, aSliderNumber, SLIDER_FLAG_SET_ACTIVE);
    }
}

void BlueDisplay::deactivateSlider(uint8_t aSliderNumber) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_SETTINGS, 2, aSliderNumber, SLIDER_FLAG_RESET_ACTIVE);
    }
}

void BlueDisplay::activateAllSliders(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_ACTIVATE_ALL, 0);
    }
}

void BlueDisplay::deactivateAllSliders(void) {
    if (USART_isBluetoothPaired()) {
        sendUSARTArgs(FUNCTION_TAG_SLIDER_DEACTIVATE_ALL, 0);
    }
}

/***************************************************************************************************************************************************
 *
 * Text sizes
 *
 **************************************************************************************************************************************************/

/*
 * Formula for Monospace Font on Android
 * TextSize * 0.6
 * Integer Formula: (TextSize *6)+4 / 10
 */
uint8_t getTextWidth(uint8_t aTextSize) {
    if (aTextSize == 11) {
        return TEXT_SIZE_11_WIDTH;
    }
#ifdef PGMSPACE_MATTERS
    return TEXT_SIZE_22_WIDTH;
#else
    if (aTextSize == 22) {
        return TEXT_SIZE_22_WIDTH;
    }
    return ((aTextSize * 6) + 4) / 10;
#endif
}

/*
 * Formula for Monospace Font on Android
 * float: TextSize * 0.76
 * int: (TextSize * 195 + 128) >> 8
 */
uint8_t getTextAscend(uint8_t aTextSize) {
    if (aTextSize == TEXT_SIZE_11) {
        return TEXT_SIZE_11_ASCEND;
    }
#ifdef PGMSPACE_MATTERS
    return TEXT_SIZE_22_ASCEND;
#else
    if (aTextSize == TEXT_SIZE_22) {
        return TEXT_SIZE_22_ASCEND;
    }
    uint16_t tRetvalue = aTextSize;
    tRetvalue = ((tRetvalue * 195) + 128) >> 8;
    return tRetvalue;
#endif
}

/*
 * Formula for Monospace Font on Android
 * float: TextSize * 0.24
 * int: (TextSize * 61 + 128) >> 8
 */
uint8_t getTextDecend(uint8_t aTextSize) {
    if (aTextSize == TEXT_SIZE_11) {
        return TEXT_SIZE_11_ASCEND;
    }
#ifdef PGMSPACE_MATTERS
    return TEXT_SIZE_22_ASCEND;
#else

    if (aTextSize == TEXT_SIZE_22) {
        return TEXT_SIZE_22_ASCEND;
    }
    uint16_t tRetvalue = aTextSize;
    tRetvalue = ((tRetvalue * 61) + 128) >> 8;
    return tRetvalue;
#endif
}
/*
 * Ascend - Decent
 * is used to position text in the middle of a button
 * Formula for positioning:
 * Position = ButtonTop + (ButtonHeight + getTextAscendMinusDescend())/2
 */
uint16_t getTextAscendMinusDescend(uint8_t aTextSize) {
    if (aTextSize == TEXT_SIZE_11) {
        return TEXT_SIZE_11_ASCEND - TEXT_SIZE_11_DECEND;
    }
#ifdef PGMSPACE_MATTERS
    return TEXT_SIZE_22_ASCEND - TEXT_SIZE_22_DECEND;
#else
    if (aTextSize == TEXT_SIZE_22) {
        return TEXT_SIZE_22_ASCEND - TEXT_SIZE_22_DECEND;
    }
    uint16_t tRetvalue = aTextSize;
    tRetvalue = ((tRetvalue * 133) + 128) >> 8;
    return tRetvalue;
#endif
}

/*
 * (Ascend -Decent)/2
 */
uint8_t getTextMiddle(uint8_t aTextSize) {
    if (aTextSize == TEXT_SIZE_11) {
        return (TEXT_SIZE_11_ASCEND - TEXT_SIZE_11_DECEND) / 2;
    }
#ifdef PGMSPACE_MATTERS
    return (TEXT_SIZE_22_ASCEND - TEXT_SIZE_22_DECEND) / 2;
#else
    if (aTextSize == TEXT_SIZE_22) {
        return (TEXT_SIZE_22_ASCEND - TEXT_SIZE_22_DECEND) / 2;
    }
    uint16_t tRetvalue = aTextSize;
    tRetvalue = ((tRetvalue * 66) + 128) >> 8;
    return tRetvalue;
#endif
}

/*
 * fast divide by 11 for MI0283QT2 driver arguments
 */
uint8_t getLocalTextSize(uint8_t aTextSize) {
    if (aTextSize <= 11) {
        return 1;
    }
#ifdef PGMSPACE_MATTERS
    return 2;
#else
    if (aTextSize == 22) {
        return 2;
    }
    return aTextSize / 11;
#endif
}

