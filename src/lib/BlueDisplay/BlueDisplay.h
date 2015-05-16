/*
 * BlueDisplay.h
 *
 *  Created on: 12.09.2014
 * @author Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmail.com
 *      License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 * @version 1.0.0
 */

#ifndef BLUEDISPLAY_H_
#define BLUEDISPLAY_H_

// Uncomment this if using a local MI0283QT2
//#define LOCAL_DISPLAY_EXISTS

#include <avr/pgmspace.h>

#include <stdint.h>

#define DISPLAY_DEFAULT_HEIGHT 240 // value to use if not connected
#define DISPLAY_DEFAULT_WIDTH 320
#define STRING_BUFFER_STACK_SIZE 20 // Buffer size allocated on stack for ...PGM() functions.
struct XYSize {
    uint16_t XWidth;
    uint16_t YHeight;
};
/*
 * Basic colors
 */
// RGB to 16 bit 565 schema - 5 red | 6 green | 5 blue
#define COLOR_WHITE     0xFFFF
#define COLOR_BLACK     0X0001 // 16 because 0 is used as flag (e.g. in touch button for default color)
#define COLOR_RED       0xF800
#define COLOR_GREEN     0X03E0
#define COLOR_BLUE      0x001F
#define COLOR_DARK_BLUE 0x0014
#define COLOR_YELLOW    0XFFE0
#define COLOR_MAGENTA   0xF81F
#define COLOR_CYAN      0x03FF
#define COLOR_ORANGE    0xF880

// If used as background color for char or text, the background will not filled
#define COLOR_NO_BACKGROUND   0XFFFE

#define BLUEMASK 0x1F
#define RGB(r,g,b)   (((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3)) //5 red | 6 green | 5 blue
/*
 * Android system tones
 */
#define TONE_CDMA_KEYPAD_VOLUME_KEY_LITE 89
#define TONE_PROP_BEEP 27
#define TONE_PROP_BEEP2 28
#define TONE_CDMA_ONE_MIN_BEEP 88
#define TONE_DEFAULT TONE_CDMA_KEYPAD_VOLUME_KEY_LITE
/*
 * Android Text sizes which are closest to the 8*12 font used locally
 */
#define TEXT_SIZE_11 11
#define TEXT_SIZE_22 22 // for factor 2 of 8*12 font
#define TEXT_SIZE_33 33 // for factor 3 of 8*12 font
// TextSize * 0.6
#ifdef LOCAL_DISPLAY_EXISTS
// 8/16 instead of 7/13 to be compatible with 8*12 font
#define TEXT_SIZE_11_WIDTH 8
#define TEXT_SIZE_22_WIDTH 16
#else
#define TEXT_SIZE_11_WIDTH 7
#define TEXT_SIZE_22_WIDTH 13
#endif

// TextSize * 0.93
// 12 instead of 11 to  have a margin
#define TEXT_SIZE_11_HEIGHT 12
#define TEXT_SIZE_22_HEIGHT 24

// TextSize * 0.93
// 9 instead of 8 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_11_ASCEND 9
// 18 instead of 17 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_22_ASCEND 18

// TextSize * 0.24
#define TEXT_SIZE_11_DECEND 3
// 6 instead of 5 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_22_DECEND 6

uint8_t getTextWidth(uint8_t aTextSize);
uint8_t getTextAscend(uint8_t aTextSize);
uint16_t getTextAscendMinusDescend(uint8_t aTextSize);
uint8_t getTextMiddle(uint8_t aTextSize);
uint8_t getLocalTextSize(uint8_t aTextSize);

/*
 * Function tags for Bluetooth serial communication
 */
// Sub functions for GLOBAL_SETTINGS
extern const int FUNCTION_TAG_GLOBAL_SETTINGS;
extern const int SET_FLAGS_AND_SIZE;
extern const int SET_CODEPAGE;
extern const int SET_CHARACTER_CODE_MAPPING;
extern const int SET_LONG_TOUCH_DOWN_TIMEOUT;
// Sub functions for SET_FLAGS_AND_SIZE
extern const int BD_FLAG_FIRST_RESET_ALL;
extern const int BD_FLAG_TOUCH_BASIC_DISABLE;
extern const int BD_FLAG_TOUCH_MOVE_DISABLE;
extern const int BD_FLAG_LONG_TOUCH_ENABLE;
extern const int BD_FLAG_USE_MAX_SIZE;

extern const int FUNCTION_TAG_CLEAR_DISPLAY;

//3 parameter
extern const int FUNCTION_TAG_DRAW_PIXEL;

// 5 parameter
extern const int FUNCTION_TAG_DRAW_LINE;
extern const int FUNCTION_TAG_DRAW_RECT;
extern const int FUNCTION_TAG_FILL_RECT;

extern const int FUNCTION_TAG_DRAW_CIRCLE;
extern const int FUNCTION_TAG_FILL_CIRCLE;

// Parameter + Data
extern const int FUNCTION_TAG_DRAW_CHAR;
extern const int FUNCTION_TAG_DRAW_STRING;
extern const int FUNCTION_TAG_DRAW_CHART;
extern const int FUNCTION_TAG_DRAW_CHART_WITHOUT_DIRECT_RENDERING;
extern const int FUNCTION_TAG_DRAW_PATH;
extern const int FUNCTION_TAG_FILL_PATH;

// Flags for BUTTON_GLOBAL_SETTINGS
extern const int USE_UP_EVENTS_FOR_BUTTONS;

// Values for Button flag
extern const int BUTTON_FLAG_DO_BEEP_ON_TOUCH;
extern const int BUTTON_FLAG_NOTHING;

extern const int TOUCHSLIDER_SHOW_BORDER;
extern const int TOUCHSLIDER_VALUE_BY_CALLBACK; // if set value will be set by callback handler
extern const int TOUCHSLIDER_IS_HORIZONTAL;

#ifdef __cplusplus
class BlueDisplay {
public:
    BlueDisplay();
    void setFlagsAndSize(uint16_t aFlags, uint16_t aWidth, uint16_t aHeight);
    void setCharacterMapping(uint8_t aChar, uint16_t aUnicodeChar);

    void playTone(uint8_t aToneIndex);
    void playTone(void);
    void setLongTouchDownTimeout(uint16_t aLongTouchTimeoutMillis);

    void clearDisplay(uint16_t aColor);
    void drawDisplayDirect(void);

    void drawPixel(uint16_t aXPos, uint16_t aYPos, uint16_t aColor);
    void drawCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, uint16_t aColor, uint16_t aStrokeWidth);
    void fillCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, uint16_t aColor);
    void drawRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor, uint16_t aStrokeWidth);
    void drawRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, uint16_t aColor, uint16_t aStrokeWidth);
    void fillRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor);
    void fillRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, uint16_t aColor);

    uint16_t drawChar(uint16_t aPosX, uint16_t aPosY, char aChar, uint8_t aCharSize, uint16_t aFGColor, uint16_t aBGColor);
    uint16_t drawText(uint16_t aXStart, uint16_t aYStart, const char *aStringPtr, uint8_t aFontSize, uint16_t aColor,
            uint16_t aBGColor);
    uint16_t drawTextPGM(uint16_t aXStart, uint16_t aYStart, PGM_P aPGMString, uint8_t aFontSize, uint16_t aColor,
    uint16_t aBGColor);
    void debugMessage(const char *aStringPtr);

    void drawLine(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor);
    void drawLineRel(uint16_t aXStart, uint16_t aYStart, uint16_t aXDelta, uint16_t aYDelta, uint16_t aColor);
    void drawLineOneX(uint16_t x0, uint16_t y0, uint16_t y1, uint16_t color);
    void drawLineWithThickness(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, int16_t aThickness,
            uint8_t aThicknessMode, uint16_t aColor);

    void drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, uint16_t aColor, uint16_t aClearBeforeColor,
            uint8_t *aByteBuffer, uint16_t aByteBufferLength);
    void drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, uint16_t aColor, uint16_t aClearBeforeColor,uint8_t aChartIndex,bool aDoDrawDirect,
            uint8_t *aByteBuffer, uint16_t aByteBufferLength);

    void setMaxDisplaySize(struct XYSize * const aMaxDisplaySizePtr);
    void setActualDisplaySize(struct XYSize * const aActualDisplaySizePtr);
    uint16_t getDisplayWidth(void);
    uint16_t getDisplayHeight(void);

    void
    getNumber(void (*aNumberHandler) (const float ));
    void getNumberWithShortPrompt(void (*aNumberHandler)(const float), const char *aShortPromptString);
    void getNumberWithShortPromptPGM(void (*aNumberHandler)(const float),const char *tShortPromptLengtht);
    void getText(void (*aTextHandler)(const char *));

    /*
     * Button stuff
     */
    void resetAllButtons(void);
    uint8_t createButton(const uint16_t aPositionX, const uint16_t aPositionY, const uint16_t aWidthX, const uint16_t aHeightY,
            const uint16_t aButtonColor, const char * aCaption, const uint8_t aCaptionSize, const uint8_t aFlags,
            const int16_t aValue, void (*aOnTouchHandler)(uint8_t, int16_t));
    uint8_t createButtonPGM(const uint16_t aPositionX, const uint16_t aPositionY, const uint16_t aWidthX,
            const uint16_t aHeightY, const uint16_t aButtonColor, PGM_P aPGMCaption, const uint8_t aCaptionSize, const uint8_t aFlags,
            const int16_t aValue, void (*aOnTouchHandler)(uint8_t, int16_t));
    void drawButton(uint8_t aButtonNumber);
    void drawButtonCaption(uint8_t aButtonNumber);
    void setButtonCaption(uint8_t aButtonNumber, const char * aCaption, bool doDrawButton);
    void setButtonCaptionPGM(uint8_t aButtonNumber, PGM_P aPGMCaption, bool doDrawButton);
    void setButtonValue(uint8_t aButtonNumber, const int16_t aValue);
    void setButtonColor(uint8_t aButtonNumber, const int16_t aButtonColor);
    void setButtonColorAndDraw(uint8_t aButtonNumber, const int16_t aButtonColor);
    void setRedGreenButtonColor(uint8_t aButtonNumber, int16_t aValue, bool doDrawButton);

    void activateButton(uint8_t aButtonNumber);
    void deactivateButton(uint8_t aButtonNumber);
    void activateAllButtons(void);
    void deactivateAllButtons(void);
    void setButtonsGlobalFlags(uint16_t aFlags);
    void setButtonsTouchTone(uint8_t aToneIndex, uint8_t aToneVolume);

    /*
     * Slider stuff
     */
    void resetAllSliders(void);
    uint8_t createSlider(const uint16_t aPositionX, const uint16_t aPositionY, const uint8_t aBarWidth, const uint16_t aBarLength,
            const uint16_t aThresholdValue, const int16_t aInitalValue, const uint16_t aSliderColor, const uint16_t aBarColor,
            const uint8_t aOptions, void (*aOnChangeHandler)(const uint8_t, const int16_t));
    void drawSlider(uint8_t aSliderNumber);
    void drawSliderBorder(uint8_t aSliderNumber);
    void setSliderActualValueAndDraw(uint8_t aSliderNumber,int16_t aActualValue);

    void activateSlider(uint8_t aSliderNumber);
    void deactivateSlider(uint8_t aSliderNumber);
    void activateAllSliders(void);
    void deactivateAllSliders(void);

private:
    struct XYSize mReferenceDisplaySize; // contains requested display size
            struct XYSize mActualDisplaySize;
            struct XYSize mMaxDisplaySize;
            uint16_t mDisplayHeight;
            uint16_t mDisplayWidth;

        };
// The instance provided by the class itself
extern BlueDisplay BlueDisplay1;
#endif

#ifdef LOCAL_DISPLAY_EXISTS
#include <MI0283QT2.h>
/*
 * MI0283QT2 TFTDisplay - must provided by main program
 * external declaration saves ROM (210 Bytes) and RAM ( 20 Bytes)
 * and avoids missing initialization :-)
 */
extern MI0283QT2 LocalDisplay;
#endif

uint16_t drawMLText(uint16_t aPosX, uint16_t aPosY, const char *aStringPtr, uint8_t aTextSize, uint16_t aColor, uint16_t aBGColor);

#endif /* BLUEDISPLAY_H_ */
