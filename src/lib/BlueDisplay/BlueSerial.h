/*
 * BlueSerial.h
 *
 * @date 01.09.2014
 * @author Armin Joachimsmeyer
 *      Email:   armin.joachimsmeyer@gmail.com
 * @copyright GPL v3 (http://www.gnu.org/licenses/gpl.html)
 * @version 1.0.0
 *
 *
 * SEND PROTOCOL USED:
 * Message:
 * 1. Sync byte A5
 * 2. Byte function token
 * 3. Short length (in short units) of parameters
 * 4. Short n parameters
 *
 * Optional Data:
 * 1. Sync Byte A5
 * 2. Byte Data_Size_Type token (byte, short etc.)
 * 3. Short length of data
 * 4. Length data values
 *
 *
 * RECEIVE PROTOCOL USED:
 *
 * Touch/size message has 7 bytes:
 * 1 - Gross message length in bytes
 * 2 - Function code
 * 3 - X Position LSB
 * 4 - X Position MSB
 * 5 - Y Position LSB
 * 6 - Y Position MSB
 * 7 - Sync token
 *
 * Callback message has 13 bytes:
 * 1 - Gross message length in bytes
 * 2 - Function code
 * 16 bit button index
 * 32 bit callback address
 * 32 bit value
 * 13 - Sync token
 */

#ifndef BLUE_SERIAL_H_
#define BLUE_SERIAL_H_

#ifndef USE_SIMPLE_SERIAL
//#define USE_SIMPLE_SERIAL // outcomment it or better define it for the compiler with -DUSE_SIMPLE_SERIAL
#endif

#define BAUD_STRING_4800 "4800"
#define BAUD_STRING_9600 "9600"
#define BAUD_STRING_19200 "19200"
#define BAUD_STRING_38400 "38400"
#define BAUD_STRING_57600 "57600"
#define BAUD_STRING_115200 "115200"
#define BAUD_STRING_230400 "230400"
#define BAUD_STRING_460800 "460800"
#define BAUD_STRING_921600 " 921600"
#define BAUD_STRING_1382400 "1382400"

#define BAUD_4800 (4800)
#define BAUD_9600 (9600)
#define BAUD_19200 (19200)
#define BAUD_38400 (38400)
#define BAUD_57600 (57600)
#define BAUD_115200 (115200)
#define BAUD_230400 (230400)
#define BAUD_460800 (460800)
#define BAUD_921600 ( 921600)
#define BAUD_1382400 (1382400)

#define PAIRED_PIN 5

#define SYNC_TOKEN 0xA5
extern const int DATAFIELD_TAG_BYTE;
extern const int LAST_FUNCTION_TAG_DATAFIELD;

void sendUSARTArgs(uint8_t aFunctionTag, int aNumberOfArgs, ...);
void sendUSARTArgsAndByteBuffer(uint8_t aFunctionTag, int aNumberOfArgs, ...);
void USART3_sendBuffer(uint8_t * aBytePtr, int aLengthOfDataToSend);
void sendUSART5Args(uint8_t aFunctionTag, uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aColor);
void sendUSART5ArgsAndByteBuffer(uint8_t aFunctionTag, uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd,
        uint16_t aColor, uint16_t aBufferLength, uint8_t * aBuffer);

#ifdef LOCAL_DISPLAY_EXISTS
bool USART_isBluetoothPaired(void);
#else
#define USART_isBluetoothPaired() (true)
#endif

// Simple blocking serial version without receive buffer and other overhead
// Remove comment if you want to use it instead of Serial....
//#define USE_SIMPLE_SERIAL

#ifdef USE_SIMPLE_SERIAL
extern bool allowTouchInterrupts;
void initSimpleSerial(uint32_t aBaudRate, bool aUsePairedPin);
void USART3_send(char aChar);
#else
void serialEvent(void);
#endif

#endif /* BLUE_SERIAL_H_ */
