# Arduino-Simple-DSO

##SUMMARY
The DSO needs only a standard Arduino-Uno or Arduino-Micro, a HC-05 Bluetooth module and this software.

## Features
- 150 kSamples per second with good quality.
- 300 kSamples per second with acceptable quality because of internal ADC limitations.
- Full touch screen control of all parameters.
- AC Measurement supported by using (passive) external attenuator circuit (see below).
- Automatic trigger level, range and offset selection.
- Manual trigger level and range select.
- Trigger delay.
- External trigger.
- 1120 Byte data buffer - 3.5 times display size.
- Display of min, max, average and peak to peak values.
- Display of period and frequency.
- 3 different types of external attenuator detected by software.
  - no attenuator (pin 8+9 left open).
  - passive attenuator with /1, /10, /100 attenuation (pin 8 connected to ground).
  - active attenuator (pin 9 connected to ground).
- Using 1.1 Volt internal reference. 5 Volt (VCC) also selectable which is useful if no attenuator is attached.

- Integrated frequency generator using 16 bit Timer1. Frequency from 119 mHz (8.388 second) to 8MHz

- Code serves also as an example of C++/Assembler coding and a non trivial interrupt handling routine.

- Included as example in the [BlueDisplay library for Arduino](https://github.com/ArminJo/android-blue-display/tree/master/arduino/libraries/BlueDisplay/BlueDisplay.zip).

# DOCUMENTATION

## SHORT INFO OUTPUT
- Arithmetic-average and peak to peak voltage of actual chart (In hold mode, chart is longer than display!)
- Frequency
- Timebase for div (31 pixel)

## LONG INFO OUTPUT
- Timebase for div (31 pixel)
- Slope of trigger
- Input channel: (0-5), T->AVR-temperature, R->1.1Volt-internal-reference G->internal-ground
- Minimum, arithmetic-average, max and peak to peak voltage of actual chart (In hold mode, chart is longer than display!)
- Trigger level
- Reference used: 5=5V 1  1=1.1Volt-internal-reference
On second line
- Period
- Frequency

## TOUCH
Short touch switches info output, long touch shows active GUI elements.

#SCHEMATIC for passive attenuator
```
configured by connecting pin 8 to ground

          D2    ADC INPUT_0  1.1 Volt         ADC INPUT_1 11 Volt        ADC INPUT_2 110 Volt
          /\          /\                         /\     ______              /\     ______
          |           |                          +-----| 220k |----+        +-----| 10 k |----------+
          _           |      ______              |      ______     |        |      ------           |
         | |          +-----| >4 M |----+        +-----| 220k |----+        |      _____            |
         | | 100 k    |      ------     |        |      ------     |        +-----| 5 M |-+ 2*5 M or|
         | |          _                 |        _                 |        _      -----  _ 3*3.3 M |
          -          | |                |       | |                |       | |           | |        |
          |          | | 10 k           |       | | 1 M            |       | | 1 M       | | 5 M    |
          O          | |                |       | |                |       | |           | |        |
   External_Trigger   -                 |        -                 |        -             -         |
                      |                 |        |                 |        |             |         |
                      +----+            |        +----+            |        +----+        +----+    |
                      |    |            |        |    |            |        |    |        |    |    |
                      |    = C 0.1uF    |        |    = C 0.1uF    |        |    = 0.1uF  |    = 0.1uF  400 Volt
                      |    |            |        |    |            |        |    |        |    |    |
                      O    O            |        O    O            |        O    O        O    O    |
                     DC   AC            |       DC   AC            |       DC   AC       DC   AC    |
         D3                            |                          |                     1000 V Range for mains
         /\           +----------------+--------------------------+--------------------------------+
         |            |
         O            O
        / _________  /   AC/DC            D8/Mode       D10/Frequency generator
       /            /    Switch             /\             \/
     O/    O      O/    O----------+        |              |
    GND    DC  AC |     DC         |        |              |
        ______    |       ______   |        |              |
  VREF-| 100k |---+------| 100k |--+        |              \/
        ------    |       ------   |        |
                  +--------||------+-GND----+
                         33 uF

```
DSO with passive attenuator on breadboard
![DSO with passive attenuator on breadboard](https://github.com/ArminJo/android-blue-display/blob/gh-pages/pictures/ArduinoDSO.jpg)
DSO settings menu
![DSO settings menu](https://github.com/ArminJo/android-blue-display/blob/gh-pages/screenshots/DSOSettings.png)
DSO frequency generator menu
![Frequency generator menu](https://github.com/ArminJo/android-blue-display/blob/gh-pages/screenshots/Frequency.png)

