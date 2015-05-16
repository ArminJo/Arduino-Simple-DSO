# Arduino-Simple-DSO

##  Features:
No dedicated hardware, just a plain arduino, a HC-05 Bluetooth module and this software.
Full touch screen control of all parameters.
150/300 kSamples per second
supports AC Measurement with (passive) external attenuator circuit.
3 external circuits detected by software - no attenuator, passive attenuator /1, /10, /100, active attenuator.
Automatic trigger, range and offset value selection
1120 Byte data buffer - 3.5 * displays
Min, max, average and peak to peak display
Period and frequency display
All settings can be changed during measurement
Gesture (swipe) control of timebase and chart
All AVR ADC input channels selectable
Touch trigger level select
1.1 Volt internal reference. 5 Volt (VCC) also usable.

The code can also be used as an example of C++/Assembler coding
and non trivial interrupt handling routine

## DOCUMENTATION

### INFO LINE
101us  - Time for div (31 pixel)
A Ch0  - Slope of trigger
       - Number of input channel (0-5) and T=AVR-temperature R=1.1Volt-internal-reference G=internal-ground
1.35 2.40 4.38 P2P3.03V - Min, arithmetic-average, max and peak to peak voltage of actual chart (In hold mode, chart is longer than display!)
0.86V  - Trigger auto => Value of actual trigger level (not updated)
5      - Reference used 5=5V 1  1=1.1Volt-internal-reference

### SECOND INFO LINE
20ms  - Period
50Hz  - Frequency
0.45 - Value of voltage picker line

### Action on touching on display:
while running, switch between upper info line on/off
while stopped, switch between:
1. show chart + grid + voltage picker
2. show additional info line
3. show gui

##SCHEMATIC for external components of Arduino DSO
...
SIMPLE EXTERNAL ATTENUATOR CIRCUIT (configured by connecting pin 10 to ground)

Safety circuit and AC/DC switch
3 resistors   2 diodes   1 capacitor   1 switch

               ADC INPUT_0  1.1 Volt         ADC INPUT_1 11 Volt        ADC INPUT_2 11 Volt
                     /\                         /\                         /\
                     |                          |                          |
                     |                          +-----| 220k |----+        |
                     +-----| >4 M |----+        +-----| 220k |----+        +-----| 10 k |----+
                     |                 |        |                 |        |                 |
                     _                 |        _                 |        _                 |
                    | |                |       | |                |       | |                |
                    | | 10 k           |       | | 1 M            |       | | 1 M            |
                    | |                |       | |                |       | |                |
                     -                 |        -                 |        -                 |
                     |                 |        |                 |        |                 |
                     +----+            |        +----+            |        +----+            |
                     |    |            |        |    |            |        |    |            |
                     |    = C 0.1uF    |        |    = C 0.1uF    |        |    = C 0.1uF    |
                     |    |            |        |    |            |        |    |            |
                     O    O            |        O    O            |        O    O            |
                    DC   AC            |       DC   AC            |       DC   AC            |
                                       |                          |                          |
                      +----------------+--------------------------+--------------------------+
                      |
                      O
          AC/DC      /
          Switch    /
                  O/    O----------+
               AC |     DC         |
                  |                |
  VREF-| 100k |---+------| 100k |--+
                  |                |
                  +--------||------+-GND
                         33 uF

...
