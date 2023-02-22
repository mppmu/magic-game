# MAGIC Game Board V1.0



## Features

* Socket for one Arduino Mega 2560 Rev3.
* Socket for one Joy-IT KY-023 analogue joystick module.
* Two headers to connect two stepper motors of the type 28BYJ-48.
* Socket for one 16x2 LCD module of the type LCD-016N002L (alternative:
  DEM16215-CYR22).
* Power over mini USB connector or from Arduino.
* Additional UART over mini USB.
* Status LEDs: Power, OK, ERROR, progress, position, stepper motor activity.
* Header for connecting external LEDs.
* Buttons and connectors for these signals:
  - Reset.
  - Start of game.
  - Limit switches for azimuth and elevation of the telescope model.



## Notes

* In order to fit on the MAGIC Game board, the Joy-IT KY-023 analogue joystick
  module must be modified:
  - The 5-pin horizontal pin header with which the module is equipped by
    default must be removed.
  - A 5-pin vertical pin socket must be soldered on the bottom side of the
    module instead.
  - Four standoffs can be used to firmly fix the module to the MAGIC Game
    board.



## Mistakes, Bugs and Fixes

* Mini-USB J1 too far away from the edge of the board:
  - Move J1 closer to the edge of the PCB.
* Fuse F1 needs to support a current of at least 800 mA:
  - Change fuse type from MF-MSMF050-2 (500 mA) to MF-MSMF125-2 (1.3 A).
* USB_UART_TX/USB_UART_RX in conflict with Arduino RX/TX:
  - Connect to Arduino RX1/TX1 (D19/D18) instead of RX0/TX0 (D0/D1).
* The blue LED D25 of the type QLSP05B does not turn on.
  - The diode of the replacement type QLSP05B has an anode marking instead of
    cathode marking!
  - Rotate diode D25 by 180 degrees with respect to the mounting position
    indicated by the silkscreen.
* The white LED D24 is very bright compared to the other LEDs.
  - Change R41 from 220 to 1.5 kOhms.
  - In addition, change R56 from 220 Ohms to 1.5 kOhms for the external LED
    header.
* The blue LED D25 is very bright compared to the other LEDs.
  - Change R42 from 220 to 1.5 kOhms.
  - In addition, change R57 from 220 Ohms to 1.5 kOhms for the external LED
    header.
* LCD contrast regulation not working:
  - Use Arduino PWD pin D8 instead of A14.
* LCD backlight regulation not working:
  - Use Arduino PWD pin D9 instead of A15.
* LCD backlight polarity:
  - The LCD module of the type DEM16215-CYR22 has a swapped polarity of the
    backlight compared to the schematic symbol for the LCD-016N002L module.
  - For the DEM16215-CYR22 module, pin 15 = cathode, pin 16 = anode.
  - Implement option for both polarities with two sets of resistors, so that
    the polarity can be defined at assembly time by mounting the one or the
    other set of resistors.

