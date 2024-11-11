[![Website](https://img.shields.io/badge/Website-Link-blue.svg)](https://gavinlyonsrepo.github.io/)  [![Rss](https://img.shields.io/badge/Subscribe-RSS-yellow.svg)](https://gavinlyonsrepo.github.io//feed.xml)  [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/paypalme/whitelight976)


![ lcd image ](https://github.com/gavinlyonsrepo/pic_16F1619_projects/blob/master/images/LCDPCF.jpg)

#  Hd44780_LCD_PCF8574

Table of contents
---------------------------

  * [Overview](#overview)
  * [Installation](#installation)
  * [Software](#software)
  * [Output](#output)
  * [Tested on](#tested-on)
  * [Notes](#notes)

Overview
--------------------
* Name : HD44780_LCD_PCF8574
* Description :

0. Library to support the HD44780 LCD , (I2C PCF8574 "backpack" interface) 
   for the Arduino Eco system
1. C++ library.
2. Backlight, scroll, cursor and entry-mode control.
3. Custom character support + print class for numerical data.
4. Hardware I2C 
5. Tested on size 16x02 + 20x04 (but may work on other sizes  eg 16X4 but not tested).
6. Can support both I2C ports on the STM32 see tested section.

* Author: Gavin Lyons

Installation
------------------------------

The library is included in the official Arduino library manger and the optimum way to install it is using the library manager in the Arduino IDE. 

Software
--------------------------

**API**

The API (application programming interface)  html documentation is at link. Hosted on github pages and generated by Doxygen software. Here the user will find lots of information on files, functions & data types. 

[Software API Url Link](https://gavinlyonsrepo.github.io/misc/software_docs/HD44780_LCD_PCF8574/index.html)

Output
---------------------

Output  of custom character test in testrun example  file on 16x02 display.

![ pic ](https://github.com/gavinlyonsrepo/HD44780_LCD_RPI/blob/main/extras/image/custom_output.jpg)

20x04 display. 

![ pic2 ](https://github.com/gavinlyonsrepo/HD44780_LCD_PCF8574/blob/main/extras/image/2004.jpg)

Tested on
------------------------

Tested on following MCUs.
The example files are setup for an UNO/NANO rev 3.0 for the pin connections used 
by for other MCU testing see extras/doc folder GPIO_MCU_used.txt file.

1. Arduino  UNO & NANO v3
2. ESP8266 
3. ESP32 
4. STM32 "blue pill", Can support both I2C ports , Use STM32 example file. 

Notes
------------------------

1. "stm32duino" board manager core used in testing STM32 "blue pill"
2.  For description of entry modes , cursor types, custom characters etc [See]( http://dinceraydin.com/lcd/commands.htm) 
3. 16X04 board not tested but should work.
4. I2C Debugging can be turned on by commenting in a define in header file. 