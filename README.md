# OM127-reboot
Basic board bring up, CAN 500 working/Shared with USB, LCD working, Buttons (polling) Working, UART1 Serial Working(UARTS shared with KWP/PWM lines, USB/USB Detect working. Partial REV ENG of schematics,

CPU:STM32F103RB 
CAN IC:TJA1050
K-Line: Transistor/Opamp
PWM Line: 

Still TODO:
Add more CAN OBD logic,
remap SWD to GPIO for CAN chip mute control
rev eng K-line etc (Prob similar to ELM327 schematics)
Add protocols for :-
1	SAE J1850 PWM (41.6 kbaud)
2	SAE J1850 VPW (10.4 kbaud)
3	ISO 9141-2 (5 baud init, 10.4 kbaud)
4	ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
5	ISO 14230-4 KWP (fast init, 10.4 kbaud)
6	ISO 15765-4 CAN (11 bit ID, 500 kbaud)
7	ISO 15765-4 CAN (29 bit ID, 500 kbaud)
8	ISO 15765-4 CAN (11 bit ID, 250 kbaud) - used mainly on utility vehicles and Volvo
9	ISO 15765-4 CAN (29 bit ID, 250 kbaud) - used mainly on utility vehicles and Volv
EEPROM not working, SPI still needs work :- FIXED

*******************
Requires modified Arduino STM32F1x see :- http://www.stm32duino.com/viewtopic.php?t=72
also
rogerclarkmelbourne/Arduino_STM32@master...coddingtonbear:HardwareCAN
also SPI flash lib
http://www.stm32duino.com/viewtopic.php?t=9

Device URL :- http://www.autophix.com/en/obd-mate/om127.html

PA13 is shared with CAN Mute & SWD, lift leg and leave floating for CANBUS TX Enable or add a jumper wire from solder point next to R18 to PIN8 of the
TJA1050 SO8 chip, leg must be lift from pad to allow control (PA13 is always high when remapped for SWD)


Will be adding K-Line soon
https://github.com/iwanders/OBD9141
**********************
I have included a current as of march 2018 (all needed)stmduino libs, in linux place in your /home/username/.arduino15 folder and the arduino system will accept it, 
note of you update any libs you could break the build, this is mainly the alterations to stmduino system
======================
i sudgest you also get openocd from source and build, openocd is required to see memory values/debugging, this is not a dev board so not all pins broken out to easy access
see pinouts in pdf for real pin descriptions,often i use just PA12 instead of Up_Button, UART1 is on resistors R1-TX and R31 RX (needs pull up)
SWD port is as follows


USB Port side------GND--SWCLK---SWDIO---3.3V----------Cable out to OBD



