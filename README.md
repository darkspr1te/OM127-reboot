### OM127-reboot

Basic board bring up, CAN 500 working, LCD working, Buttons (polling) Working, UART1 Serial Working, USB/USB Detect working. Partial REV ENG of schematics,

```
CPU:STM32F103RB 
CAN IC:TJA1050
K-Line : Transistor/Opamp
```


### Still TODO:

- [o] Add more CAN OBD logic,
- [x] remap SWD to GPIO for CAN chip mute control
- [x] rev eng K-line etc (Prob similar to ELM327 schematics)
- [o] Add protocols for :-
  - [ ] 1	SAE J1850 PWM (41.6 kbaud)
  - [ ] 2	SAE J1850 VPW (10.4 kbaud)
  - [o] 3	ISO 9141-2 (5 baud init, 10.4 kbaud)
  - [o] 4	ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
  - [o] 5	ISO 14230-4 KWP (fast init, 10.4 kbaud)
  - [o] 6	ISO 15765-4 CAN (11 bit ID, 500 kbaud)
  - [ ] 7	ISO 15765-4 CAN (29 bit ID, 500 kbaud)
  - [o] 8	ISO 15765-4 CAN (11 bit ID, 250 kbaud) - used mainly on utility vehicles and Volvo
  - [ ] 9	ISO 15765-4 CAN (29 bit ID, 250 kbaud) - used mainly on utility vehicles and Volvo
- [x] EEPROM not working, SPI still needs work :- FIXED

x = working but not necessarily feature rich
o = implemented but not necessarily working correctly 


Device URL :- http://www.autophix.com/en/obd-mate/om127.html

---

Requires modified Arduino STM32F1x see [this](http://www.stm32duino.com/viewtopic.php?t=72)

also see:
`rogerclarkmelbourne/Arduino_STM32@master...coddingtonbear:HardwareCAN`

also [SPI flash lib](http://www.stm32duino.com/viewtopic.php?t=9)

Device [URL](http://www.autophix.com/en/obd-mate/om127.html)

`PA13` is shared with CAN Mute & SWD, lift leg and leave floating for CANBUS `TX Enable` or add a jumper wire from solder point next to `R18` to `PIN8` of the **TJA1050** SO8 chip, leg must be lifted from pad to allow control (`PA13 `is always *high* when remapped for SWD)


Will be adding K-Line soon

https://github.com/iwanders/OBD9141

Additional reading 
http://www.cnblogs.com/shangdawei/p/3627565.html
https://github.com/hackingvolvo/SardineCAN-Arduino
https://github.com/fenugrec/oj2534-fw
https://github.com/NikolaKozina/j2534
