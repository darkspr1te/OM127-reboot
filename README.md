### OM127-reboot

Basic board bring up, CAN 500 working, LCD working, Buttons (polling) Working, UART1 Serial Working, USB/USB Detect working. Partial REV ENG of schematics,

Hackaday Logs for current discussions [URL](https://hackaday.io/project/158560-om127-reboot)

```
CPU:STM32F103RB 
CAN IC:TJA1050
K-Line : Transistor/Opamp
```
This project is also a sister project to the J2534-pic project 

J2534-pic:-[URL](https://github.com/darkspr1te/J2534-pic.git)

### Warning: Using this software on your device will erase OEM bootloader+firmware and you will no longer be able to use it in it's original application

### Still TODO:

- [ ] Add more CAN OBD logic,
- [x] remap SWD to GPIO for CAN chip mute control
- [x] rev eng K-line etc (Prob similar to ELM327 schematics)
- [ ] add pictures of internals/mods and current menus 
- [ ] add schematics in pdf format 
- [ ] Add protocols for :-
  - [ ] 1	SAE J1850 PWM (41.6 kbaud)
  - [ ] 2	SAE J1850 VPW (10.4 kbaud)
  - [ ] 3	ISO 9141-2 (5 baud init, 10.4 kbaud)
  - [ ] 4	ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
  - [ ] 5	ISO 14230-4 KWP (fast init, 10.4 kbaud)
  - [x] 6	ISO 15765-4 CAN (11 bit ID, 500 kbaud)
  - [x] 7	ISO 15765-4 CAN (29 bit ID, 500 kbaud)
  - [x] 8	ISO 15765-4 CAN (11 bit ID, 250 kbaud)  
  - [x] 9	ISO 15765-4 CAN (29 bit ID, 250 kbaud)  
- [x] EEPROM saving, CAN Sniffer.




Device Autophix OM127URL :- [URL](http://www.autophix.com/en/obd-mate/om127.html)
Ancel AD310(Orange) :-[URL](http://www.anceldirect.com/products/2017/20170423/n2017042314043.html)
---

Requires modified Arduino STM32F1x see [this](http://www.stm32duino.com/viewtopic.php?t=72)

also see:
`rogerclarkmelbourne/Arduino_STM32@master...coddingtonbear:HardwareCAN`

also [SPI flash lib](http://www.stm32duino.com/viewtopic.php?t=9)

Check notes within the source code for descriptions for now, other documentation to follow. 

`
Current Features of software
	RAW CAN sniffer 
	CAN injector/Denial  
	RAW K-Line Sniffer
	EEPROM save/dump via USB for CAN sniffer
	USB/CAN Auto detect
	UART1 software switchable (Kline/uart)
	Contrast setting
	PID Filter (usable within CAN Sniffer)
`

K-Line Docs [URL](https://github.com/iwanders/OBD9141) 

Additional reading on OBD General:- 
[URL](http://www.cnblogs.com/shangdawei/p/3627565.html)
[URL](https://github.com/hackingvolvo/SardineCAN-Arduino)
[URL](https://github.com/fenugrec/oj2534-fw)
[URL](https://github.com/NikolaKozina/j2534)
[URL](https://github.com/macchina/m2-hardware/blob/master/M2/Interface%20Board%20Schematic.pdf)
