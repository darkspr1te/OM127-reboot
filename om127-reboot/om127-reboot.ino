
#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareCAN.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


/*
  U8glib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer example.    
*/
int incomingByte = 0;   // for incoming serial data
// OM127 LCD Screen Type
U8G2_ST7565_ERC12864_1_4W_SW_SPI  u8g2(U8G2_R0,/* clock=*/ PB0, /* data=*/ PB1, /* cs=*/ PB10, /* dc=*/ PB8, /* reset=*/PC13);
//HW Serial UART1 PA9/PA10

 const long interval = 500; 
 const int ledPin =  PA8;
 unsigned long previousMillis = 0; 
 int ledState = HIGH; 

#define BPIN 0
#define SPIN 1
#define Up_Button PB15
#define Down_Button PB12
#define Exit_Button PB13
#define Menu_Button PB14
#define USB_Detect PC15
#define LED_CAN_MUTE PA8


byte msgD0 ; // variable to be used in the example.
int old_ID =0;
int DataOne =0;
int DataTwo =0;
int DataThree =0;
int DataFour =0;
int DataFive =0;
int DataSix =0;
int DataSeven =0;
int DataEight =0;

// Instanciation of CAN interface
HardwareCAN canBus(CAN1_BASE);
CanMsg msg ;
CanMsg *r_msg;

void CANSetup(void)
{
  CAN_STATUS Stat ;

  Serial.end();

   Serial1.println("UART Online, About to setup CAN");
  // Initialize CAN module
 // Stat= canBus.map(CAN_GPIO_PB8_PB9);       // This setting is already wired in the OM127
  Stat = canBus.map(CAN_GPIO_PA11_PA12);       // This setting is already wired in the OM127
    if (Stat != CAN_OK){
       Serial1.println("ERROR map");
       Serial1.println(Stat,DEC);
    }
  /*
   * CAN_SPEED_33,
 CAN_SPEED_95,
  CAN_SPEED_125,
  CAN_SPEED_250,
  CAN_SPEED_500,
  CAN_SPEED_1000,
   */
 // canBus.set_pool_mode();
  Stat = canBus.begin(CAN_SPEED_500, CAN_MODE_NORMAL);    // Other speeds go from 125 kbps to 1000 kbps. CAN allows even more choices.
  if (Stat != CAN_OK){
       Serial1.println("ERROR begin");
       Serial1.println(Stat,DEC);
  }
  //filter(uint8 idx, uint32 id, uint32 mask, uint32 extID)
  //canBus.filter(0, 0x00ffee00, 0x1fffffff,0);          // filter don't work for ID 0x00FFEE00
  canBus.filter(0, 0x7df, 0,0);                // filter work for ID 0x00FFEE00
  //canBus.filter(1, 0, 0x1FFFFFFF, 0x1FFFFFFF);
  canBus.set_irq_mode();              // Use irq mode (recommended), so the handling of incoming messages
                                      // will be performed at ease in a task or in the loop. The software fifo is 16 cells long, 
                                      // allowing at least 15 ms before processing the fifo is needed at 125 kbps
                                    
  Stat = canBus.status();
  if (Stat != CAN_OK)
  {
       Serial1.println("ERROR status");
       Serial1.println(Stat,DEC);
  }
  else
   Serial1.println("CanBus INit Passed ");
     /* Your own error processing here */ ;   // Initialization failed
}

#define CAN_INAK_TimeOut 30
// Send one frame. Parameter is a pointer to a frame structure (above), that has previously been updated with data.
// If no mailbox is available, wait until one becomes empty. There are 3 mailboxes.
CAN_TX_MBX CANsend(CanMsg *pmsg) // Should be moved to the library?!
{
  CAN_TX_MBX mbx;
  volatile uint32 wait_ack = 0 ;
//Serial1.println("can send");
  do 
  {
    mbx = canBus.send(pmsg) ;
#ifdef USE_MULTITASK
    vTaskDelay( 1 ) ;                 // Infinite loops are not multitasking-friendly
#endif
//Serial1.println("can send part 2");
wait_ack++;
  }
  while((wait_ack != CAN_INAK_TimeOut) && ((mbx == CAN_TX_NO_MBX) != 0));        // Waiting outbound frames will eventually be sent, unless there is a CAN bus failure.
    if (wait_ack == CAN_INAK_TimeOut) 
    {
    Serial1.println("send fail");
    CANSetup() ;
    
    }
  return mbx ;
}

// Send message
// Prepare and send a frame containing some value 
void SendCANmessage(long id=0x001, byte dlength=8, byte d0=0x00, byte d1=0x00, byte d2=0x00, byte d3=0x00, byte d4=0x00, byte d5=0x00, byte d6=0x00, byte d7=0x00)
{
  // Initialize the message structure
  // A CAN structure includes the following fields:
  msg.IDE = CAN_ID_STD;          // Indicates a standard identifier ; CAN_ID_EXT would mean this frame uses an extended identifier
  msg.RTR = CAN_RTR_DATA;        // Indicated this is a data frame, as opposed to a remote frame (would then be CAN_RTR_REMOTE)
  msg.ID = id ;                  // Identifier of the frame : 0-2047 (0-0x3ff) for standard idenfiers; 0-0x1fffffff for extended identifiers
  msg.DLC = dlength;                   // Number of data bytes to follow

  // Prepare frame : send something
  msg.Data[0] = d0 ;
  msg.Data[1] = d1 ;
  msg.Data[2] = d2 ;
  msg.Data[3] = d3 ;
  msg.Data[4] = d4 ;
  msg.Data[5] = d5 ;
  msg.Data[6] = d6 ;
  msg.Data[7] = d7 ;
//Serial1.println("sendcan ");
 // digitalWrite(PA8, LOW);    // turn the onboard LED on
  CANsend(&msg) ;      // Send this frame
  delay(180);              
 // digitalWrite(PA8, HIGH);   // turn the LED off 
  delay(100);  
  //Serial1.println("endsend");
  Serial1.print(id,HEX);
    Serial1.print("#");
  Serial1.print(d0,HEX);
    Serial1.print(".");
  Serial1.print(d1,HEX);
    Serial1.print(".");
  Serial1.print(d2,HEX);
    Serial1.print(".");
  Serial1.print(d3,HEX);
    Serial1.print(".");
  Serial1.print(d4,HEX);
    Serial1.print(".");
  Serial1.print(d5,HEX);
    Serial1.print(".");
  Serial1.print(d6,HEX);
    Serial1.print(".");
  Serial1.println(d7,HEX);
          
}

// The application program starts here
int bState = 0;         // variable for reading the pushbutton status
int sState = 0;         // variable for reading the switch status
byte st = 0x31; // buttot 1 on the CD30MP3


  
void box()
{
  int Button_Key =0;
  u8g2.setFont(u8g2_font_6x10_tf);
//u8g2.setFontRefHeightAll();    /* this will add some extra space for the text inside the buttons */
//u8g2.userInterfaceMessage("Boot STM32", "darkspr1te 2018", "USB/CAN Switch", " Ok \n Cancel ");
Button_Key = u8g2.userInterfaceSelectionList("Darkspr1te OM127 Menu", 1, "Change CAN Speed\nChange CAN bits\nSomething else here");
switch (Button_Key) {
    case 0:
      //do something when var equals 1
       Serial1.println("Select cancel");
      break;
    case 2:
      //do something when var equals 2
      Serial1.println("option 1");
      break;
      case 3:
      //do something when var equals 2
      Serial1.println("option 2");
      break;      
            case 4:
      //do something when var equals 2
      Serial1.println("option 3");
      break;  
            case 5:
      //do something when var equals 2
      Serial1.println("option 4");
      break;  
            case 6:
      //do something when var equals 2
      Serial1.println("option 5");
      break;  
            case 7:
      //do something when var equals 2
      Serial1.println("option 6");
      break;  
    default:
      // if nothing else matches, do the default
      // default is optional
      Serial1.println("default");
      break;
  }
Serial1.println("Box Draw finished");
 while (digitalRead(Exit_Button)!=1){delay(100);}
}
void box_two()
{
  int Button_Key =0;
  u8g2.setFont(u8g2_font_6x10_tf);

Button_Key = u8g2.userInterfaceMessage("Boot STM32", "darkspr1te 2018", "msg3", " Ok \n Cancel ");
switch (Button_Key) {
    case 0:
      //do something when var equals 1
       Serial1.println("Select Cancel Button");
      break;
    case 2:
      //do something when var equals 2
      Serial1.println("Select Cancel Option");
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      Serial1.println("Select OK");
      break;
  }
Serial1.println("Box Draw finished");
  while (digitalRead(Exit_Button)!=1){}
}


void Led_Blink()

{
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
   //Serial1.print("LED");
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }

 }
void setup(void) {

/* SET PA8 output for LED 
 *  #define Up_Button PB15
#define Down_Button PB12
#define Exit_Button PB13
#define Menu_Button PB14
#define USB_Detect PC15


 */

  pinMode(Down_Button, INPUT);//Down
  pinMode(Exit_Button, INPUT);//Exit
  pinMode(Menu_Button, INPUT);//Enter
  pinMode(Up_Button, INPUT);//Up
  pinMode(USB_Detect, INPUT);//USB Plug detect
  //pinMode(PA9,OUTPUT);//UART 1 TX 
  //pinMode(PA10,OUTPUT);//UART 1 RX
  //pinMode(PA12,OUTPUT);//CAN 1 TX 
  //pinMode(PA11,OUTPUT);//CAN 1 RX 
  pinMode(LED_CAN_MUTE, OUTPUT);
  digitalWrite(LED_CAN_MUTE,LOW);
  pinMode(PA13, OUTPUT);
  digitalWrite(PA13,LOW);
  
//begin(uint8_t menu_select_pin, uint8_t menu_next_pin, uint8_t menu_prev_pin, uint8_t menu_up_pin = U8X8_PIN_NONE, uint8_t menu_down_pin = U8X8_PIN_NONE, uint8_t menu_home_pin = U8X8_PIN_NONE)
  u8g2.begin(PB14,PB15, PB12, U8X8_PIN_NONE, U8X8_PIN_NONE, PB13);
 // u8g2.begin();  
  Serial1.begin(115200);
  Serial1.println("Serial1 OK Ready");


//  pinMode(PA11, OUTPUT); // input for hardware switch
 // digitalWrite(PA11,LOW);


    msgD0 = 0x01;
    CANSetup() ; 
    Serial1.print("CPU Speed ");
     Serial1.println(F_CPU);
     Serial1.print("PCLK1 speed ");
    Serial1.println(PCLK1); 
    Serial1.print("PCLK2 speed ");
    Serial1.println(PCLK1); 
  delay(1500);
}


void loop(void) {
  u8g2.firstPage();
  u8g2.setContrast(0);

  do {
     if (Serial1.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial1.read();
               Serial1.print(incomingByte,DEC);
                if (incomingByte == 108) {
                   box();
                 }
               if (incomingByte == 109) {
                   box_two();
                 }
        }
      if (digitalRead(Up_Button)==1){
      
        
      }
      if (digitalRead(Menu_Button)!=1){
          while (digitalRead(Menu_Button)!=1){}
                box_two();
      }
      if (digitalRead(Down_Button)==1){
        
      }
      if (digitalRead(Exit_Button)!=1){
          while (digitalRead(Exit_Button)!=1){}
          
                box();
      }
      
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2,6,"darkspr1te debug");
    u8g2.setCursor(2, 15);
    u8g2.print(incomingByte,DEC);
    u8g2.setCursor(2, 30);
    u8g2.print("USB Power ");
    u8g2.print(digitalRead(PC15));
    u8g2.drawFrame(0,0,128,64);
     if (old_ID != 0){
                  u8g2.setCursor(2, 40);
                  u8g2.print("Code $");
                  u8g2.setCursor(2, 49);
                  u8g2.print(old_ID,HEX);
                  u8g2.print(".");
                  u8g2.print( DataOne,HEX);
                  u8g2.print(".");
                   u8g2.print( DataTwo,HEX);
                  u8g2.print(".");
                  u8g2.print( DataThree,HEX);
                  u8g2.print(".");
                   u8g2.print( DataFour,HEX);
                  u8g2.print(".");
                   u8g2.print( DataFive,HEX);
                  u8g2.print(".");
                   u8g2.print(DataSix,HEX);
                  u8g2.print(".");
                   u8g2.print( DataSeven,HEX);
                  u8g2.print(".");
                 
     }

        if ((r_msg = canBus.recv()) != NULL){
                  old_ID=r_msg->ID;
                  DataOne=r_msg->Data[0];             
                  DataTwo=r_msg->Data[1];             
                  DataThree=r_msg->Data[2];             
                  DataFour=r_msg->Data[3];             
                  DataFive=r_msg->Data[4];             
                  DataSix=r_msg->Data[5];             
                  DataSeven=r_msg->Data[6];             
                  DataEight=r_msg->Data[7];             
                  
                  long msgID = 0x7DF ;
                  SendCANmessage(r_msg->ID+9, 8, 0x04, st, 0x00) ;
               //   u8g2.setFont(u8g2_font_5x7_tf);
                  //u8g2.print(2,6,r_msg->ID);

                  canBus.free();
         
        }
    
  } while ( u8g2.nextPage() );
 //delay(1000);
  //digitalWrite(PA8, LOW);
//delay(1000);
//digitalWrite(PA8, HIGH);
//Led_Blink();
   

 
}


