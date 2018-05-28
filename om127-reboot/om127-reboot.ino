
#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareCAN.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "winbondflash.h"


int incomingByte = 0;   // for incoming serial data
// OM127 LCD Screen Type
U8G2_ST7565_ERC12864_1_4W_SW_SPI  u8g2(U8G2_R0,/* clock=*/ PB0, /* data=*/ PB1, /* cs=*/ PB10, /* dc=*/ PB8, /* reset=*/PC13);


typedef struct
 {
     int stamp_sr;
     CanMsg Data_sr ;
     
 }  record_type;

//HW Serial UART1 PA9/PA10

 const long interval = 500; 
 const long blink_delay = 1000;
 const int ledPin =  PA8;
 unsigned long previousMillis = 0; 
  unsigned long previousMillis_two = 0; 
 int ledState = HIGH; 
 boolean cur_state =false;
int LOG_ENABLED = 0;
boolean select_char=false;

#define DEFAULT_MASK  0x7df
int saved_ID_Filter = DEFAULT_MASK;
winbondFlashSPI mem;
  
//begin(uint8_t menu_select_pin, uint8_t menu_next_pin, uint8_t menu_prev_pin, uint8_t menu_up_pin = U8X8_PIN_NONE, uint8_t menu_down_pin = U8X8_PIN_NONE, uint8_t menu_home_pin = U8X8_PIN_NONE)
// init LCD with Menu buttons tied in
 // u8g2.begin(PB14,PB12, PB15, U8X8_PIN_NONE, U8X8_PIN_NONE, PB13);
  //u8g2.begin(Menu_Button,Down_Button,Up_Button,U8X8_PIN_NONE,U8X8_PIN_NONE,Exit_Button);
#define SPI_SLAVE_SEL_PIN    PA4
#define Up_Button            PB15
#define Down_Button          PB12
#define Exit_Button          PB13
#define Menu_Button          PB14
#define USB_Detect           PC15 //usefull to enable STLINK via IRQ-Menu

#define LED_CAN_MUTE         PA8
#define CAN_MUTE             PA13 //in order to function disable STLINK port

#define KWP_LOW              PA15 //in order to function disable STLINK port //Sets PIN15 on OBD  High 12v
#define KWP_TX               PA9 //ISO-9141 //Sets PIN7 on OBD  High 12v
#define KWP_RX               PA10//ISO-9141

#define J1850_ONE            PA0 //Q13 Emmiter of PNP AND Q11- Base NPN - input maybe?
#define J1850_TWO            PA1 //Comp1 output - def input 
#define J1850_THREE          PA2 //Q8 Base NPN  On to Q9 Base PNP onto //Sets PIN 10 OBD High 5V
#define J1850_FOUR           PA3//Q6  // Creates 5v Differential On PIN2/10 On obd


//define unknown pins
//PA1 goes to Comparater out 1//UART2 RTS
//PA2 goes to Q8 Gate ///uart2 tx 
//PA3 goes to Q6 Gate //uart 2 rx
//PA10 RX Comparator out 2 //uart1 RX
//PA9 TX To Base Q4 // Uart2 TX 
//PA15 Base Q2
//


byte msgD0 ; // variable to be used in the example.
//need to switch to canmsg array, for testing this works
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

  Serial.end();//Ensure USB has given up PA11/PA12
   Serial1.println("About to setup CAN");
// Initialize CAN module
  Stat = canBus.map(CAN_GPIO_PA11_PA12);       // This setting is already wired in the OM127
    if (Stat != CAN_OK){
       Serial1.println("ERROR map can port");
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

  Stat = canBus.begin(CAN_SPEED_500, CAN_MODE_NORMAL);    // Other speeds go from 125 kbps to 1000 kbps. CAN allows even more choices.
  if (Stat != CAN_OK){
       u8g2.print("ERROR CAN BUS Stuck");
       u8g2.print(Stat,DEC);
  }
  //filter(uint8 idx, uint32 id, uint32 mask, uint32 extID)
  //canBus.filter(0, 0x00ffee00, 0x1fffffff,0);          // filter don't work for ID 0x00FFEE00
  canBus.filter(0, DEFAULT_MASK, 0,0);                // listen for OBD req, ack simple 
  canBus.set_irq_mode();              // Use irq mode (recommended), so the handling of incoming messages
                                      // will be performed at ease in a task or in the loop. The software fifo is 16 cells long, 
                                      // allowing at least 15 ms before processing the fifo is needed at 125 kbps                                  
  Stat = canBus.status();
  if (Stat != CAN_OK)
  {
       u8g2.print("ERROR CAN system -");
       u8g2.print(Stat,DEC);
  }
  else
   u8g2.print("CAN Init Success ");
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
   // CANSetup() ;
    
    }
  return mbx ;
}

//Blink led with being stuck in a loop
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
  digitalWrite(LED_CAN_MUTE, LOW);    // turn the onboard LED on - also tied to /CAN Enable - Tie low or floating for CAN enable
  CANsend(&msg) ;      // Send this frame
  delay(180);              
 // digitalWrite(LED_CAN_MUTE, HIGH);   // turn the LED off 
 // delay(100);  
  //Serial1.println("endsend");
  /*
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
    */      
}


void CAN_Change()
{
  int Button_Key =0;
  CAN_STATUS Stat ;
  
  Button_Key = u8g2.userInterfaceSelectionList("CAN Speed Options", 1, "CAN 500\nCAN 250");
  switch (Button_Key) {
       case 0:
      //do something when var equals 1
      
       Serial1.println("Select cancel");
      break;
      case 2:
      Serial1.println("can_change_250");
      Stat = canBus.begin(CAN_SPEED_250, CAN_MODE_NORMAL);
        if (Stat != CAN_OK){
       Serial1.println("ERROR begin, CAN BUS Stuck");
       Serial1.println(Stat,DEC);
  }   
      break;
      case 3:
      //do something when var equals 2
  //    Serial1.println("option 2");
      break;      
      case 4:
      //do something when var equals 2
  //    Serial1.println("option 3");
      break;  
      case 5:
      //do something when var equals 2
  //    Serial1.println("option 4");
      break;  
      case 6:
      //do something when var equals 2
  //    Serial1.println("option 5");
      break;  
      case 7:
      //do something when var equals 2
   //   Serial1.println("option 6");
      break;  
      default:
      
      
      // if nothing else matches, do the default
      // default is optional
      Serial1.println("can_change_500");
      Stat = canBus.begin(CAN_SPEED_500, CAN_MODE_NORMAL);
        if (Stat != CAN_OK){
       Serial1.println("ERROR begin, CAN BUS Stuck");
       Serial1.println(Stat,DEC);
  }
      break;
  }
}



  /**** ISO 9141-2. This protocol has an asynchronous serial data rate of 10.4 kBaud. It is somewhat similar to RS-232, but that the signal levels are different, and that communications happens on a single, bidirectional line without extra handshake signals. ISO 9141-2 is primarily used in Chrysler, European, and Asian vehicles.
   *  
   *    pin 7: K-line
   *    pin 15: L-line (optional)
   *    UART signaling (though not RS-232 voltage levels)
   *    K-line idles high 
   *    High voltage is Vbatt
   *    Message length is restricted to 12 bytes, including CRC
   * 
   *  Pinos: 5,7,15 e 16 ativos - Nesse Protocolo o pino 15 é opicional e pode não existir no veículo
   */

  /**** ISO 14230 KWP2000 (Keyword Protocol 2000)
   *
   *    pin 7: K-line
   *    pin 15: L-line (optional)
   *    Physical layer identical to ISO 9141-2
   *    Data rate 1.2 to 10.4 kBaud
   *    Message may contain up to 255 bytes in the data field 
   *  
   *  Pinos: 5,7,15 e 16 ativos - Nesse Protocolo o pino 15 é opicional e pode não existir no veículo
   */
   
 /**** SAE J1850 VPW (variable pulse width - 10.4/41.6 kB/sec, standard of General Motors)
   *
   *    pin 2: Bus+
   *    Bus idles low
   *    High voltage is +7 V
   *    Decision point is +3.5 V
   *    Message length is restricted to 12 bytes, including CRC
   *    Employs CSMA/NDA
   *  
   *  Pinos: 2,5 e 16 ativos
   */
   /**** SAE J1850 PWM (pulse-width modulation - 41.6 kB/sec, standard of the Ford Motor Company)
  *    pin 2: Bus+
   *    pin 10: Bus–
   *    High voltage is +5 V
   *    Message length is restricted to 12 bytes, including CRC
   *    Employs a multi-master arbitration scheme called 'Carrier Sense Multiple Access with Non-Destructive Arbitration' (CSMA/NDA)
   *
   *  Pinos: 2,5,10 e 16 ativos
   *
#define KWP_LOW              PA15 //in order to function disable STLINK port //Sets PIN15 on OBD  High 12v
#define KWP_TX               PA9 //ISO-9141 //Sets PIN7 on OBD  High 12v
#define KWP_RX               PA10//ISO-9141

   
*/
//placeholders

void Init__J1850_PWM(){}
void Init_SAE_J1850_VPW(){}
void Init_KWP2000(){} 
void Init_ISO9141_2(){}

#define K_HIGH(time) digitalWrite(KWP_TX,HIGH);delay(time);
#define K_LOW(time) digitalWrite(KWP_TX,LOW);delay(time);
/*
void K_LOW(int d_time)
{
  digitalWrite(KWP_TX,LOW);
  delay(d_time);
  digitalWrite(KWP_TX,HIGH);
}*/


void Init_KKL_ALT(boolean ALT_INIT)
{
  //baud is 10400 baud (8N1)
 // Serial1.begin(10400);
   afio_cfg_debug_ports(AFIO_DEBUG_NONE);//this enables PA15 for output 
// Do some KKL functions here, right now I have nothing

  if (ALT_INIT)
  {
        K_LOW(70); // Low for 70ms
        K_HIGH(130); // High for 130ms
  }
  else
  {
         K_LOW(25); // Low for 25ms
         K_HIGH(25); // High for 25ms
  }


   delay(1000);
   Serial1.end();
 //  afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ);//enable debug / in usage you will leave debug disable for KKL to always function and switch it on via dev menu
}

boolean Init_SLOW_KKL()
{
  char serial;
  K_HIGH(320); // As per ISO standard

  // Send 0x33 at 5 baud (00110011)
  // K-line level: |_--__--__-|
  K_LOW(200); // Low for 200ms - start bit
  K_HIGH(400); // High for 400ms - 00
  K_LOW(400); // Low for 400ms - 11
  K_HIGH(400); // High for 400ms - 00
  K_LOW(400); // Low for 400ms - 11
  K_HIGH(200); // High for 200ms - stop bit
  Serial1.begin(10400);
  delay(25);
  serial = Serial1.read();
return 0;
}



void SNIFF_Menu()
{
    CanMsg* local_msg = new CanMsg[6] ;
   int LCD_ROW = 0;
   int MAX_ROW = 6;
   int ACTUAL_ROW = 0;
   int LOOPING_NOW =0 ;
  u8g2.setFont(u8g2_font_5x7_tf);
  if (saved_ID_Filter!=DEFAULT_MASK)canBus.filter(0, saved_ID_Filter, 0x1fffffff,0); 
//  u8g2.setFont(u8g2_font_6x10_tf);
        while (digitalRead(Exit_Button)==1)
        {
           u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  if (saved_ID_Filter==DEFAULT_MASK)
                  u8g2.print("-----+CAN Sniffer+-----");
                  else 
                  {
                  u8g2.print("Current Filter:0x");
                  u8g2.print(saved_ID_Filter,HEX);
                  u8g2.print("  ");
                  }
                  
                  if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
                  
                     // long msgID = 0x7DF ;
                    //  SendCANmessage(r_msg->ID+9, 8, 0x04, 0x02, 0x02) ;
               //   u8g2.setFont(u8g2_font_5x7_tf);
                  //u8g2.print(2,6,r_msg->ID);
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                        LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }
                  if (old_ID != 0)
                    {
                //      u8g2.setCursor(2, 16);
               //       u8g2.print("Code $");
               if (LOOPING_NOW ==1)
               {
                 for(int i = 0;i < MAX_ROW+1
                 ; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                          u8g2.print(local_msg[i].Data[b],HEX);
                          u8g2.print(".");
                      }

                    }
               }
               else
                    for(int i = 0;i < ACTUAL_ROW; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                        u8g2.print(local_msg[i].Data[b],HEX);
                        u8g2.print(".");
                      }
                      
                    }
                 }
                  
            }while ( u8g2.nextPage() );
                
        } 
}
void Dump_menu()
{
  /*
   *      Serial1.print(F("Read "));
        int addr = 0x0
        int len = sizeof(can_log)*MAX_LOG;
        
        uint8_t buff[sizeof(can_log)*MAX_LOG];
         memset(buff,0,sizeof(can_log)*MAX_LOG);
         
        Serial1.print(F("addr=0x"));
        Serial1.print(addr>>8,HEX);
        Serial1.print(addr,HEX);
        Serial1.print(F(",len=0x"));
        Serial1.print(len>>8,HEX);
        Serial1.print(len,HEX);
        Serial1.println(F(":"));
        uint8_t *buf = new uint8_t[len];
        while(mem.busy());
        mem.read(addr,buf,len);
        memcpy(can_log,buff,(sizeof(can_log)*MAX_LOG));
        for(int i = 0;i < len; i++)
        {
          Serial1.print("[");
          Serial1.print(can_log[i].stamp_sr,DEC);
          Serial1.print("]");
          Serial1.print(can_log[i].Data_sr.ID,HEX);
           for(int c = 0;c < 7; c++)
           {
              Serial1.print(":");
              Serial1.print(can_log[i].Data_sr.Data[c],HEX);
           }
           Serial1.println();
        }
        Serial1.println();
        Serial1.println(F("OK"));
        delete [] buf;
   */
}
/*
 * u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                          u8g2.print(local_msg[i].Data[b],HEX);
                          u8g2.print(".");
                      }
 */
void dump_flash()
{
      int ACTUAL_ROW = 0;
      record_type* can_log = new record_type[30];
        uint8_t buff[256];
         memset(buff,0,256);
        
        
        while(mem.busy());
        mem.read(0,buff,256);
        while(mem.busy());
        memcpy(&can_log[0],buff,256);
        mem.read(256,buff,256);
        memcpy(&can_log[10],buff,256);
       // mem.read(0x1,buff,256);
      //  while(mem.busy());
      //  memcpy(can_log+256,buff,256);
        for(int i = 0;i < 15; i++)
        {
          Serial1.print("[");
          Serial1.print(can_log[i].Data_sr.ID,HEX);
          Serial1.print("]");
          for(int b = 0;b < 7; b++)
          Serial1.print(can_log[i].Data_sr.Data[b],HEX);
          Serial1.println();
        }
}

void Send_PID()
{
 CanMsg* local_msg = new CanMsg[6] ;
 CanMsg* local_msg_send = new CanMsg[6] ;
 int MAX_ROW = 4;
 int ACTUAL_ROW = 0;
 int Button_Key =0;
 int do_once=0;
//setup some messages to send
//standard pid 0x00
    local_msg_send[0].ID=0x7DF;
    local_msg_send[0].Data[0]=0x2;
    local_msg_send[0].Data[1]=0x0;
    local_msg_send[0].Data[2]=0;
    local_msg_send[0].Data[3]=0;
    local_msg_send[0].Data[4]=0;
    local_msg_send[0].Data[5]=0;
    local_msg_send[0].Data[6]=0;
    local_msg_send[0].Data[7]=0;
//pid 0x01
    local_msg_send[1].ID=0x7DF;
    local_msg_send[1].Data[0]=0x2;
    local_msg_send[1].Data[1]=0x1;
    local_msg_send[1].Data[2]=0;
    local_msg_send[1].Data[3]=0;
    local_msg_send[1].Data[4]=0;
    local_msg_send[1].Data[5]=0;
    local_msg_send[1].Data[6]=0;
    local_msg_send[1].Data[7]=0;
//pid 0x0
    local_msg_send[0].ID=0x7DF;
    local_msg_send[0].Data[0]=0x2;
    local_msg_send[0].Data[1]=0x2;
    local_msg_send[0].Data[2]=0;
    local_msg_send[0].Data[3]=0;
    local_msg_send[0].Data[4]=0;
    local_msg_send[0].Data[5]=0;
    local_msg_send[0].Data[6]=0;
    local_msg_send[0].Data[7]=0;
//SendCANmessage(r_msg->ID+9, 8, 0x04, 0x02, 0x02) ;
canBus.filter(0, 0x7fd, 0x1fffff00,0);
Button_Key = u8g2.userInterfaceSelectionList("PID Send Menu", 1, "Send PID 0x00\nSend PID 0x01\nSend PID 0x02\nSend PID 0x03");
switch (Button_Key) {
       case 0:
      //do something when var equals 1
       Serial1.println("Select cancel");
      break;
      case 1:
     while (digitalRead(Exit_Button)==1)
        {
           u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  u8g2.print("Sending 0x00");
                  if (do_once==0){SendCANmessage(0x7df, 8, 0x02, 0x00, 0x00);do_once++;}
                  
                  delay(150);
                if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
                  
                     // long msgID = 0x7DF ;
                    //  SendCANmessage(r_msg->ID+9, 8, 0x04, 0x02, 0x02) ;
               //   u8g2.setFont(u8g2_font_5x7_tf);
                  //u8g2.print(2,6,r_msg->ID);
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                      //  LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }
                  if (ACTUAL_ROW>0)
                  {
                     for(int i = 0;i < ACTUAL_ROW; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                        u8g2.print(local_msg[i].Data[b],HEX);
                        u8g2.print(".");
                      }
                      
                    }
                    u8g2.setCursor(2,55);
                    u8g2.print("Press Ent to send again");
                    
                  }
                }while ( u8g2.nextPage() );
              
              Serial1.print("menu button bit");
              //while (digitalRead(Menu_Button)==1){}
              while ((r_msg = canBus.recv()) == NULL){}
              do_once==1;
        }//end of while(digitalread
      case 2:
     while (digitalRead(Exit_Button)==1)
        {
           u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  u8g2.print("Sending 0x01");
                  if (do_once==0){SendCANmessage(0x7df, 8, 0x02, 0x01, 0x00);do_once++;}
                  
                  delay(150);
                if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                      //  LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }
                  if (ACTUAL_ROW>0)
                  {
                     for(int i = 0;i < ACTUAL_ROW; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                        u8g2.print(local_msg[i].Data[b],HEX);
                        u8g2.print(".");
                      }
                      
                    }
                    u8g2.setCursor(2,55);
                    u8g2.print("Press Ent to send again");
                    
                  }
                }while ( u8g2.nextPage() );
              
              Serial1.print("menu button bit");
              //while (digitalRead(Menu_Button)==1){}
              while ((r_msg = canBus.recv()) == NULL){}
              do_once==1;
        }//end of while(digitalread
      break;
      case 3:
     while (digitalRead(Exit_Button)==1)
        {
           u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  u8g2.print("Sending 0x02");
                  if (do_once==0){SendCANmessage(0x7df, 8, 0x02, 0x02, 0x00);do_once++;}
                  
                  delay(150);
                if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      //old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
                  
         
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                      //  LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }
                  if (ACTUAL_ROW>0)
                  {
                     for(int i = 0;i < ACTUAL_ROW; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                        u8g2.print(local_msg[i].Data[b],HEX);
                        u8g2.print(".");
                      }
                      
                    }
                    u8g2.setCursor(2,55);
                    u8g2.print("Press Ent to send again");
                    
                  }
                }while ( u8g2.nextPage() );
              
              Serial1.print("menu button bit");
              //while (digitalRead(Menu_Button)==1){}
              while ((r_msg = canBus.recv()) == NULL){}
              do_once==1;
        }//end of while(digitalread
      break;
      case 4:
      while (digitalRead(Exit_Button)==1)
        {
           u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  u8g2.print("Sending 0x04");
                  if (do_once==0){SendCANmessage(0x7df, 8, 0x02, 0x04, 0x00);do_once++;}
                  
                  delay(150);
                if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
         
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                      //  LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }
                  if (ACTUAL_ROW>0)
                  {
                     for(int i = 0;i < ACTUAL_ROW; i++)
                    {
                      u8g2.setCursor(2,((i*8))+16);
                      u8g2.print("[");
                      u8g2.print(local_msg[i].ID,HEX);
                      u8g2.print("]");
                      for(int b = 0;b < 7; b++)
                      {
                        u8g2.print(local_msg[i].Data[b],HEX);
                        u8g2.print(".");
                      }
                      
                    }
                    u8g2.setCursor(2,55);
                    u8g2.print("Press Ent to send again");
                    
                  }
                }while ( u8g2.nextPage() );
              
              Serial1.print("menu button bit");
              //while (digitalRead(Menu_Button)==1){}
              while ((r_msg = canBus.recv()) == NULL){}
              do_once==1;
        }//end of while(digitalread
        break;
      default:
      //do something
      break;
/*
    if ((r_msg = canBus.recv()) != NULL)
                  {
                      local_msg[ACTUAL_ROW].ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                        local_msg[ACTUAL_ROW].Data[i] = r_msg->Data[i];
                      
                      old_ID=r_msg->ID;
                      DataOne=r_msg->Data[0];             
                      DataTwo=r_msg->Data[1];             
                      DataThree=r_msg->Data[2];             
                      DataFour=r_msg->Data[3];             
                      DataFive=r_msg->Data[4];             
                      DataSix=r_msg->Data[5];             
                      DataSeven=r_msg->Data[6];             
                      DataEight=r_msg->Data[7];             
                  
                     // long msgID = 0x7DF ;
                    //  SendCANmessage(r_msg->ID+9, 8, 0x04, 0x02, 0x02) ;
               //   u8g2.setFont(u8g2_font_5x7_tf);
                  //u8g2.print(2,6,r_msg->ID);
                      if (ACTUAL_ROW>MAX_ROW)
                      { 
                        ACTUAL_ROW=1;
                      //  LOOPING_NOW=1;
                      }
                        else
                          ACTUAL_ROW++;
                        canBus.free();
         
                  }*/
                  }
}
void Tick_menu()//This is a ghetto time stamp solution, we only need to know what order the pid's come in, not really 'exatcly when' yet, maybe next feature
{
  int stamp;
  String time_stamp;
  String time_stamp_cut;
  int done = 0;
  int MAX_LOG = 20;
  String files = "trypetxt";
#define FILENAME_STRING_SIZE      13
 char filename[FILENAME_STRING_SIZE];
  record_type* can_log = new record_type[30];
  int MAX_ROW = 6;
  int addr =0;
  int ACTUAL_ROW = 0;

/*  typedef struct
 {
     int stamp_sr;
     CanMsg Data_sr ;
     
 }  record_type;
*/
Serial1.println("erasing blocks");
mem.WE();
//mem.eraseAll();
//mem.erase64kBlock(0);
long ss = millis();
long pp = millis();
while(mem.busy())
 {
          Serial1.println(millis()-ss);
          delay(1000);
        }
        mem.erase64kBlock(1);
         {
          Serial1.println(millis()-ss);
          delay(1000);
        }
Serial1.println("done");
  u8g2.firstPage();
    uint8_t buff[sizeof(can_log)*MAX_LOG];
     memset(buff,0,sizeof(can_log)*MAX_LOG);

     
  while (digitalRead(Exit_Button)==1)
    //while ((ACTUAL_ROW < MAX_LOG+2) ||(digitalRead(Exit_Button)!=1)||(done!=1))
  {
    if (done==1) return;
    u8g2.firstPage();
  
              do 
                {
                  u8g2.setCursor(2, 8);
                 
                  u8g2.print(" Ticks ");
                  stamp = micros();
                  time_stamp= String(stamp);
                  time_stamp_cut= time_stamp.substring(0, 3);
                    if ((r_msg = canBus.recv()) != NULL)
                  {
                    long msgID = 0x7DF ;
                      SendCANmessage(r_msg->ID+9, 8, 0x04, 0xde,0xad, 0xbe,0xef) ;
                       
                      // can_log[ACTUAL_ROW].stamp_sr = time_stamp_cut.toInt();
                     // for(int i = 0;i < 3; i++)
                      //can_log[ACTUAL_ROW].stamp_sr[i] = time_stamp.toCharArray(0, 3);
                    //  time_stamp_cut.toCharArray(can_log[ACTUAL_ROW].stamp_sr, 4);
                    
                       can_log[ACTUAL_ROW].stamp_sr=ACTUAL_ROW;
              
                      can_log[ACTUAL_ROW].Data_sr.ID = r_msg->ID;
                      for(int i = 0;i < 7; i++)
                       can_log[ACTUAL_ROW].Data_sr.Data[i] = r_msg->Data[i];
                       Serial1.print((millis()-pp));
                       Serial1.print("[");
                       Serial1.print(can_log[ACTUAL_ROW].Data_sr.ID,HEX);
                       Serial1.print("]");
                       for(int p=0;p <7;p++)
                       {
                            Serial1.print(can_log[ACTUAL_ROW].Data_sr.Data[p],HEX);
                            Serial1.print(":");
                       }
                       Serial1.println();
                       canBus.free();
                       ACTUAL_ROW++;
                  }


                  //stamp = stamp / 10000;
                  u8g2.print(time_stamp_cut);
                //  Serial1.print(" ");
                 // Serial1.print(time_stamp_cut);
                  
                  

                
                if ((ACTUAL_ROW > MAX_LOG)|(digitalRead(Up_Button)!=1))
                    {
                    ACTUAL_ROW=0;
                     
                    u8g2.setCursor(2, 20);
                    Serial1.println("flash write start");
                     while(mem.busy());
                     mem.WE();
                    uint8_t buf[256];
                    memset(buf,0,256);
                    memcpy(buf,can_log,256);
                    mem.writePage(0x0,buf);
                   // memcpy(buf,&can_log[256],256);
                    mem.writePage(256,buf);

                    //mem.writebyte(0x0,buf,256);
                    // mem.writePage(addr,buf);
                    ss = millis();
                    while(mem.busy())
                          {
                              Serial1.println(millis()-ss);
                              delay(1000);
                            }
            
                   ss = millis();
                   Serial1.println("Page two");
                   mem.writePage(addr+256,buf);
                    while(mem.busy())
                          {
                              Serial1.println(millis()-ss);
                              delay(1000);
                            }
        Serial1.println();
                      for(int i = 1;i < 256; i++)
                        {
                        Serial1.print("-");
                        Serial1.print(buf[i],DEC);
                        }
                    Serial1.println("finish");

                    done=1;
                   // return;
                    
                    }
                 
                }while ( u8g2.nextPage() );
                if (done==1) return;
  }
   //while ((done !0=) | (digitalRead(Exit_Button)!=1)){}
  delay(500);
}

void Cursor_Blink()

{
 // cur_state
 int speed_blink = 500;
    unsigned long currentMillis = millis();
    if (select_char)speed_blink=speed_blink/2;
  if (currentMillis - previousMillis_two >= speed_blink) {
    // save the last time you blinked the LED
    previousMillis_two = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (cur_state == false) {
      cur_state = true;
    } else {
      cur_state = false;
    }
  //Blink Cursor time or not
  //  return cur_state;
  }

 }
void CAN_Filter_Menu(String value_text,int array_size,uint32_t seed_value)
{
  int  ID_Filter[4]={0,0,0,0};
  int filter_Id=0;
  
  //int Mask_Filter[0x1,0xF,0xF,0xF,0xF,0x8,0x0,0x0]
  int Mask_Filter[8];
//  int char_select = 1;
  int cur_row =0;
 int b=1;
 int pending=0;
 int Button=0;
 int max_digits=4;
 String Filter_Name=value_text;
//seed_value =0x1234;
String value_split = String(seed_value,HEX);
 /*filll ID_Filter with initial seed, we will reuse the shiftin code
 filter_Id=filter_Id | (ID_Filter[0]<<12);
filter_Id=filter_Id | (ID_Filter[1]<<8);
 filter_Id=filter_Id | (ID_Filter[2]<<4);
filter_Id=filter_Id | (ID_Filter[3]);
 */
 char * pEnd;
 filter_Id=seed_value;
 if (value_split.length()==3)value_split="0"+value_split;
 if (value_split.length()==2)value_split="00"+value_split;
 if (value_split.length()==1)value_split="000"+value_split;
 if (value_split.length()==0);
Serial1.println(strtol(&value_split[2], &pEnd, 16));
Serial1.println(value_split[1]);
Serial1.println(value_split[0]);
Serial1.println(String(value_split.length()));


  ID_Filter[3]=strtol(&value_split[3], &pEnd, 16);
  ID_Filter[2]=(strtol(&value_split[2], &pEnd, 16));
  ID_Filter[2]=ID_Filter[2]>>4;
  ID_Filter[1]=(strtol(&value_split[1], &pEnd, 16));
  ID_Filter[1]=ID_Filter[1]>>8;
  ID_Filter[0]=(strtol(&value_split[0], &pEnd, 16));
  ID_Filter[0]=ID_Filter[0]>>12;

 
   //ID_Filter[4]=strtol(&value_split[4],&pEnd ,16);
  // ID_Filter[1]=strtol(&value_split[1], &pEnd, 16);
   // ID_Filter[0]=strtol(&value_split[0], &pEnd, 16);
 // ID_Filter[0]=int(value_split[0]) %20;
 //ID_Filter[3]=seed_value % 10;
 //ID_Filter[3]=(seed_value 10) % 10;
 //ID_Filter[2]<< ((seed_value>>4) % 10);
 //ID_Filter[1]<<(seed_value>>8) % 10;
 //ID_Filter[0]=(seed_value>>12) % 10;

 
 //ID_Filter[1]=ID_Filter[1] | (seed_value>>8);
 //ID_Filter[2]=ID_Filter[2] | (seed_value>>8);
// ID_Filter[3]=ID_Filter[3] | (seed_value);
  //start with just can ID filter
  //u8g2 does not have built in function for this so we will make one
 //now that we have a working function, we should swap hard coded numbers for passed variables, that way we can resuse this code
   
   while ((digitalRead(Exit_Button)==1) && (digitalRead(Exit_Button)==1) )
      {
              u8g2.firstPage();
              do 
                {
                  Cursor_Blink();
                  u8g2.setCursor(2, 8);
                  u8g2.print(Filter_Name);
                  u8g2.setCursor(2,18);
                  u8g2.print("0x");
                  for ( b=0;b<max_digits;b++)
                    {
                     if (cur_row==b)
                     {
                        if (cur_state)
                        {
                          u8g2.setFontMode(0);
                          u8g2.setDrawColor(0);
                          if (select_char) { u8g2.setFontMode(0);
                                            u8g2.setDrawColor(0);
                                            u8g2.print(ID_Filter[b],HEX);
                                          u8g2.setFontMode(0);
                                         u8g2.setDrawColor(1);}
                          else
                          {
                            u8g2.print(ID_Filter[b],HEX);
                            u8g2.setFontMode(0);
                            u8g2.setDrawColor(1);
                          }
                          
                        }
                        else u8g2.print(ID_Filter[b],HEX);
                     }
                     else 
                     {
                      u8g2.setFontMode(0);
                      u8g2.setDrawColor(1);
                      u8g2.print(ID_Filter[b],HEX);
                     }
                     
                    }
                    u8g2.setCursor(2, 50);
                    u8g2.print("Saved Filter ID ");
               // 


 
                    
                    if (digitalRead(Exit_Button)!=1)
                    {
                      int Exit_delay =0;
                       if ((digitalRead(Exit_Button)!=1) && (digitalRead(Menu_Button)!=1)){break;}
                       while (digitalRead(Exit_Button)!=1){Exit_delay++;delay(25);Serial1.println(Exit_delay);if (Exit_delay>=50)return;}
                     
                     
                      Button = u8g2.userInterfaceMessage("Store Filter", "Press Ent To Store", "Exit To Cancel", " Ok \n Cancel ");
                      delay(150);
                      Exit_delay =0;
                      switch (Button) {
                                          case 0:
                                          //Cancel chosen
                                           break;
                                           case 2:
                                          //Exit chosen 
                                           break;
                                           default:
                                                 filter_Id=0;
                                                filter_Id=filter_Id | (ID_Filter[0]<<12);
                                                filter_Id=filter_Id | (ID_Filter[1]<<8);
                                                filter_Id=filter_Id | (ID_Filter[2]<<4);
                                                filter_Id=filter_Id | (ID_Filter[3]);
                                                saved_ID_Filter=filter_Id;
                                                canBus.filter(0, filter_Id, 0x1fffffff,0);
                                                
                                               Serial1.print("Filter saved ");
                                               for (int o=0;o<=3;o++){Serial1.print(ID_Filter[o],HEX);Serial1.print(":");}
                                               Serial1.print(filter_Id,HEX);
                                               break;
                                         }
                    }
                    if (digitalRead(Up_Button)!=1){
                    
                      if (select_char==1)
                          {
                            ID_Filter[cur_row]++;
                            Serial1.println("incrment");
                            if (ID_Filter[cur_row]>15)ID_Filter[cur_row]=15;
                            
                            while (digitalRead(Up_Button)!=1){delay(150);
                          }
                        }
                        else 
                        {cur_row++;if (cur_row==max_digits)cur_row=max_digits-1;while (digitalRead(Up_Button)!=1){delay(150);}}}
                        
                    if (digitalRead(Down_Button)!=1){if (select_char)
                        {ID_Filter[cur_row]--;
                        if (ID_Filter[cur_row]<0)ID_Filter[cur_row]=0;
                        while (digitalRead(Down_Button)!=1){delay(150);}
                        }
                        else                  
                          {cur_row--;if (cur_row==-1)cur_row=0;while (digitalRead(Down_Button)!=1){delay(150);}}} 
                          
                    if (digitalRead(Menu_Button)!=1)
                      {
                       if (select_char == false)  
                       {
                          select_char= true;pending=true;
                            } 
                            else 
                            {
                              select_char = false;
                            }
                        while (digitalRead(Menu_Button)!=1){delay(150);}
                      }
                   u8g2.print(filter_Id,HEX); 
                } while ( u8g2.nextPage() );
     }  
  
}

void ADC_Menu()
{
   while (digitalRead(Exit_Button)==1)
      {
              u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 8);
                  u8g2.print("ADC Values- ");
                  u8g2.setCursor(2, 16);
                  u8g2.print("PC0/1/2/3-PC4/PC5");
                  u8g2.setCursor(2, 25);
                  u8g2.println((analogRead(PC0)*3300)/4096);u8g2.println(" ");
                  u8g2.println((analogRead(PC1)*3300)/4096);u8g2.println(" ");
                  u8g2.print((analogRead(PC2)*3300)/4096);u8g2.println(" ");
                  u8g2.print((analogRead(PC3)*3300)/4096);
                  u8g2.setCursor(2, 33);
                  u8g2.print((analogRead(PC4)*3300)/4096);u8g2.println(" ");
                  u8g2.print((analogRead(PC5)*3300)/4096); 
                  if (digitalRead(Exit_Button)!=1){break;}  
                } while ( u8g2.nextPage() );
     }  
}

void Settings_menu()
{
  int selection = 0;
  int old_contrast = 0;
  uint8_t contrast_value =0;
  
    u8g2.setFont(u8g2_font_crox2hb_tf);
    u8g2.setCursor(2, 8);
    u8g2.println("Press Up Or Down to adjust contrast");
    while (digitalRead(Exit_Button)==1)
      {
              u8g2.firstPage();
              do 
                {
                  u8g2.setCursor(2, 16);
                  u8g2.setFontMode(0);
                  u8g2.setDrawColor(1);
                  u8g2.print("Contrast - ");
                  u8g2.setFontMode(1);
                  u8g2.setDrawColor(1);
                  u8g2.print(contrast_value);
                  if (digitalRead(Up_Button)!=1){contrast_value++;delay(150);}
                  if (digitalRead(Down_Button)!=1){contrast_value--;delay(150);}
                  u8g2.setContrast(contrast_value);
                  if (digitalRead(Exit_Button)!=1){break;}  
                } while ( u8g2.nextPage() );
     }
  
}

void dev_menu()
{
  int Button_Key =0;
  u8g2.setFont(u8g2_font_6x10_tf);
afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ);//this edisables PA15/PA13 for output // gives testers a option to reflash via STLINK
Button_Key = u8g2.userInterfaceMessage("STLINK Enabled", "Press OK To END", "*Disables CAN*", " Ok \n Cancel ");
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
Serial1.println("USB Menu Finished");
  while (digitalRead(Exit_Button)!=1){}
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);//this enables PA15/PA13 for output 
}





void Main_Menu()
{
  int Button_Key =0;
  uint8_t PID_P1 = 0;
  uint8_t PID_P2 = 0;
  uint8_t PID_P3 = 0;
  int input_val = 0;
  u8g2.setFont(u8g2_font_6x10_tf);
//  u8g2.setFont(u8g2_font_5x7_tf);
//u8g2.setFontRefHeightAll();    /* this will add some extra space for the text inside the buttons */
//u8g2.userInterfaceMessage("Boot STM32", "darkspr1te 2018", "USB/CAN Switch", " Ok \n Cancel ");
Button_Key = u8g2.userInterfaceSelectionList("Darkspr1te OM127 Menu", 1, "Change CAN Speed\nChange CAN bits\nPID Menu\nSniff Menu\nADC Menu\nDump Menu\nSettings Menu\Developer Menu");
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
 //     Serial1.println("option 2");
      input_val = u8g2.userInterfaceInputValue("Select PID Value  0x?00", "PID Value 1=", &PID_P1, 5, 1, 1, " hex");
      if (input_val != 1)
          break;
      input_val = u8g2.userInterfaceInputValue("Select PID Value  0x??0", "PID Value 2=", &PID_P2, 1, 5, 1, " hex");
      if (input_val != 1)
         break;
      input_val = u8g2.userInterfaceInputValue("Select PID Value  0x???", "PID Value 3=", &PID_P3, 1, 5, 1, " hex");
      if (input_val != 1)
         break;
    //do calcs to combine pid into one figure    
      break;      
      case 4:

   //   Serial1.println("Sniff Menu");
      SNIFF_Menu();
      break;  
      case 5:

  //    Serial1.println("ADC_Menu");      
      ADC_Menu();
      break; 
      case 6:     
  //    Serial1.println("Logging Options");      
      Tick_menu();
      break;
      case 7:
   //   Serial1.println("Settings");      
      Settings_menu();
      break;
      case 8:
    //  Serial1.println("Developer Mode- Enable STLINK port (disables CAN_MUTE/KK Line Differential");      
      dev_menu();
      break;  
      default:
  // if nothing else matches, do the default
  // default is optional
      Serial1.println("can_change");
      CAN_Change();

      break;
  }
Serial1.println("Main_Menu finished");
 while (digitalRead(Exit_Button)!=1){delay(100);}
}

void option_Page_One()
{
  int Button_Key =0;
  u8g2.setFont(u8g2_font_6x10_tf);

Button_Key = u8g2.userInterfaceMessage("Enable USB STM32", "darkspr1te 2018", "*Disables CAN*", " Ok \n Cancel ");
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
Serial1.println("USB Menu Finished");
  while (digitalRead(Exit_Button)!=1){}
}




//SPI Flash Stuff
void read_buffer(){
        int addr = Serial1.parseInt();
        int len = Serial1.parseInt();
        Serial1.print(F("addr=0x"));
        Serial1.print(addr>>8,HEX);
        Serial1.print(addr,HEX);
        Serial1.print(F(",len=0x"));
        Serial1.print(len>>8,HEX);
        Serial1.print(len,HEX);
        Serial1.println(F(":"));
        uint8_t *buf = new uint8_t[len];
        while(mem.busy());
        mem.read(addr,buf,len);
        for(int i = 0;i < len; i++)
        {
          Serial1.print((char)buf[i]);
        } 
}
void setup(void) {

/* SET PA8 output for LED 
 *  
#define Up_Button PB15
#define Down_Button PB12
#define Exit_Button PB13
#define Menu_Button PB14
#define USB_Detect PC15
#define SPI_SLAVE_SEL_PIN PA4


 */
   Serial1.begin(115200);//test , in usage UART1 wont be available

Serial1.println();
   Serial1.println("-------------------");
   Serial1.println("");
   Serial1.println("");
  Serial1.println("Serial1 OK Ready");//test , in usage UART1 wont be available
 //SPI Setup -currently causes a bug, i think it's erasing the system eeprom 
 

  //Button Setup
  pinMode(Down_Button, INPUT);//DownPB12
  pinMode(Exit_Button, INPUT);//Exit/PB13
  pinMode(Menu_Button, INPUT);//Enter/PB14
  pinMode(Up_Button, INPUT);//Up
  pinMode(USB_Detect, INPUT);//USB Plug detect

// testing ADC
pinMode(PA0, INPUT_ANALOG);
pinMode(PC1, INPUT_ANALOG);
pinMode(PC2, INPUT_ANALOG);
pinMode(PC3, INPUT_ANALOG);
pinMode(PC4, INPUT_ANALOG);
pinMode(PC5, INPUT_ANALOG);
  pinMode(SPI_SLAVE_SEL_PIN ,OUTPUT);  
  //Normally PA13, routed to PA8 for tempory CAN_MUTE control 
  pinMode(LED_CAN_MUTE, OUTPUT);
  digitalWrite(LED_CAN_MUTE,LOW);
  //PA13 used by SWDIO debug right now, later we have to remap to gain control to revert to factory board setup

  
//begin(uint8_t menu_select_pin, uint8_t menu_next_pin, uint8_t menu_prev_pin, uint8_t menu_up_pin = U8X8_PIN_NONE, uint8_t menu_down_pin = U8X8_PIN_NONE, uint8_t menu_home_pin = U8X8_PIN_NONE)
// init LCD with Menu buttons tied in
  u8g2.begin(Menu_Button,Down_Button,Up_Button,U8X8_PIN_NONE,U8X8_PIN_NONE,Exit_Button);
  u8g2.setContrast(0);//in usage remove delay from setup procedure, current shows lcd is working by going black,clear then text.maybe turn into boot routine. maybe add a logo
  u8g2.setCursor(2, 7);
    SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
 SPI.setDataMode(SPI_MODE0);
Serial1.println("testing 4");
  u8g2.firstPage();
  do{
          u8g2.setFont(u8g2_font_5x7_tf);
          u8g2.setCursor(2, 15);
          if(mem.begin(_W25Q64,SPI,SPI_SLAVE_SEL_PIN))
          {
              LOG_ENABLED = 1;
              u8g2.print("Flash OK, Logging allowed");
          }
           else
          {
          u8g2.print("Flash init FAILED, cannot log data");
          LOG_ENABLED = 0;
   
          }

          u8g2.setCursor(2, 22);

          CANSetup(); 

  
//Debug data, print clock speed to confirm CAN bitrates
          u8g2.setCursor(2, 30);
          u8g2.print("CPU Speed MHZ ");
          u8g2.print(F_CPU/1000000);
          u8g2.setCursor(2, 38);
          u8g2.print("PCLK1 speed MHZ ");
          u8g2.print(PCLK1/1000000); 
          u8g2.setCursor(2, 45);
          u8g2.print("PCLK2 speed MHZ ");
          u8g2.println(PCLK1/1000000); 
          u8g2.setCursor(10, 52);
   
          u8g2.print("PRESS ENTER TO CONTINUE");
    } while ( u8g2.nextPage() );
      if (digitalRead(Exit_Button)!=1)
  {
     Serial1.println("erasing flash");
     mem.WE();
    mem.eraseAll();
    while(mem.busy());
    Serial1.println("done");
  }
  dump_flash();
    while (digitalRead(Menu_Button)==1){}
  delay(500);
  while (digitalRead(Menu_Button)!=1){}
 // afio_cfg_debug_ports(AFIO_DEBUG_NONE);//this enables PA15/PA13 for output- looses stlink
   pinMode(CAN_MUTE, OUTPUT);
  digitalWrite(CAN_MUTE,LOW);
     pinMode(KWP_LOW , OUTPUT);
  digitalWrite(KWP_LOW ,LOW);
}


void loop(void) {
  u8g2.firstPage();
  do {
    //test , in usage UART1 wont be available as it controls kline
  /*   if (Serial1.available() > 0) {
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
       */
      if (digitalRead(Up_Button)!=1){
            while (digitalRead(Up_Button)!=1){}
                //nothing right now
                //SNIFF_Menu();
                Send_PID();
                // dump_flash();
            //Serial1.print("Up Pressed");
        
      }
      if (digitalRead(Menu_Button)!=1){
          while (digitalRead(Menu_Button)!=1){}
                Main_Menu();
                }
      if (digitalRead(Down_Button)!=1){
              while (digitalRead(Down_Button)!=1){}
                //nothing right now   
               // ADC_Menu();  
               CAN_Filter_Menu("ID_Filter",4,saved_ID_Filter); 
               //Tick_menu();          
           // Serial1.print("Down Pressed");
      }
      if (digitalRead(Exit_Button)!=1){
          while (digitalRead(Exit_Button)!=1){}          
            //    box_two(); nothing right now
      }
      if (digitalRead(USB_Detect)==1)
        {
          afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ);//this enables PA15 for output
          option_Page_One();
        }
   u8g2.setFont(u8g2_font_6x10_tf);     
   // u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2,9,"debug-OM127 OSS");
    //u8g2.setCursor(2, 15);
    u8g2.drawStr(2,17,"ENTER = Menu");
    u8g2.drawStr(2,25,"UP = CAN Sniffer");
    u8g2.drawStr(2,33,"DOWN = ADC Check");
    u8g2.drawStr(2,41,"EXIT = Nothing");
    

   // u8g2.print(incomingByte,DEC);
    u8g2.setCursor(2, 30);
    //USB power detect will allow 'upgrades' via the USB bootloader which shares mem/pins with CAN, in setup if USB_Detect >0 then activate serial and not CAN, possible quick way of dumping flash log   
    u8g2.drawFrame(0,0,128,64);// lcd is 128,64 
 /*    if (old_ID != 0){
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
//display canbus message, need to move into seperate area
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
                  SendCANmessage(r_msg->ID+9, 8, 0x04, 0x00, 0x00) ;
               //   u8g2.setFont(u8g2_font_5x7_tf);
                  //u8g2.print(2,6,r_msg->ID);

                  canBus.free();
         
        }
   */ 
  } while ( u8g2.nextPage() );
}


