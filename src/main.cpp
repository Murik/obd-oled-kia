#include <Arduino.h>
//Kia RIO3 Display indication

#include "mcp_can.h"
#include <SPI.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h>

#include "U8g2lib.h"

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED

// #define u8g_logo_width 19
// #define u8g_logo_height 16
static const unsigned char u8g_cruise_bits[] U8X8_PROGMEM = {
    0x85,    0x0F,    0x00,
    0x66,    0x32,    0x00,
    0x17,    0x42,    0x00,
    0x08,    0x80,    0x00,
    0x14,    0x40,    0x01,
    0x22,    0x00,    0x02,
    0x42,    0x00,    0x02,
    0x81,    0x02,    0x04,
    0x01,    0x07,    0x04,
    0x87,    0x0D,    0x07,
    0x01,    0x07,    0x04,
    0x01,    0x02,    0x04,
    0x02,    0x00,    0x02,
    0x02,    0x00,    0x02,
    0x14,    0x40,    0x01,
    0x08,    0x80,    0x00};

#define OBDRate1    371             //ReadOBD rate (for fast changing parameters) real time 371ms
#define OBDRate2    795             //ReadOBD rate (for slow changing parameters) real time 795ms *3 - read only one parameter at a time
unsigned char flagRecv = 0;         //it is set when new data over the can is recieved (used by ChekDataRadio())
 static unsigned char IdxSlowRead=1;

unsigned int displayTimer = 0;
#define DisplayRate 7
static unsigned char slowDisplay=1;

int volumetricEff = 80;  // Volumetric efficiency on most modern vehicles should be about 85%
int engineDisp = 1591; // volume of anengine’s cylinders in cm3

//Global Variables Containing the OBD Parameters
// int EngOil = 80;
int ATFOil;                   // Gear Oil Temperature (CanID = 0x43F from  4 byte) 
// float EngLoad = 6.3;               // Engine Load (PID = 0x04) //not used but worked
int EngCool;                   // Engine Coolant Temperature (PID = 0x05)
unsigned int EngRPM;         // Engine RPM (PID = 0C)
unsigned int Speed;           // Speed (PID = 0D)
int AirTemp;                // Air Temp (PID = 46)
float BrdVoltage = 14.3;            //Voltage in a Board net (PID = 0x42)
// float FuelLvl = 0;  //	Fuel Tank Level (PID = 0x2F)

unsigned int MAP; // Intake manifold absolute pressure kPa
unsigned int IAT; // Intake air temperature °C
// float MAF = 100.0; // MAF air flow rate //not Work
float MAF_CALC; // MAF air flow rate calculated by formula
float FuelRate; // Engine fuel rate calculated cause 0x015E not response 

char cruiseStatus = 0x00; //0x32 -cruise on 0x36- cruise set

//Used PIDs 
// #define PID_OIL_TEMP        0x01    //(PID 2101)  // not tested
// #define PID_ENGIN_LOAD      0x04    //(PID 0104) //not used but worked
#define PID_COOLANT_TEMP    0x05    //(PID 0105)
// #define PID_ENGIN_PRM       0x0C    //(PID 010C) //not used but worked
// #define PID_VEHICLE_SPEED   0x0D    //(PID 010D) //not used but worked
// #define PID_TIMING_ADVANCE  0x0E    //(PID 010E) //not used but worked
#define PID_BOARD_VOLTAGE   0x42    //(PID 0142)
#define PID_AIR_TEMP   0x46    //(PID 0146)
#define PID_MAP   0x0B    //(PID 011B) 
#define PID_IAT   0x0F    //(PID 010F)
// #define PID_MAF   0x10    //(PID 0110) // not work
// #define PID_FUEL_LVL   0x2F    //(PID 012F) // not work
// #define PID_FUEL_RATE   0x5E    //(PID 015E) // not work

//USED CAN ID (ECU ID)
#define CAN_ID_PID         0x7DF

//CAN Shield definitions
#define CAN0_INT 2            // Set INT to pin 2
// #define MCP2551_Rs    5      //Pin PD5 - to be used to set the mode of MCP2551(CAN Driver) - Rs input
const int SPI_CS_PIN = 10;


//Global Timer Flag 1ms
unsigned char TmFlg;
unsigned char TmFlg1ms;

char msgString[128];
char speedStr[] = "---";

//define the pin where the button is connected (D3)
// #define But     3

unsigned char NoDataCounter;           //conts the missing data attempts (used to put MCU to sleep)

//Debug option
// boolean toggle = false;


MCP_CAN CAN0(SPI_CS_PIN);     // Set CS pin for the MCP2515


void set_mask_filt()
{
    //For Masks - when 0000 everything from this addres is alowed
  CAN0.init_Mask(0,0,0x7FF);                // Init first mask...
  CAN0.init_Filt(0,0,0x260);                // Init first filter... (for Cruise)
  CAN0.init_Filt(1,0,0x7E8);                // Init second filter...(ECU params read - RPM, Speed....)
  
  CAN0.init_Mask(1,0,0x7FF);                // Init second mask... 
  CAN0.init_Filt(2,0,0x316);                // Init third filter..(direct params RPM, Speed)
  CAN0.init_Filt(3,0,0x43F);                // Init fouth filter...(ATF Oil temp)
  CAN0.init_Filt(4,0,0x43F);                // Init fifth filter...(ATF Oil temp)
  CAN0.init_Filt(5,0,0x43F);                // Init sixth filter...(ATF Oil temp)

}

//************** Routine to form the CAN PID question and send it *********//
void sendPid(unsigned char __pid0, unsigned char __pid1)
{
    unsigned char tmp[8] = {0x02, __pid0, __pid1, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

void draw(void)
{
  // graphic commands to redraw the complete screen should be placed here
  u8g2.drawXBMP(0, 0, /*u8g_logo_width*/19, /*u8g_logo_height*/16, u8g_cruise_bits);
}

void MCP2515_ISR()
{
  flagRecv = 1;
}

ISR(TIMER1_OVF_vect)       
{
  TCNT1 = 63535;            // preload timer
  
  TmFlg=1;
  
}


void ScanTimer()
{
  if(TmFlg==1)
    {
      TmFlg1ms=1;
      TmFlg=0;
    }
    else
      TmFlg1ms=0;
}

void setup()
{
    // wdt_disable();
    // noInterrupts();
    // Serial.begin(115200);
    delay(2000);

      u8g2.begin();
      u8g2.enableUTF8Print();
    
    //to be able to filter data corectrly have to be in MCP_STDEX mode
    while (CAN_OK != CAN0.begin(CAN_500KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
    {
        // Serial.println("CAN BUS Shield init fail");
        // Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    // Serial.println("CAN BUS Shield init ok!");
    
    pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input
    set_mask_filt();
    CAN0.setMode(MODE_NORMAL);                           // Change to normal mode to allow messages to be transmitted

    // initialize timer1
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 63535;            // preload timer 65536-16MHz/256/2Hz
    TCCR1B |= (1 << CS11);    // 256 prescaler
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

    //initialize Digital input D3
    // pinMode(But, INPUT_PULLUP);

    //initialize Mode select Output for MCP2551 (Driver)
    // pinMode(MCP2551_Rs, OUTPUT); 
    // digitalWrite(MCP2551_Rs,LOW);     // Set Rs pin to Low (High Speed operation)                            
 
    // attachInterrupt(digitalPinToInterrupt(But), But_ISR, FALLING); // wakeUpNow when Button gets FALLING edge
    NoDataCounter = 0;                                                            

    attachInterrupt(digitalPinToInterrupt(CAN0_INT), MCP2515_ISR, FALLING);        // start interrupt from MCP2515
    
    interrupts();             // enable all interrupts

    // RadioDispFLG=0;
    
    // pinMode(4, OUTPUT);                                //only debug 
}

//***************** Routine to read the recieved mesage form block 01XX from ECU *******************
//*********** also update directly the global variable for corresponding parameter *****************
  void updateDisplay(){
    u8g2.clearBuffer();
    //круиз индикатор / скорость
                if (cruiseStatus==0x32) {   //КК включен
                    draw();
                    u8g2.setFont(u8g2_font_ncenB14_tn);
                    u8g2.drawStr(20, 16, speedStr);
                }
                else if (cruiseStatus==0x36) {   //SET включен
                  draw();
                   u8g2.setFont(u8g2_font_courB18_tn);
                   sprintf(speedStr, "%d", Speed);
                  u8g2.drawStr(20, 16, speedStr);
                }
                else  //все выключено
                {
                  u8g2.setFont(u8g2_font_courB18_tn);
                  u8g2.setCursor(20, 16);
                  u8g2.print(Speed);
                  }
     
    // u8g2.setFont(u8g2_font_crox3hb_tf);
    u8g2.setFont(u8g2_font_crox4h_tf); 
    char buff[10];
    u8g2.setCursor(1, 32);
    displayTimer++;
    if (displayTimer>DisplayRate)  //test whether the period has elapsed
  {
      if(slowDisplay==1) slowDisplay=2;
      else if (slowDisplay==2) slowDisplay=0;
      else slowDisplay=1;
      displayTimer=0;
  }
    switch (slowDisplay) {
      case 1:
            u8g2.print("C:");
            u8g2.print(EngCool);
            u8g2.print("°C");
      break;
      case 2:
            u8g2.print("A:");
            u8g2.print(ATFOil);
            u8g2.print("°C");
      break;
      default:
            dtostrf(BrdVoltage, 3, 1, buff);
            u8g2.print("V:");
            u8g2.print(buff);
      break;
    }

    // u8g2.setFont(u8g2_font_crox4h_tf);  
    //температура за бортом
    
    sprintf(buff, "%d", AirTemp);
    // const char *degree = buff;
    // u8g2.setDrawColor(1);
    u8g2.setCursor(120 - u8g2.getStrWidth(buff), 14);
    u8g2.print(buff); //degree
    u8g2.print("°");

    //средний / мгновенный расход
    dtostrf(FuelRate/3, 2, 1, buff);
      if (Speed >5 ){
        dtostrf(FuelRate*100.0/Speed, 2, 1, buff);
      
    }
 
    u8g2.setCursor(100 - u8g2.getStrWidth(buff), 32);  //fuel
    u8g2.print(buff);
    u8g2.setFont(u8g2_font_crox2h_tr);

    if (Speed >5 ){
      u8g2.print(" l/km");
    }else{
      u8g2.print(" l/h");
    }

  u8g2.sendBuffer();
}

void taskCanRecv()
{
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char buf[8];
    unsigned int timeout = 0;
    unsigned char DoneFL = 0;     //flag for done reading

    
    //stay in for certan time or till the mesage is recieved (DoneFL)
     do
      {
      timeout++;
      if(CAN_MSGAVAIL == CAN0.checkReceive())                   // check if get data
        {
          CAN0.readMsgBuf( &len, buf);    // read data,  len: data length, buf: data buf
            rxId = CAN0.getCanId();
            // sprintf(msgString, "Standard ID: 0x%.3lX  DLC: %1d  Data:", rxId, len);
            // Serial.print(msgString);
            // for(byte i = 0; i<len; i++){
                // sprintf(msgString, " 0x%02X", buf[i]);
                // Serial.print(msgString);
            // }
            // Serial.println();

          if(rxId == 0x7E8)
            {
              switch(buf[2])
                {
              //   case PID_ENGIN_LOAD:
              //   EngLoad = buf[3]/2.55;  
              //   Serial.print("EngLoad: " );
              //   Serial.print(EngLoad);
              //   Serial.println();
              //   break;
                case PID_COOLANT_TEMP:
                   EngCool = buf[3]-40;
                  //  Serial.println("------->EngCool: " + EngCool);
                break;
                // case PID_ENGIN_PRM:
              //   EngRPM = ((buf[3]*256)+buf[4])/4;  
              //   Serial.println("EngRPM: " + EngRPM);
                // break;
                case PID_BOARD_VOLTAGE:
                   BrdVoltage = ((buf[3]*256)+buf[4])/1000.0;
                  //  Serial.print("------>BrdVoltage: ");
                  //  Serial.println(BrdVoltage);
                break;
                // case PID_VEHICLE_SPEED:
              //   Speed = buf[3];  
              //   Serial.println(" Speed: " + Speed);
                // break;
                case PID_AIR_TEMP:
                   AirTemp = buf[3]-40;  
                  //  Serial.println("------>AirTemp: " + AirTemp);
                break;
            //  case PID_MAF:
              //   MAF = ((buf[3]*256)+buf[4])/100.0; 
              //   Serial.println("------>MAF: "+ MAF );
              //   break;
                case PID_MAP:
                   MAP = buf[3];  
              //   Serial.println("------>MAP: " + MAP);
                break;
                case PID_IAT:
                   IAT = buf[3]-40;  
              //   Serial.println("------>IAT: " +IAT);
                break;
            //  case PID_FUEL_LVL:
              //   FuelLvl = buf[3]/2.55;  
              //   Serial.println("----------->FuelLvl: "+ FuelLvl );
              //   Serial.println("Litres: " +FuelLvl * 43/100 ); //43 is tank volume for rio3
              // break;
            //  case PID_FUEL_RATE:
              //   FuelRate = ((buf[3]*256)+buf[4])/20.0;  
              //   Serial.print("------------->FuelRate: " + FuelRate);
              //   Serial.print("l/h : ");
              //   if (Speed != 0){
                  //   Serial.print(FuelRate*100/Speed +" l/100km");
              //   }
              //   Serial.println();
              // break;
                }
              DoneFL=1;   //set flag for done reading
            } else if (rxId == 0x260){
              cruiseStatus = buf[3];
              // Serial.print("------>Cruise: " );
              // Serial.println(buf[3], HEX);
            } else if (rxId == 0x43F){
              ATFOil = buf[4]-40;
              // Serial.println("------>ATFOil: " + ATFOil);
            } else if (rxId == 0x316){
              Speed = buf[6];  
              // Serial.println(" Speed: " +Speed);
              EngRPM = ((buf[3]*256)+buf[4])/4;  
              // Serial.println("EngRPM: " + EngRPM);
            }  
      }
    }
    while((timeout < 4000) && (DoneFL < 1));  //do till the time out expire or the data is recieved
    if(timeout > 3900){                        //chek if it was recieved data
      NoDataCounter++;                        //not recieved increase the conter
    }else
      NoDataCounter = 0;                      //recieved => zero the counter
}


// void But_ISR()
// {
//       DispMode++;
//       if(DispMode>4)
//       DispMode=0;
// }
//*************** Routine to read OBD Parameters **************
//Read some paramethers OBDRate1 times slower than update display
//Read other parameters OBDRate2 times slower 
void ReadOBD ()
{ 
 static unsigned int ReadOBDTm0;
 static unsigned int ReadOBDTm1;

  if(TmFlg1ms==1)
    {
      ReadOBDTm0++;
      ReadOBDTm1++;
    }
 
  if(ReadOBDTm0>OBDRate1)
    {
        ReadOBDTm0=0;
        // sendPid(0x01,PID_VEHICLE_SPEED);     //send request for the speed // get from internal messages
        // taskCanRecv();
        // sendPid(0x01,PID_MAF);         //send request for MAF air flow rate //not work
        // taskCanRecv();
        // sendPid(0x01,PID_ENGIN_PRM);         //send request for the Engine RPM // get from internal messages
        // taskCanRecv();
        // sendPid(0x01,PID_FUEL_RATE);     //send request for the Timing Advance //not work
        sendPid(0x01,PID_MAP);         //send request for the Intake manifold absolute pressure
        taskCanRecv();
        // sendPid(0x01,PID_ENGIN_LOAD);     //send request for Engine Load 
        // taskCanRecv();

        //calculate MAF
        float IMAP = (EngRPM * MAP) / ((IAT*0.98668 + 273) / 2.0); //273 -> Cel to Kelvin
        MAF_CALC  = (IMAP / 60.0) * (volumetricEff / 100.0) * (engineDisp / 1000.0) * (28.9644 / 8.314472);
        //R = 8.314 J/oK/mole.
        //MMAir = 28.97
        FuelRate = (MAF_CALC * 3600) / (14.7 * 820);
        //Density g/dm3 = 830 Gasoline /  750 Disel
        //Ratio by mass = 14.7 Gasoline /  14.5 Disel

        //0x7E8  DLC: 8  Data: 0x04 0x41 0x44 0x80 0x00 -> Fuel–Air commanded equivalence ratio =1;

        // Serial.print("------------->FuelRate: ");
        // Serial.print(FuelRate/3);
        // Serial.print("l/h : " );
        // if (Speed > 5){
        //    Serial.print(FuelRate*100/Speed);
        //    Serial.print(" l/100km");
        // }
        // Serial.println();
        updateDisplay();
    }
  else if(ReadOBDTm1>OBDRate2)
    {
    ReadOBDTm1=0;
    
    switch (IdxSlowRead) {
      
      case 0:
        sendPid(0x01,PID_COOLANT_TEMP);  //send request for Coolant temperature
        taskCanRecv();
        IdxSlowRead=1;
      break;

      case 1:
         sendPid(0x01,PID_BOARD_VOLTAGE); //send request for Board Voltage 
         taskCanRecv();
         IdxSlowRead=2;
      break;

      case 2:
         sendPid(0x01,PID_AIR_TEMP);      //send request for Air Temp
         taskCanRecv();
         IdxSlowRead=3;
      break;
      case 3:
        //  sendPid(0x01,PID_FUEL_LVL);      //send request for FuelTank
         sendPid(0x01, PID_IAT);            //	Intake air temperature
         taskCanRecv();
         IdxSlowRead=0;
      break;

      default:
        // if nothing else matches, do the default
        // default is optional
        IdxSlowRead=0;
      break;
    }
  
  }
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
     
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
                             
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    
    noInterrupts();
    // detachInterrupt(digitalPinToInterrupt(But));    //"Disable" the interrupt from the button (It will be enabled during the "reset")
    wdt_enable(WDTO_1S);
    delay(1000);                //stay here till RESET comes
    
}


void loop()
  {
  ScanTimer();

  ReadOBD();

  // delay (700);

  if(NoDataCounter>20)    //if data was not recieved 20 times -> sleep
    {
    detachInterrupt(digitalPinToInterrupt(CAN0_INT));
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_courB18_tr); // choose a suitable font
      u8g2.drawStr(0,30,"No Data");  // write something to the internal memory
      u8g2.sendBuffer(); 
    // Reset Manualy the MCP2515 CAN Controller
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_PIN, LOW);
    SPI.transfer(0xC0);               //Send "MCP_RESET"
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
    
    //Put CAN Controller in Sleep Mode
    CAN0.setMode(MODE_SLEEP);               // Change to sleep mode to save energy
    
    // digitalWrite(MCP2551_Rs,HIGH);        // Set Rs pin to High, Put MCP2551 in Standby mode
    sleepNow();                            //Put AtMega328P to sleep
    }
  
  TmFlg1ms=0;                             //clear the 1ms Flag
  
}









