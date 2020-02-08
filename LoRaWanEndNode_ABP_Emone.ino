
#include "SHT20.h"
#include <ModbusMaster.h>

#include <WiMODLoRaWAN.h>
#ifndef HAVE_HW_SERIAL1
#include <HardwareSerial.h>

HardwareSerial mySerial(1); //
HardwareSerial modbus(2);
ModbusMaster node;
#endif

struct Meter
{
  uint16_t temp;
  uint16_t hum;

};

Meter meter ;

#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#endif
#define SERIAL1_RXPIN 23
#define SERIAL1_TXPIN 19
//HardwareSerial MySerial(1);
//-----------------------------------------------------------------------------
// constant values
//-----------------------------------------------------------------------------

/*
   ABP Parameters
*/

const uint32_t  DEV_ADR = 0x74802109; //74802109

// network session key (128bit)
const unsigned char NWKSKEY[] = { 0x28, 0xAE, 0xD2, 0x2B, 0x7E, 0x15, 0x16, 0xA6, 0x09, 0xCF, 0xAB, 0xF7, 0x15, 0x88, 0x4F, 0x3C };

// application session key (128bit)
const unsigned char APPSKEY[] = { 0x16, 0x28, 0xAE, 0x2B, 0x7E, 0x15, 0xD2, 0xA6, 0xAB, 0xF7, 0xCF, 0x4F, 0x3C, 0x15, 0x88, 0x09 };

//1296D01174802109  74802109


//-----------------------------------------------------------------------------
// user defined types
//-----------------------------------------------------------------------------

typedef enum TModemState
{
  ModemState_Disconnected = 0,
  ModemState_ConnectRequestSent,
  ModemState_Connected,
  ModemState_FailedToConnect,
} TModemState;


typedef struct TRuntimeInfo
{
  TModemState ModemState;
} TRuntimeInfo;


//-----------------------------------------------------------------------------
// section RAM
//-----------------------------------------------------------------------------

/*
   Create in instance of the interface to the WiMOD-LR-Base firmware
*/
WiMODLoRaWAN wimod(mySerial);  // use the Arduino Serial3 as serial interface

TRuntimeInfo RIB = {  };

static uint32_t loopCnt = 0;
static TWiMODLORAWAN_TX_Data txData;

 
 
//-----------------------------------------------------------------------------
// section code
//-----------------------------------------------------------------------------

/*****************************************************************************
   Function for printing out some debug infos via serial interface
 ****************************************************************************/
void debugMsg(String msg)
{
  Serial.print(msg);  // use default Arduino serial interface
}

void debugMsg(int a)
{
  Serial.print(a, DEC);
}

void debugMsgChar(char c)
{
  Serial.print(c);
}

void debugMsgHex(int a)
{
  Serial.print(a, HEX);
}

/*****************************************************************************
   print out a welcome message
 ****************************************************************************/
void printStartMsg()
{
  debugMsg(F("==================================================\n"));
  debugMsg(F("This is FileName: "));
  debugMsg(F(__FILE__));
  debugMsg(F("\r\n"));
  debugMsg(F("Starting...\n"));
  debugMsg(F("This simple demo will try to "));
  debugMsg(F("do the ABP procedure and "));
  debugMsg(F("send a demo message each 30 sec.\n"));
  debugMsg(F("==================================================\n"));
}

/*****************************************************************************
   Arduino setup function
 ****************************************************************************/
void setup()
{

  mySerial.begin(115200, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);
  wimod.begin();

  // debug interface
  Serial.begin(115200);
  Serial.println("Start");
  printStartMsg();

  // do a software reset of the WiMOD
  delay(100);
  //    wimod.Reset();
  delay(100);
  // deactivate device in order to get a clean start for this demo
  wimod.DeactivateDevice();

  // do a simple ping to check the local serial connection
  debugMsg(F("Ping WiMOD: "));
  if (wimod.Ping() != true) {
    debugMsg(F("FAILED\n"));
  } else {
    debugMsg(F("OK\n"));
    //AS923 Thailand radio config variable
    TWiMODLORAWAN_RadioStackConfig radioCfg;
    // setup new config
    radioCfg.DataRateIndex   = LoRaWAN_DataRate_AS923_LoRa_SF7_125kHz;
    radioCfg.TXPowerLevel    = 16;
    radioCfg.Options         = // LORAWAN_STK_OPTION_ADR |
      LORAWAN_STK_OPTION_DEV_CLASS_C |
      LORAWAN_STK_OPTION_EXT_PKT_FORMAT;
    radioCfg.PowerSavingMode = LORAWAN_POWER_SAVING_MODE_OFF;
    radioCfg.Retransmissions = 7;
    radioCfg.BandIndex       = LoRaWAN_FreqBand_AS_923_Thailand;
    // set new radio config
    if (wimod.SetRadioStackConfig(&radioCfg) != true)
      debugMsg(F("Radio config failed\n"));
    else
      debugMsg(F("Radio config OK\n"));

    // try to register the device at network server via OTAA procedure
    debugMsg(F("Starting join ABP procedure...\n"));
    TWiMODLORAWAN_ActivateDeviceData activationData;

    //setup ABP data
    activationData.DeviceAddress = DEV_ADR;
    memcpy(activationData.NwkSKey, NWKSKEY, 16);
    memcpy(activationData.AppSKey, APPSKEY, 16);


    // activate device
    if (wimod.ActivateDevice(activationData)) {
      debugMsg(F("ABP procedure done\n"));
      debugMsg(F("(An 'alive' message has been sent to server)\n"));
      RIB.ModemState = ModemState_Connected;
    } else {
      debugMsg("Error executing ABP join request: ");
      debugMsg((int) wimod.GetLastResponseStatus());
      RIB.ModemState = ModemState_FailedToConnect;
    }
  }

}


/*****************************************************************************
   Arduino loop function
 ****************************************************************************/



void readMeter() {


  read_Modbus(data_) ;

  delay(2000);
}
void read_Modbus(uint16_t  REG)
{

  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint16_t dat[2];
  uint32_t value = 0;


  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readInputRegisters(REG, 2);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
  }
  for (int a = 0; a < 2; a++)
  {
    Serial.print(data[a]);
    Serial.println("\t");
  }
  Serial.println("Modbus DATA");
  meter.temp = data[0];
  meter.hum = data[1];
  Serial.println(meter.temp);
  Serial.println(meter.hum);
  Serial.println("----------------------");


}
void loop()
{
  // check of ABP procedure has finished
  if (RIB.ModemState == ModemState_Connected) {

//    Serial.println(loopCnt);
    
    if ((loopCnt > 0) && (loopCnt % (6 * 50)) == 0) {
      // send out a simple HelloWorld messsage
      readMeter();

      // prepare TX data structure
      txData.Port = 0x22;
      txData.Payload[0] = meter.temp>>8;
      txData.Payload[1] = meter.temp;
      txData.Payload[2] = meter.hum>>8;
      txData.Payload[3] = meter.hum;
      
//      debugMsg(txData.Payload);
      txData.Length = 4;
      
      // try to send a message
      if (false == wimod.SendUData(&txData)) {
        // an error occurred

        // check if we have got a duty cycle problem
        if (LORAWAN_STATUS_CHANNEL_BLOCKED == wimod.GetLastResponseStatus()) {
          // yes; it is a duty cycle violation
          // -> try again later
          debugMsg(F("TX failed: Blocked due to DutyCycle...\n"));
        }
      }
    }
  }

  // check for any pending data of the WiMOD
  wimod.Process();

  delay(40);
  loopCnt++;
}
