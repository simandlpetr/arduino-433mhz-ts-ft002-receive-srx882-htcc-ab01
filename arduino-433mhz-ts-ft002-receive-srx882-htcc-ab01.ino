//author Petr Simandl www.simandl.cz
//Receiving TS-FT002 sensor data and sending to LoRaWan
//based on timing described on page
//https://github.com/merbanan/rtl_433/blob/ea797cff8cafd7fc385f8cf413f072df725f9e5e/src/devices/ts_ft002.c
//similar project can be found at https://github.com/theovassiliou/WTLMReceiver
//code developed on Heltec Cube Cell HTCC-AB01
//as 433Mhz receiver is used SRX882 Superheterodyne Receiver
//schematic 
//SRX882          HTCC-AB01
//pin 3 vcc       vs
//pin 5 data      gpio1
//pin 6 gnd       gnd
//usb serves as COM debug monitor and power supply
//TO DO
//- integration with alix APU centos hotsanic to get graphs

//thethingsnetwork.org payload format decoder example:
//function Decoder(bytes, port) {
//// Decode an uplink message from a buffer
//// (array) of bytes to an object of fields.
//var json={};
//json.sync=bytes[0];
//json.id=bytes[1];
//json.mtype=bytes[2];
//json.depth=((bytes[3] & 0xf0) >>4)*256+((bytes[3] & 0x0f))*16+((bytes[4] & 0x0f));
//json.tint=((bytes[4] & 0xf0) >>4);
//json.batt=((bytes[5] & 0xf0) >>4);
//json.temp=(((bytes[6] & 0xf0) >>4)*256+((bytes[6] & 0x0f))*16+((bytes[5] & 0x0f))-400)/10;
//json.rain=bytes[7];
//json.cs1=bytes[8];
//json.cs2=bytes[9];
//return json;
//}

#include "LoRaWanMinimal_APP.h"
#include "loramac/system/timeServer.h"
#include "Arduino.h"

//common setting, pins, constatnts and variables
#define RX433_Pin GPIO1
#define BUFFER_LENGTH 72     //received packet should have 72 bits
#define RX433_GAP_TIME 600   //zero gap is ~490us one gap is ~950us

// variables will change:
int rx433State = 0;          // variable for reading the rx433 output status
uint32_t t1, t2, delta;      // variables to measure time
uint8_t i=0;
uint8_t num=0;

//SS II MM DD TD BT TT RR CC
uint8_t  RX_SYNC = 0;
uint8_t  RX_ID = 0;
uint8_t  RX_MTYPE = 0;
uint16_t RX_DEPTH = 0;
uint8_t  RX_TINT = 0;
uint8_t  RX_BATT = 0;
uint16_t RX_TEMP = 0;
uint8_t  RX_RAIN = 0;
uint8_t  RX_CHKSUM = 0;

//array to store times in low state
uint32_t packet_low[BUFFER_LENGTH];
//array to store times in high state
uint32_t packet_high[BUFFER_LENGTH];
//array to store bits that corresponds to low gap size
uint8_t packet_data[BUFFER_LENGTH];
//array to have all data ready to be sent by lora
uint8_t lora_data[9];

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

//Set these OTAA parameters to match your app/node in TTN
static uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

static uint8_t counter=0;

int8_t defaultDrForNoAdr = 5;
int8_t currentDrForNoAdr = 5;

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

static void wakeUp()
{
  sleepTimerExpired=true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

void setup() {
  // put your setup code here, to run once:
  boardInitMcu();
  Serial.begin(115200);
  pinMode(RX433_Pin, INPUT); 
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW); //SET POWER

    if (ACTIVE_REGION==LORAMAC_REGION_AU915) {
    //TTN uses sub-band 2 in AU915
    LoRaWAN.setSubBand2();
  }
 
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  
  //Enable ADR
  LoRaWAN.setAdaptiveDR(true);
  LoraJoin();
}

void LoraJoin() {
   //joining lora if not joined
    if (!LoRaWAN.isJoined()) {
      //In this example we just loop until we're joined, but you could
      //also go and start doing other things and try again later
      Serial.print("Joining... ");
      LoRaWAN.joinOTAA(appEui, appKey, devEui);
        if (!LoRaWAN.isJoined()) {
          Serial.println("failed...");
        } else {
          Serial.println("OK");
        }
    }
}

void Loraloop()
{
    LoraJoin()
    if (LoRaWAN.isJoined()) {
      Serial.println("JOINED");
  
      //Counter is just some dummy data we send for the example
      counter++; 
      
      //getting received data and store them to array to be sent out
      for (int i = 0; i <= 8; i++) {
        lora_data[i] = rx_get_packet_nibble(2*i) + 16 * rx_get_packet_nibble(2*i+1);
      }
    
      //getting xor checksul of 8 received bytes
      lora_data[9] = 0;
      for (int i = 0; i <= 7; i++) {
        lora_data[9] ^= lora_data[i];
      }
    
      //Now send the data. The parameters are "data size, data pointer, port, request ack"
      Serial.printf("\nSending packet with counter=%d\n", counter);
      //Here we send confirmed packed (ACK requested) only for the first five (remember there is a fair use policy)
      bool requestack=counter<5?true:false;
      if (LoRaWAN.send(10, &lora_data[0], 1, requestack)) {
        Serial.println("Send OK");
      } else {
        Serial.println("Send FAILED");
      }
    }
}

///////////////////////////////////////////////////
//Example of handling downlink data
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Received downlink: %s, RXSIZE %d, PORT %d, DATA: ",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
}

static void rx_print_times()
{
  for (int i = 0; i <= BUFFER_LENGTH-1; i++) {
  if (!(i & 7)) {
    Serial.println();
    Serial.printf("%02d : ",i);
  }
  Serial.printf("%03d %03d ",packet_low[i],packet_high[i]);
  }
  Serial.println();
}

static void rx_print_packet()
{
  for (int i = 0; i <= BUFFER_LENGTH; i++) {
  if (!(i & 63)) {
    Serial.println();
    //Serial.printf("%02d : ",i);
  }
  Serial.printf("%01d",packet_data[i]);
  }
  Serial.println();

//output example
//00:57:53.238 -> number : 72
//00:57:53.238 -> 0101111110011100100010000010000010010001100000000110010000000000
//00:57:53.238 -> 00011110

}

static void rx_print_nibbles()
{
  for (int i = 0; i <= 15; i++) {
  if (!(i & 63)) {
    Serial.println();
    //Serial.printf("%02d : ",i);
  }
  Serial.printf("%02d ",rx_get_packet_nibble(i));
  }
  Serial.println();

//SS II MM DD TD TB TT RR CC
  RX_SYNC   = rx_get_packet_nibble(0) + 16 * rx_get_packet_nibble(1);
  RX_ID     = rx_get_packet_nibble(2) + 16 * rx_get_packet_nibble(3);
  RX_MTYPE  = rx_get_packet_nibble(4) + 16 * rx_get_packet_nibble(5);
  RX_DEPTH  = 256 * rx_get_packet_nibble(7) + 16 * rx_get_packet_nibble(6) + rx_get_packet_nibble(8);
  RX_TINT   = rx_get_packet_nibble(9);
  RX_BATT   = rx_get_packet_nibble(11);
  RX_TEMP   = ((256 * rx_get_packet_nibble(13) + 16 * rx_get_packet_nibble(12) + rx_get_packet_nibble(10)) - 400);
  RX_RAIN   = rx_get_packet_nibble(14) + 16 * rx_get_packet_nibble(15);
  RX_CHKSUM = rx_get_packet_nibble(16) + 16 * rx_get_packet_nibble(17);

  Serial.printf("%03d ",RX_SYNC);
  Serial.printf("%03d ",RX_ID);
  Serial.printf("%03d ",RX_MTYPE);
  Serial.printf("%03dcm ",RX_DEPTH);
  Serial.printf("%03d ",RX_TINT);
  Serial.printf("%03d ",RX_BATT);
  Serial.printf("%03ddegC ",RX_TEMP);
  Serial.printf("%03d ",RX_RAIN);
  Serial.printf("%03d ",RX_CHKSUM);
  Serial.println();

//output example
//00:57:53.238 -> 10 15 09 03 01 01 04 00 09 08 01 00 06 02 00 00 
//00:57:53.238 -> 250 057 017 072cm 009 000 209degC 000 120 
  
}

static int rx_get_packet_nibble(int rx_packet_nibble)
{
  int result;
  //bits in packet are sent with LSB first
  result = packet_data[rx_packet_nibble*4] + 2*packet_data[rx_packet_nibble*4+1] + 4*packet_data[rx_packet_nibble*4+2] + 8*packet_data[rx_packet_nibble*4+3];
  //result = packet_data[rx_packet_nibble];
  return result;
}


void loop() {

  start:

  for (int i = 0; i <= BUFFER_LENGTH; i++) {
  // waiting in LOW state 
  t1 = micros();
  while (digitalRead(RX433_Pin) == 0 ) {
  // statement(s)
  t2 = micros();
  
  //test if we are still waiting for a packet
  if (((t2 - t1) > 2500)&&(i == 0)) {
    goto start;
  }
  
  //test if we have the end of a packet
  if ((t2 - t1) > 2500) {
    num = i;
    goto print;
  }

  }

  packet_low[i]=(t2 - t1);
  //for long gap >RX433_GAP_TIMEus we set bit to log.1 except the very first bit that is considered allways as log.0
  packet_data[i]=0;
  if ((packet_low[i] > RX433_GAP_TIME)&&(i > 0)) packet_data[i]=1;
  
  //waiting in HIGH state
  t1 = micros();
  while (digitalRead(RX433_Pin) == 1 ) {
  // statement(s)
  t2 = micros();
  }

  packet_high[i]=(t2 - t1);

  }

  print:

  //print received packet times
  //rx_print_times();
  
  if (num == 72) { 
    Serial.println();
    Serial.printf("received bits : %02d",num);
    rx_print_packet();
    rx_print_nibbles();
    Loraloop();
    //lowPowerSleep(25000);  
  }

  //clearing variables
  for (int i = 0; i <= BUFFER_LENGTH; i++) {
  packet_low[i]=0;
  packet_high[i]=0;  
  packet_data[i]=0;
  }
  num = 0;
}
