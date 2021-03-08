//author Petr Simandl www.simandl.cz
//based on TS-FT002 timing described on page
//https://github.com/merbanan/rtl_433/blob/ea797cff8cafd7fc385f8cf413f072df725f9e5e/src/devices/ts_ft002.c
//code developed on Heltec Cube Cell HTCC-AB01
//as 433Mhz receiver is used SRX882 Superheterodyne Receiver
//schematic 
//SRX882          HTCC-AB01
//pin 3 vcc       vs
//pin 5 data      gpio1
//pin 6 gnd       gnd
//usb serves as COM debug monitor and power supply
//TO DO
//- received packet checksum control
//- integration with alix APU centos hotsanic to get graphs
//- adding LoRaWan packet sending code to intergate it with thethingsnetwork.org and similar

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


void setup() {
  // put your setup code here, to run once:
  boardInitMcu();
  Serial.begin(115200);
  pinMode(RX433_Pin, INPUT); 
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW); //SET POWER
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
  RX_DEPTH  = 256 * rx_get_packet_nibble(7) + 16 * rx_get_packet_nibble(6) + rx_get_packet_nibble(9);
  RX_TINT   = rx_get_packet_nibble(8);
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
  // put your main code here, to run repeatedly:

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
  }

  //clearing variables
  for (int i = 0; i <= BUFFER_LENGTH; i++) {
  packet_low[i]=0;
  packet_high[i]=0;  
  packet_data[i]=0;
  }
  num = 0;
  
}
