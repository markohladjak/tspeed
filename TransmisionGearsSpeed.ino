#include <mcp_can.h>
#include <SPI.h>
#include <inttypes.h>

MCP_CAN CAN0(D8);     // Set CS to pin 10
#define CAN0_INT D2    // Set INT to pin 2
// #define CAN0_INT 0    // Set INT to pin 2

long unsigned int rxID;
unsigned char len = 0;
unsigned char rxBuf[8];

void setup()
{
    Serial.begin(115200);

    Serial.println(F("Initializing....."));

    // CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
		  Serial.println(F("MCP2515 Initialized Successfully!"));
	  else
		  Serial.println(F("MCP2515 Initialized Failed!"));

    CAN0.setMode(MCP_NORMAL);
}

#define POS_SIZE 5
#define GEARS_SIZE 6

enum S_POSITION_T {
    NA = -1,
    P = 0,
    D = 5,
    N = 6,
    R = 7,
    M = 8
};

unsigned char speed_pkt_440[8] = { 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
unsigned char gear_pkt_0x43F[8] = { 0x10, 0x60, 0x61, 0xff, 00, 00, 00, 00 };


void process_speed();
void process_selector_position();
void process_gears();
void process_hvac();

int can_read();
void can_send(int id, int len, unsigned char* buf);


void loop()
{
  switch (can_read())
  {
  case 0x556: process_speed();
      break;
  case 0x111: process_selector_position();
      break;
  case 0x113: process_gear();
      break;
  case 0x350: process_hvac();
      break;
  default:
      break;
  } 
  // can_send(0x440, 8, speed_pkt_440);
}

void process_speed()
{
    speed_pkt_440[2] = rxBuf[4];
    
    can_send(0x440, 8, speed_pkt_440);
}

void process_selector_position()
{
    if (rxBuf[0] != 0x00 || ((rxBuf[1] & 0xF0) != 0x40 && rxBuf[2] != 0x60)) return;

    int p = (rxBuf[1] & 0x0F);

    if (p != S_POSITION_T::D && p != S_POSITION_T::M && S_POSITION_T::N && S_POSITION_T::R) return;

    gear_pkt_0x43F[1] = (gear_pkt_0x43F[1] & 0xF0) + p;

    if (p != S_POSITION_T::M) {
        gear_pkt_0x43F[0] = p == S_POSITION_T::P ? 0x10 : 0x14; // ?

        can_send(0x43f, 8, gear_pkt_0x43F);
    }
}

void process_gear()
{
    gear_pkt_0x43F[0]= (gear_pkt_0x43F[0] & 0xF0) + ((rxBuf[2] & 0xF0) >> 4);

    if ((gear_pkt_0x43F[1] & 0x0F) == S_POSITION_T::M)
        can_send(0x43f, 8, gear_pkt_0x43F);
}

void process_hvac()
{
    can_send(0x383, len, rxBuf);
}

int can_read()
{
  rxID = 0;
  
  if(!digitalRead(CAN0_INT))
      CAN0.readMsgBuf(&rxID, &len, rxBuf);

  return rxID;
}

void can_send(int id, int len, unsigned char* buf)
{
    CAN0.sendMsgBuf(id, 0, len, buf);
}


// New HVAC connection:
// left side - new HVAC side
// Orange - Orange
// Black - Black
// Yellow - yellow
