//#undef ESP32

#ifdef ESP32
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;               // CAN Config
CAN_frame_t g_rx_frame;
const int rx_queue_size = 10;       // Receive Queue size

#else
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);     // Set CS to pin 10
#define CAN0_INT 2    // Set INT to pin 2

#endif

#include <inttypes.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


void setup()
{
  Serial.begin(115200);

#ifdef ESP32
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_21;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
#else
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) 
  Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
#endif
}

#define POS_SIZE 5
#define GEARS_SIZE 7

uint64_t positions[POS_SIZE][2] = {
    { 0x0040600000000000, 0x106061ff00000000 }, //P
    { 0x0047600000000000, 0x146761ff00000000 }, //R
    { 0x0046600000000000, 0x146661ff00000000 }, //N
    { 0x0045600000000000, 0x146561ff00000000 }, //D
    { 0x0048600000000000, 0x0000000000000000 }, //M
};

uint64_t gears[GEARS_SIZE][2] = {
    { 0x0020100000000000, 0x116861ff00000000, },
    { 0x0020200000000000, 0x126861ff00000000, },
    { 0x0020300000000000, 0x136861ff00000000, },
    { 0x0020400000000000, 0x146861ff00000000, },
    { 0x0020500000000000, 0x156861ff00000000, },
    { 0x0020600000000000, 0x166861ff00000000, },
    { 0x0020700000000000, 0x176861ff00000000, },
};

void print_buf(int id, unsigned char *buf, int len)
{
    Serial.print("id: 0x");
    Serial.print(id, 16);
    Serial.print("      ");
    for (int x=0; x<len; x++){
      Serial.print(buf[x], 16);
      Serial.print(" ");
    }
    Serial.println();
}

void reverse(unsigned char* buf)
{
  for (int i = 0; i < 4; i++)
  {
    unsigned char t = buf[i];
    buf[i] = buf[7-i];
    buf[7-i] = t;
  }
}

String UINT64ToString(uint64_t val) {
    uint32_t Vh = (unsigned long)((val & 0xFFFFFFFF00000000) >> 32 );
    uint32_t Vl = (unsigned long)((val & 0x00000000FFFFFFFF));

    return String(Vh, HEX) + String(Vl, HEX);
}

bool canSend(int id, int ext, int len, unsigned char* buf)
{
//  return true;
#ifdef ESP32
    CAN_frame_t tx_frame;
    
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id;
    tx_frame.FIR.B.DLC = len;

    for (int i=0; i < len; i++)
      tx_frame.data.u8[i] = buf[i];

    ESP32Can.CANWriteFrame(&tx_frame);

    return true;
#else
    byte sndStat = CAN0.sendMsgBuf(id, ext, len, buf);
  
//    if(sndStat == CAN_OK)
//      Serial.println("Message Sent Successfully!");
//    else 
//      Serial.println("Error Sending Message...");

    return sndStat == CAN_OK;
#endif
}
void gearRead(uint64_t &data){
    reverse(rxBuf);

    uint64_t d = (uint64_t)(*(uint64_t*)&rxBuf[0]);
    data = d & 0xfffff00000000000uLL;
  }

void gearWrite(uint64_t data){
    uint64_t d = data;
    memcpy(rxBuf, (unsigned char *)&d, 8);
    
    reverse(rxBuf);

//    Serial.print("Write: ");
//    print_buf(0x43f, rxBuf, 8);

    canSend(0x43f, 0, 8, rxBuf);
}

int position = -1;
int gear = -1;

char pos_name[5] = {'P', 'R', 'N', 'D', 'M'};

void print_state()
{
  Serial.print("Gear: ");

  if (position < 0 || position >= POS_SIZE)
    return;

  if (position == 4)
    Serial.println(String(gear));
  else
    Serial.println(pos_name[position]);
}

#ifdef ESP32
void print_frame(CAN_frame_t *rx_frame, unsigned long delta = 0)
{
    if (rx_frame->FIR.B.FF == CAN_frame_std) {
      printf("New standard frame");
    }
    else {
      printf("New extended frame");
    }

    if (rx_frame->FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r", rx_frame->MsgID,  rx_frame->FIR.B.DLC);
    }
    else {
      printf(" from 0x%08X, DLC %d, Data ", rx_frame->MsgID,  rx_frame->FIR.B.DLC);
      for (int i = 0; i < rx_frame->FIR.B.DLC; i++) {
        printf("0x%02X ", rx_frame->data.u8[i]);
      }
    }

    printf("    delta: %lu", delta);
    printf("\n");
}
#endif

int canRead()
{
  rxId = 0;
  
#ifdef ESP32
//  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
  if (xQueueReceive(CAN_cfg.rx_queue, &g_rx_frame, 0) == pdTRUE) {
    rxId = g_rx_frame.MsgID;
    len = g_rx_frame.FIR.B.DLC;

    for (int i = 0; i < len; i++)
      rxBuf[i] = (uint8_t)(g_rx_frame.data.u8[i]);
    
//    print_frame(&g_rx_frame);
  }
#else
  if(!digitalRead(CAN0_INT))      // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
#endif

    return rxId;
}

void process_gears(int id);
void process_speed();
void send_speed();

unsigned long time_point = millis();

void loop()
{
  int id = 0;
  
  if(id = canRead())      // If CAN0_INT pin is low, read receive buffer
  {
    process_speed();
    process_gears(id);
  }

  unsigned long t = millis();
  if (t - time_point > 100) {
    send_speed();
    
    time_point = t;  
  }

  process_hvac();
  
  //delay(1);
}

unsigned char speed_buf[8] = { 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,0x00 };

void process_speed()
{
  if (rxId == 0x316 && rxBuf[0] == 0x05) 
    speed_buf[2] = rxBuf[6];
}

void send_speed()
{
//    byte sndStat = CAN0.sendMsgBuf(0x440, 0, 8, speed_buf);
    auto sndStat = canSend(0x440, 0, 8, speed_buf);

    Serial.print((int)speed_buf[2]);
    
    if(sndStat)
      Serial.println("     +");
    else 
      Serial.println("     Failed");
}

void process_gears(int id) {
  uint64_t data;
  gearRead(data);
    
  if(id == -1)
    return;

  if (id == 0x111) {
    for (int i=0; i < POS_SIZE; i++)
      if (positions[i][0] == data)
        if (position != i) {
          position = i;
          print_state();
        }
  }

  if (id == 0x113) {
    int i;
    for (i=0; i < GEARS_SIZE; i++)
      if (gears[i][0] == data)
        if (gear != i) {
          gear = i;
          print_state();
        }
  }
    
  if (position >= 0 && position < 4)
    gearWrite(positions[position][1]);

  if (position == POS_SIZE-1 && gear > -1)
    gearWrite(gears[gear][1]);
}

void process_hvac()
{
    auto t = millis();
    static unsigned long lt1 = millis();
    static unsigned long lt2 = millis();

    static uint8_t f043[8] = {0x01, 0xF3, 0xCF, 0x0E, 0x01, 0x00, 0x00, 0xF0};
    static uint8_t f044[8] = {0x01, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00};
    static uint8_t f383[8] = {0x05, 0x24, 0x44, 0xFF, 0xFF, 0x00, 0x00, 0x65};

    if (t - lt1 >= 20) {
      canSend(0x383, 0, 8, f383);
      lt1 = t;
    }
    if (t - lt2 >= 1000) {
      canSend(0x043, 0, 8, f043);
      canSend(0x044, 0, 8, f044);
      lt2 = t;
    }
}
