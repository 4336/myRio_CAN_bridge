// myRIO CAN Bridge //
// 2022 PHAROS //

////////////
// DEFINE //
////////////
#define USB_DEBUG_MODE
//#define USB_DEBUG_MODE_RAW


/////////////
// LIBRARY //
/////////////
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <InternalTemperature.h>


///////////////
// STRUCTURE //
///////////////
enum
{
  GWAY1_ID = 0x100,
  GWAY2_ID = 0x101,
  GWAY3_ID = 0x102,
  GWAY4_ID = 0x103,
  APS_TX_ID = 0x300,
  APS_RX_ID = 0x301,
};


typedef union
{
  char u_char[8];
  uint8_t u_uint8t[8];
  uint16_t u_uint16t[4];
  float u_float[2];
  double u_double;
} CAN_DATA_u;

typedef struct
{
  bool isChange;
  float wheel_velocity_FL;
  float wheel_velocity_RR;
  float wheel_velocity_RL;
  float wheel_velocity_FR;
}GWAY1_t;

typedef struct
{
  bool isChange;
  float Steering_Tq;
  float Steering_Speed;
  float Steering_Angle;
  uint8_t AirConditioner_On;
  uint8_t Parking_Brake_Active;
  float Lateral_Accel_Speed;
}GWAY2_t;

typedef struct
{
  bool isChange;
  float Throttle_Position;
  uint8_t GearSelDisp;
  uint8_t Gear_Target_Change;
  float Engine_Speed;
  float BrakeMasterCylinder_Pressure;
  float Brake_Active;
  float Accel_Pedal_Position;
}GWAY3_t;

typedef struct
{
  bool isChange;
  float Yaw_Rate_Sensor;
  float Vehicle_Speed_Engine;
  float Longitudinal_Accel_Speed;
  float Cluster_Odometer;
}GWAY4_t;

typedef struct
{
  GWAY1_t GWAY1;
  GWAY2_t GWAY2;
  GWAY3_t GWAY3;
  GWAY4_t GWAY4;
}GWAY_t;

typedef struct
{
  bool isChange;
  float APS1;
  float APS2;
  float final_output;
}APS_t;

/////////////////////
// GLOBAL VARIABLE //
/////////////////////
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN;
Stream& USB = Serial;
Stream& myRIO = Serial2;

GWAY_t GWAY_;
CAN_DATA_u can_buf_;

APS_t APS_;
float APS_desired_ = 0;
uint8_t APS_override_ = 0;

int16_t int16_temp;
uint16_t uint16_temp;

bool isCanRx_ = false;

//////////
// MAIN //
//////////

void setup() {
#ifdef USB_DEBUG_MODE
  Serial.begin(115200); //USB Serial
#endif

  Serial2.begin(115200); //UART2
  Serial3.begin(115200); //UART3  

  // Init CAN
  CAN.begin();
  CAN.setBaudRate(500 * 1000);
  CAN.setMaxMB(16);
  CAN.enableFIFO();
  CAN.enableFIFOInterrupt();
  CAN.onReceive(canSniff);
//  CAN.mailboxStatus();
}


//////////////
// FUNCTION //
//////////////

void serialEvent2(){
  static bool isInit = false;
  static int err = 0;
  
  static uint8_t c;
  static uint8_t buf_cnt = 0;
  static uint8_t rx_buf[8];
  c = myRIO.read();

  if(isInit){
    if(c == '\n'){
      if(buf_cnt == 7){
        // End of Packet
        // (ascii)$ (byte x4)float_APS (byte)isOverride (ascii)\n
        memcpy(&APS_desired_, rx_buf+1,4);
        memcpy(&APS_override_, rx_buf+5,1);
      }else{
        buf_cnt = 0;
        isInit = false;
        err++;
      }
    }else{
      if(buf_cnt > 6) buf_cnt = 6;
      rx_buf[buf_cnt++] = c;
    }
  }else if(c == '$'){
    isInit = true;
    buf_cnt = 0;
    rx_buf[buf_cnt++] = c;
  }
}

#define APS1_IN_MIN 235
#define APS1_IN_MAX 1285
#define APS2_IN_MIN 117
#define APS2_IN_MAX 652
float APS1InMap(float x)
{
  if (x < APS1_IN_MIN) x = APS1_IN_MIN;
  if (x > APS1_IN_MAX) x = APS1_IN_MAX;
  return (x - APS1_IN_MIN) / (APS1_IN_MAX - APS1_IN_MIN) * 100.; //[%]
}
float APS2InMap(float x)
{
  if (x < APS2_IN_MIN) x = APS2_IN_MIN;
  if (x > APS2_IN_MAX) x = APS2_IN_MAX;
  return (x - APS2_IN_MIN) / (APS2_IN_MAX - APS2_IN_MIN) * 100.; //[%]
}

void canSniff(const CAN_message_t &msg) {
#ifdef USB_DEBUG_MODE
#ifdef USB_DEBUG_MODE_RAW
  USB.print(" ID: "); USB.print(msg.id, HEX);
  USB.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    USB.print(msg.buf[i], HEX); USB.print(" ");
  } USB.println();
#endif
#endif

  isCanRx_ = true;

  for (int i = 0 ; i < 8; i++) {
    can_buf_.u_uint8t[i] = msg.buf[i]; //CAN & Teensy use little endian?
  }
  
  switch(msg.id){
    case GWAY1_ID:
      GWAY_.GWAY1.isChange = true;
      GWAY_.GWAY1.wheel_velocity_FL = (can_buf_.u_uint16t[3] & 0x3FFF) * 0.03125;
      GWAY_.GWAY1.wheel_velocity_RR = (can_buf_.u_uint16t[2] & 0x3FFF) * 0.03125;
      GWAY_.GWAY1.wheel_velocity_RL = (can_buf_.u_uint16t[1] & 0x3FFF) * 0.03125;
      GWAY_.GWAY1.wheel_velocity_FR = (can_buf_.u_uint16t[0] & 0x3FFF) * 0.03125;
      break; 
    case GWAY2_ID:
      GWAY_.GWAY2.isChange = true;
      GWAY_.GWAY2.Steering_Tq = (can_buf_.u_uint16t[3] - 0x800) * 0.01;
      GWAY_.GWAY2.Steering_Speed = can_buf_.u_uint8t[5] * 4.0;
      GWAY_.GWAY2.Steering_Angle = (int16_t)(((can_buf_.u_uint8t[4] & 0x00FF)<<8) | can_buf_.u_uint8t[3]) * 0.1;
      GWAY_.GWAY2.AirConditioner_On = can_buf_.u_uint8t[2] >> 4;
      GWAY_.GWAY2.Parking_Brake_Active = can_buf_.u_uint8t[2] & 0b00001111;
      GWAY_.GWAY2.Lateral_Accel_Speed = can_buf_.u_uint16t[0] * 0.01 - 10.23;
      break;
    case GWAY3_ID:
      GWAY_.GWAY3.isChange = true;
      GWAY_.GWAY3.Throttle_Position = ((((can_buf_.u_uint8t[7]&0x0F)<<4) | (can_buf_.u_uint8t[6]&0xF0)>>4) - 0x20) / 2.13;
      GWAY_.GWAY3.GearSelDisp = can_buf_.u_uint8t[6] & 0x0F;
      GWAY_.GWAY3.Gear_Target_Change = can_buf_.u_uint8t[5] >> 4;
      GWAY_.GWAY3.Engine_Speed = (((can_buf_.u_uint8t[5] & 0x000F) << 12) | ((can_buf_.u_uint8t[4] & 0x00FF) << 4) | ((can_buf_.u_uint8t[3] & 0xF0) >> 4)) * 0.25;
      GWAY_.GWAY3.BrakeMasterCylinder_Pressure = (((can_buf_.u_uint8t[3] & 0x000F) << 12) | ((can_buf_.u_uint8t[2] & 0x00FF) << 4) | ((can_buf_.u_uint8t[1] & 0xF0) >> 4)) * 0.1;
      GWAY_.GWAY3.Brake_Active = can_buf_.u_uint8t[1] & 0x0F;
      GWAY_.GWAY3.Accel_Pedal_Position = can_buf_.u_uint8t[0] * 0.3906;
      break;
    case GWAY4_ID:
      GWAY_.GWAY4.isChange = true;
      GWAY_.GWAY4.Yaw_Rate_Sensor = ((can_buf_.u_uint16t[3]&0x1FFF) * 0.01) - 40.95;
      GWAY_.GWAY4.Vehicle_Speed_Engine = can_buf_.u_uint8t[5];
      GWAY_.GWAY4.Longitudinal_Accel_Speed = (((can_buf_.u_uint8t[4] & 0x00FF)<<8) | can_buf_.u_uint8t[3]) * 0.01 - 10.23;
      GWAY_.GWAY4.Cluster_Odometer = can_buf_.u_uint16t[0] * 0.01;
      break;
    case APS_RX_ID:
      APS_.isChange = true;
      APS_.APS1 = APS1InMap(can_buf_.u_uint16t[0]);
      APS_.APS2 = APS2InMap(can_buf_.u_uint16t[1]);
      APS_.final_output = can_buf_.u_float[1];
      break;
    default: //APS_TX_ID & etc
      isCanRx_ = false;
      break;
  }
}

#ifdef USB_DEBUG_MODE
void USB_Debug() {  
  static uint16_t debugCnt = 0;
  USB.print("\nseq: ");
  USB.println(debugCnt++);
  
  USB.print("temp: ");
  USB.println(InternalTemperature.readTemperatureC());

  USB.println("APS\ndesired: " + String(APS_desired_));
  USB.println("override: " + String(APS_override_));
  
  USB.println("G1\nFL: " + String(GWAY_.GWAY1.wheel_velocity_FL,3));
  USB.println("RR: " + String(GWAY_.GWAY1.wheel_velocity_RR,3));
  USB.println("RL: " + String(GWAY_.GWAY1.wheel_velocity_RL,3));
  USB.println("FR: " + String(GWAY_.GWAY1.wheel_velocity_FR,3));
  
  USB.println("G2\nTq: " + String(GWAY_.GWAY2.Steering_Tq,3));
  USB.println("Speed: " + String(GWAY_.GWAY2.Steering_Speed,3));
  USB.println("Angle: " + String(GWAY_.GWAY2.Steering_Angle,3));
  USB.println("Air: " + String(GWAY_.GWAY2.AirConditioner_On,3));
  USB.println("Park: " + String(GWAY_.GWAY2.Parking_Brake_Active,3));
  USB.println("LatACC: " + String(GWAY_.GWAY2.Lateral_Accel_Speed,3));
  
  USB.println("G3\nThrottle: " + String(GWAY_.GWAY3.Throttle_Position,3));
  USB.println("Gear: " + String(GWAY_.GWAY3.GearSelDisp,3));
  USB.println("Target: " + String(GWAY_.GWAY3.Gear_Target_Change,3));
  USB.println("RPM: " + String(GWAY_.GWAY3.Engine_Speed,3));
  USB.println("Pressure: " + String(GWAY_.GWAY3.BrakeMasterCylinder_Pressure,3));
  USB.println("Brake: " + String(GWAY_.GWAY3.Brake_Active,3));
  USB.println("APS: " + String(GWAY_.GWAY3.Accel_Pedal_Position,3));
  
  USB.println("G4\nYaw: " + String(GWAY_.GWAY4.Yaw_Rate_Sensor,3));
  USB.println("Engine: " + String(GWAY_.GWAY4.Vehicle_Speed_Engine,3));
  USB.println("LonACC: " + String(GWAY_.GWAY4.Longitudinal_Accel_Speed,3));
  USB.println("Odometer: " + String(GWAY_.GWAY4.Cluster_Odometer,3));
}
#endif


void loop() {
  CAN.events();
  
  if (GWAY_.GWAY1.isChange) {
    GWAY_.GWAY1.isChange = false;
    myRIO.print("$G1,");
    myRIO.print(GWAY_.GWAY1.wheel_velocity_FL); myRIO.print(",");
    myRIO.print(GWAY_.GWAY1.wheel_velocity_RR); myRIO.print(",");
    myRIO.print(GWAY_.GWAY1.wheel_velocity_RL); myRIO.print(",");
    myRIO.print(GWAY_.GWAY1.wheel_velocity_FR); myRIO.println();
  }
  if (GWAY_.GWAY2.isChange) {
    GWAY_.GWAY2.isChange = false;
    myRIO.print("$G2,");
    myRIO.print(GWAY_.GWAY2.Steering_Tq); myRIO.print(",");
    myRIO.print(GWAY_.GWAY2.Steering_Speed); myRIO.print(",");
    myRIO.print(GWAY_.GWAY2.Steering_Angle); myRIO.print(",");
    myRIO.print(GWAY_.GWAY2.AirConditioner_On); myRIO.print(",");
    myRIO.print(GWAY_.GWAY2.Parking_Brake_Active); myRIO.print(",");
    myRIO.print(GWAY_.GWAY2.Lateral_Accel_Speed); myRIO.println();
  }
  if (GWAY_.GWAY3.isChange) {
    GWAY_.GWAY3.isChange = false;
    myRIO.print("$G3,");
    myRIO.print(GWAY_.GWAY3.Throttle_Position); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.GearSelDisp); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.Gear_Target_Change); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.Engine_Speed); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.BrakeMasterCylinder_Pressure); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.Brake_Active); myRIO.print(",");
    myRIO.print(GWAY_.GWAY3.Accel_Pedal_Position); myRIO.println();
  }
  if (GWAY_.GWAY4.isChange) {
    GWAY_.GWAY4.isChange = false;
    myRIO.print("$G4,");
    myRIO.print(GWAY_.GWAY4.Yaw_Rate_Sensor); myRIO.print(",");
    myRIO.print(GWAY_.GWAY4.Vehicle_Speed_Engine); myRIO.print(",");
    myRIO.print(GWAY_.GWAY4.Longitudinal_Accel_Speed); myRIO.print(",");
    myRIO.print(GWAY_.GWAY4.Cluster_Odometer); myRIO.println();
  }
  if (APS_.isChange) {
    APS_.isChange = false;
    myRIO.print("$AP,");
    myRIO.print(APS_.APS1); myRIO.print(",");
    myRIO.print(APS_.APS2); myRIO.print(",");
    myRIO.print(APS_.final_output); myRIO.println();
  }

#ifdef USB_DEBUG_MODE
  static uint32_t time_1Hz = 0;
  if(time_1Hz + 100 < millis()){
    time_1Hz = millis();
    USB_Debug();
  }
#endif
}
