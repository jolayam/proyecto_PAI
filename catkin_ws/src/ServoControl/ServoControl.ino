/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
can bus arduino with ros
#include <mcp2515.h>  
#include <SPI.h>  
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

MCP2515 mcp2515(10);     // Set CS to pin 10
struct can_frame canSent;
struct can_frame canReceived;

//Define LED pins
#define LED1 5 

// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set values
float p_in[3] = {0.0f,0.0f,0.0f};
float v_in[3] = {0.0f,0.0f,0.0f};
float kp_in[3] = {0.0f,0.0f,0.0f};
float kd_in[3] = {0.0f,0.0f,0.0f};
float t_in[3] = {0.0f,0.0f,0.0f};
// measured values
float p_out[3] = {0.0f,0.0f,0.0f};
float v_out[3] = {0.0f,0.0f,0.0f};
float t_out[3] = {0.0f,0.0f,0.0f};

// variable to receive the message
unsigned char len = 0;
// variable to send the message
unsigned char buf[8];

//for MIT controllers
float p_step=200;send 

//Define Publishers
ros::Publisher publisher_p_out("p_out", &Float32);
ros::Publisher publisher_v_out("v_out", &Float32);
ros::Publisher publisher_t_out("t_out", &Float32);

//Define Suscribers
void can_id_func( const std_msgs::Float32& cmd_msg1){
  can_id
}


void p_in_func( const std_msgs::Float32& cmd_msg11){
  p_in[can_id]=cmd_msg2.data;
}
void v_in_func( const std_msgs::Float32& cmd_msg12){
  v_in[can_id]=cmd_msg3.data;
}
void t_in_func( const std_msgs::Float32& cmd_msg13){
  t_in[can_id]=cmd_msg3.data;
}


void kp_in_func( const std_msgs::Float32& cmd_msg4){
  for (int i=0;i <4;i++){
    kp_in[i]=cmd_msg3.data;
  }
}
void kd_in_func( const std_msgs::Float32& cmd_msg5){
  for (int j=0;j <4;j++){
    kp_in[j]=cmd_msg3.data;
  }
}


ros::Subscriber<std_msgs::Float32> subscriber_canid("can_id", can_id_func);
ros::Subscriber<std_msgs::Float32> subscriber_canid("p_in", p_in_func);
ros::Subscriber<std_msgs::Float32> subscriber_canid("v_in", v_in_func);
ros::Subscriber<std_msgs::Float32> subscriber_canid("kp_in", kp_in_func);
ros::Subscriber<std_msgs::Float32> subscriber_canid("kd_in", kd_in_func);
ros::Subscriber<std_msgs::Float32> subscriber_canid("t_in", t_in_func);


void setup(){

  nh.initNode();

  Serial.begin(115200);
  delay(1000);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("MCP2515 Initialized Successfully!");

  Serial.println("CAN BUS Shield init ok!");

  for (int i=0;i<3;i++){
    delay(5);
    pack_cmd(i+1);

    //receive msg
    delay(5);
    if(mcp2515.readMessage(&canReceived) == MCP2515::ERROR_OK){
      unpack_reply();
    }

    p_in[i]=p_out[i];

    //CODIGO PARA ENVIARLO POR ROS
    //TODO

    pinMode(LED1, OUTPUT);
    digitalWrite(LED1, HIGH);
  }

  //enable motors
  for (int i=0;i<3;i++){
    delay(500);
    EnableMotorMode(i+1);
  }

  
  nh.subscribe(sub);
  
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}

void EnterMotorMode(int id){
  struct can_frame canMsg1;
  // Enter Motor Mode (enable)
  canMsg1.can_id = float_to_uint(id);
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFC;

  mcp2515.sendMessage(&canMsg1);
}

void ExitMotorMode(int id){
  struct can_frame canMsg2;
  // Enter Motor Mode (enable)
  canMsg2.can_id = float_to_uint(id);
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0xFF;
  canMsg2.data[1] = 0xFF;
  canMsg2.data[2] = 0xFF;
  canMsg2.data[3] = 0xFF;
  canMsg2.data[4] = 0xFF;
  canMsg2.data[5] = 0xFF;
  canMsg2.data[6] = 0xFF;
  canMsg2.data[7] = 0xFD;

  mcp2515.sendMessage(&canMsg2);

}

void pack_cmd(int id){
  //byte buf[8];

  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX); //fminf(fmaxf(P_MIN, p_in), P_MAX);                    
  float v_des = constrain(v_in, V_MIN, V_MAX); //fminf(fmaxf(V_MIN, v_in), V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX); //fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX); //fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX); //fminf(fmaxf(T_MIN, t_in), T_MAX);

  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  canSent.can_id = float_to_uint(id);
  canSent.can_dlc = 8;
  canSent.data[0] = p_int >> 8;  
  canSent.data[1] = p_int & 0xFF;
  canSent.data[2] = v_int >> 4;
  canSent.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  canSent.data[4] = kp_int & 0xFF;
  canSent.data[5] = kd_int >> 4;
  canSent.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  canSent.data[7] = t_int & 0xFF;

  mcp2515.sendMessage(&canSent);
}

void unpack_reply(int id){

  /// CAN Reply Packet Structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and + 30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  unsigned float canId = uint_to_float(canReceived.can_id);

  /// unpack ints from can buffer ///
  unsigned int id = canReceived.data[0];
  unsigned int p_int = (canReceived.data[1] << 8) | canReceived.data[2];
  unsigned int v_int = (canReceived.data[3] << 4) | (canReceived.data[4] >> 4);
  unsigned int i_int = ((canReceived.data[4] & 0xF) << 8) | canReceived.data[5];
  /// convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
} 

unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}