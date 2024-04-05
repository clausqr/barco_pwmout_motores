#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include "secrets.h"

#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <AHTxx.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <iostream>
//using namespace std; Habilitar esto rompe el alocator pero me habilita la funcion to_string(int)
#include <string> 
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>

#include <rosidl_runtime_c/string_functions.h>
//#include <micro_ros_utilities>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 


#define PWM1_OUT_PIN GPIO_NUM_12
#define PWM2_OUT_PIN GPIO_NUM_13
#define PWM3_OUT_PIN GPIO_NUM_14
#define PWM4_OUT_PIN GPIO_NUM_15

//I2C address of sensor INA219 en Ao to gnd y A1 to gnd
#define I2C_DEV_ADDR_1 0x41
#define I2C_DEV_ADDR_2 0x40

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

//rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pwm;
//std_msgs__msg__Int32 adc_value;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// create executor
rclc_executor_t executor;

// publishers
//Motor R
rcl_publisher_t publisher_v1;
rcl_publisher_t publisher_i1;
std_msgs__msg__Int32 v1_out;
std_msgs__msg__Int32 i1_out;

//Motor L
rcl_publisher_t publisher_v2;
rcl_publisher_t publisher_i2;
std_msgs__msg__Int32 v2_out;
std_msgs__msg__Int32 i2_out;

//temp y hum
rcl_publisher_t publisher_t;
rcl_publisher_t publisher_h;
std_msgs__msg__Int32 t_out;
std_msgs__msg__Int32 h_out;

//std_msgs__msg__String msg_adc;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// subscriber reference:
rcl_subscription_t subscriber;
const char * topic_name = "pwm_out";
const char * topic_name21 = "V1_out";
const char * topic_name31 = "I1_out";
const char * topic_name22 = "V2_out";
const char * topic_name32 = "I2_out";
const char * topic_name4 = "T_out";
const char * topic_name5 = "H_out";

uint8_t i;
float ahtValue;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type

const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64);


#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


static uint8_t i2cread(void) {
  return Wire.read();
}

static void i2cwrite(uint8_t x) {
  Wire.write((uint8_t)x);
}

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value>>8));
  i2cwrite((uint8_t)(value & 0xFF));
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
}

static int16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((i2cread() << 8) | i2cread());  
}



void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Lectura de tension y corriente del Motor R
    uint32_t lectura_V1 = ((readRegister(I2C_DEV_ADDR_1, 0x02) >> 3) & 0x00001FFF );
    int32_t lectura_I1 = (readRegister(I2C_DEV_ADDR_1, 0x04));
    //Serial.printf("V1: %u\n", lectura_V1*4);
    //Serial.printf("I1: %u\n", lectura_I1*5);
    v1_out.data = lectura_V1*3.9985; //Esto se puede poner como parametro en ros
    i1_out.data = lectura_I1*5;
    RCSOFTCHECK(rcl_publish(&publisher_v1, &v1_out, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_i1, &i1_out, NULL));




    //lectura de tension y corrienteMotor L
    
    //Lectura de los registros del sensor
    uint32_t lectura_V2 = ((readRegister(I2C_DEV_ADDR_2, 0x02) >> 3) & 0x00001FFF);
    int32_t lectura_I2 = (readRegister(I2C_DEV_ADDR_2, 0x04));
    
    //Asignar las lecturas a la estructura.data 
    v2_out.data = lectura_V2*3.9975;
    i2_out.data = lectura_I2*5;
    

    //Enviar la data por el nodo de ros
    RCSOFTCHECK(rcl_publish(&publisher_v2, &v2_out, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_i2, &i2_out, NULL));

   
   
    // lectura de la temperatura y humedad cada 10 seg
    if (i >= 10) 
    {

      ahtValue = aht10.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds

      if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
      {
        t_out.data = ahtValue;
        RCSOFTCHECK(rcl_publish(&publisher_t, &t_out, NULL));
      }


      ahtValue = aht10.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds

      if (ahtValue != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
      {
        h_out.data = ahtValue;
        RCSOFTCHECK(rcl_publish(&publisher_h, &h_out, NULL));
      }
    
      i=0;
    }
    else {i += 1;}

    
    
  }
}

// subscriber callback:
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float64 * msg_pwm = (const std_msgs__msg__Float64 *)msgin;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  Serial.print("I heard: ");
  Serial.println(msg_pwm->data);
  float pwm_float = msg_pwm->data;
  if (pwm_float > 1.0)
  {
    pwm_float = 1.0;
  }
  else if (pwm_float < -1.0)
  {{}
    pwm_float = -1.0;
  }
  int pwm_us = (int)(abs(pwm_float*500.0+1500.0));
  Serial.print("int pwm_us = ");
  Serial.print(pwm_us);
  Serial.println(" ");
  mot1.writeMicroseconds(pwm_us);
  mot2.writeMicroseconds(pwm_us);
  mot3.writeMicroseconds(pwm_us);
  mot4.writeMicroseconds(pwm_us);
}

void setup() 
{
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);
  aht10.begin();

  //configuration register init
  writeRegister(I2C_DEV_ADDR_1, 0x00, 0x3FFF);
  writeRegister(I2C_DEV_ADDR_2, 0x00, 0x3FFF);
  //Calibration register init
  writeRegister(I2C_DEV_ADDR_1, 0x05, 0x117C); // R2
  writeRegister(I2C_DEV_ADDR_2, 0x05, 0x0F79); // R1
  
  //valor de calibracion para prueba con la r de stock
  //writeRegister(I2C_DEV_ADDR_2, 0x05, 0x1062);


  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	// standard 50 hz servo
  mot1.setPeriodHertz(50);
  mot2.setPeriodHertz(50);
  mot3.setPeriodHertz(50);
  mot4.setPeriodHertz(50);

  // attach servos to pins
  // using default min/max of 1000us and 2000us
  mot1.attach(PWM1_OUT_PIN, 1000, 2000);
  mot2.attach(PWM2_OUT_PIN, 1000, 2000);
  mot3.attach(PWM3_OUT_PIN, 1000, 2000);
  mot4.attach(PWM4_OUT_PIN, 1000, 2000);

	
  //IPAddress agent_ip(10, 42, 0, 1);
  //IPAddress agent_ip(192, 168, 0, 106);
  //size_t agent_port = 8888;

  //IPAddress local_ip(192,168,5,1);
  //IPAddress gateway(192,168,5,1);
  //IPAddress subnet(255,255,255,0);
  // byte local_mac[] = { 0xD4, 0xD4, 0xDA, 0x5C, 0x47, 0xF1 };
  

  //WiFi.softAP(ssid, psk);        //Start Acces point mode
  //WiFi.softAPConfig(local_ip, gateway, subnet);

  // uint8_t baseMac[6];
  // esp_read_mac(baseMac, ESP_MAC_WIFI_SOFTAP);
  // Serial.print("SoftAP MAC: ");
  // for (int i = 0; i < 5; i++) {
  //   Serial.printf("%02X:", baseMac[i]);
  // }
  // Serial.printf("%02X\n", baseMac[5]);

  // delay(10000);

  //Serial.print("Setting up Micro-ROS WiFi transport...");

  //set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  //set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
  
  Serial.print("Setting up Micro-ROS Serial transport...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  
  Serial.println("OK");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);


  Serial.println("Starting Micro-ROS WiFi node...");
  
  allocator = rcl_get_default_allocator();

  Serial.println("1");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  Serial.println("2");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));
  Serial.println("3");

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    type_support,
    topic_name));
  Serial.println("4");

  // create publishers
  //Motor R
  RCCHECK(rclc_publisher_init_default(
      &publisher_v1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name21));

  RCCHECK(rclc_publisher_init_default(
      &publisher_i1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name31));

  //Motor L
  RCCHECK(rclc_publisher_init_default(
      &publisher_v2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name22));

  RCCHECK(rclc_publisher_init_default(
      &publisher_i2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name32));

  //Temp y Humedad
  RCCHECK(rclc_publisher_init_default(
      &publisher_t,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name4));
  

    RCCHECK(rclc_publisher_init_default(
      &publisher_h,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // la variable msg no remite al mensaje sino a la subcategoria de sts_msg de ros2
      topic_name5));


  Serial.println("5");

  // create timer, called every 1000 ms to publish adc
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
  Serial.println("6");

  // create executor
  int num_handles = 1;
  
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  // Add subscriber to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_pwm, &subscription_callback, ON_NEW_DATA));




}

void loop() 
{
 // timeout specified in nanoseconds (here 1s)
  //Serial.println("Test_Loop_1");
  rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
  rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
  //Serial.println("Test_Loop_2");
}
