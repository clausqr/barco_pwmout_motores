#include <Arduino.h>
#include <ESP32Servo.h>

#include "secrets.h"

#include <micro_ros_platformio.h>
#include <WiFi.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <std_msgs/msg/int32.h>

#include <std_msgs/msg/float64.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 


#define PWM1_OUT_PIN GPIO_NUM_12
#define PWM2_OUT_PIN GPIO_NUM_13
#define PWM3_OUT_PIN GPIO_NUM_14
#define PWM4_OUT_PIN GPIO_NUM_15

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// create executor
rclc_executor_t executor;


// subscriber reference:
rcl_subscription_t subscriber;
const char * topic_name = "pwm_out";


const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64);

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) 
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //int dt = millis() - last_call_time;
    // msg.data++;
    msg.data = 888;
  }
}

// subscriber callback:
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  Serial.print("I heard: ");
  Serial.println(msg->data);
  float pwm_float = msg->data;
  if (pwm_float > 1.0)
  {
    pwm_float = 1.0;
  }
  else if (pwm_float < -1.0)
  {
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

	
  IPAddress agent_ip(192, 168, 0, 1);
  size_t agent_port = 8888;

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  Serial.begin(115200);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    type_support,
    topic_name));

  // create executor
  int num_handles = 1;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  // Add subscriber to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));



}

void loop() 
{
 // timeout specified in nanoseconds (here 1s)
  rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
}