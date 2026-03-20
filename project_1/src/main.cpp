#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>


#define RELAY_PIN 14
#define SENSOR_PIN 18
#define SENSOR_PIN2 19
#define LED_PIN 

rcl_publisher_t publisher;
std_msgs__msg__Int8 collision_msg; 
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup() {
  Serial.begin(115200); 
  set_microros_serial_transports(Serial);
  pinMode(SENSOR_PIN, OUTPUT);
  pinMode(SENSOR_PIN2, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "collision_detector", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "bumper_status"
  );
}

void loop() {
    int8_t sensor_value = 0;
    digitalRead(SENSOR_PIN);
    if (digitalRead(SENSOR_PIN) == HIGH) {
        sensor_value += 1;
    }
    if (digitalRead(SENSOR_PIN2) == HIGH) {
        sensor_value += 2;
    }
    collision_msg.data = sensor_value;

    if (sensor_value != 0) {
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("COLLISION DETECTED");
    } else {
        digitalWrite(RELAY_PIN, HIGH);
    }
    rcl_publish(&publisher, &collision_msg, NULL);
    if (collision_msg.data > 0) {
        delay(5000); 
    } else {
        delay(50);   
    }
}