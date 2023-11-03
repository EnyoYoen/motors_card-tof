#include "pins.h"

#include "hardware/i2c.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include "hardware/i2c.h"

extern "C" {
    #include "VL53L1X_api.h"
    #include "VL53L1X_types.h"
}

#define I2C_DEV_ADDR 0x29

enum TofInit : uint8_t {
  Ok,
  Failed,
};

//const uint32_t tof_boot_duration_treshold = 20; // in milliseconds
const uint32_t tof_data_ready_treshold = 100;
const uint8_t tof_count = 1;


rcl_publisher_t distances_publisher;

std_msgs__msg__Int32__Sequence *msg; // Replace Int32 by a UInt16?

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


uint8_t dataReady;
uint8_t sensorState;
VL53L1X_Status_t status;
VL53L1X_Result_t results;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

uint8_t init_tof_parameters(uint16_t tof_dev) {
  // Initialize and configure sensor
  VL53L1X_SensorInit(tof_dev);
  VL53L1X_SetDistanceMode(tof_dev, 1);
  VL53L1X_SetTimingBudgetInMs(tof_dev, 25);
  VL53L1X_SetInterMeasurementInMs(tof_dev, 25);
  VL53L1X_SetROI(tof_dev, 5, 5);
  VL53L1X_StartRanging(tof_dev);

  do {
    VL53L1X_CheckForDataReady(tof_dev, &dataReady);
  } while(dataReady == 0);
  VL53L1X_GetResult(tof_dev, &results);
  
  // 2 clear interrupt when start ranging
  VL53L1X_ClearInterrupt(tof_dev);
  VL53L1X_ClearInterrupt(tof_dev);

  return 0;
}

int32_t get_tof_distance(int tof_dev){
  uint8_t k = 0, c = 0;

  std_msgs__msg__Int32 single_msg;
  single_msg.data = 1;
  RCSOFTCHECK(rcl_publish(&distances_publisher, &single_msg, NULL));

  do {
    VL53L1X_CheckForDataReady(tof_dev, &dataReady);

    /*std_msgs__msg__Int32 single_msg;
    single_msg.data = c;
    RCSOFTCHECK(rcl_publish(&distances_publisher, &single_msg, NULL));*/

    sleep_us(10);
    k++;
    c++;
    if (c > tof_data_ready_treshold) {
      break;
    }
    if (k == 20){
      VL53L1X_StopRanging(tof_dev);
      delay(20);
      init_tof_parameters(tof_dev);
      k=0;
    }
  } while (dataReady == 0);

  VL53L1X_GetResult(tof_dev, &results);
  VL53L1X_ClearInterrupt(tof_dev);
  
  return results.distance;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  std_msgs__msg__Int32 single_msg;
  //single_msg.data = 12;
  //for (uint8_t i = 0 ; i < tof_count ; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    single_msg.data = get_tof_distance(I2C_DEV_ADDR);
    digitalWrite(LED_BUILTIN, LOW);
    //single_msg.data = 0;
  //  msg->data[i] = single_msg;
  //}*/
  RCSOFTCHECK(rcl_publish(&distances_publisher, &single_msg, NULL));
}

uint8_t init_single_tof(uint16_t tof_dev) {
  // Ensure the sensor has booted
  do {
    status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
  } while (sensorState == 0);

  init_tof_parameters(tof_dev);

  return (status == 0 ? TofInit::Ok : TofInit::Failed);
}

uint32_t init_tof() {

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Create node
  RCCHECK(rclc_node_init_default(&node, "motors_tof_ucontroller", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &distances_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motors_tof/tof"));

  const unsigned int timer_timeout = 300;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  msg = std_msgs__msg__Int32__Sequence__create(tof_count);
  if (msg == nullptr) {
    return uint32_t(0x80000000); // first bit set
  }

  /*// I2C init
  _i2c_init(i2c0, 5000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0) {
      return uint32_t(0x40000000); // second bit set
  }

  uint32_t tof_init_results = 0;
  for (uint8_t i = 0 ; i < tof_count ; i++) {
    tof_init_results |= ((uint32_t)init_single_tof(I2C_DEV_ADDR) << i);
  }

  return tof_init_results;*/

  /*#define I2C_DEV_ADDR_RIGHT 0x29

  #define I2C_DEV_ADDR_SHIFTED 0x31
  #define I2C_DEV_ADDR I2C_DEV_ADDR_SHIFTED>>1

  // ENABLE SHUTDOWN GPIO
  _gpio_init(PIN_FREE_GPIO);
  gpio_set_dir(PIN_FREE_GPIO, GPIO_OUT);
  gpio_put 	(PIN_FREE_GPIO,LOW);    // ==> shuntdown the first tof in order to change address of the second one.


  _i2c_init(i2c0, 5000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);



  if (VL53L1X_I2C_Init(I2C_DEV_ADDR_RIGHT, i2c0) < 0) {
      return TofInit::Failed;
  }

  

  // Ensure the sensor has booted
  uint8_t sensorState;
  do {
      status += VL53L1X_BootState(I2C_DEV_ADDR_RIGHT, &sensorState);
      VL53L1X_WaitMs(I2C_DEV_ADDR_RIGHT, 2);
  } while (sensorState == 0);


  // change address of second tof and stop shutdown the first one.
  VL53L1X_SetI2CAddress(I2C_DEV_ADDR_RIGHT, I2C_DEV_ADDR_SHIFTED);
  gpio_put 	(PIN_FREE_GPIO,HIGH);
  delay(10);

  // init the first tof
  if (VL53L1X_I2C_Init(I2C_DEV_ADDR_RIGHT, i2c0) < 0) {
      return TofInit::Failed;
  }

  // Ensure the sensor has booted
  do {
      status += VL53L1X_BootState(I2C_DEV_ADDR_RIGHT, &sensorState);
      VL53L1X_WaitMs(I2C_DEV_ADDR_RIGHT, 2);
  } while (sensorState == 0);


  // Initialize and configure sensor
  init_tof_parameters(I2C_DEV_ADDR);
  init_tof_parameters(I2C_DEV_ADDR_RIGHT);*/


  if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0) {
    return TofInit::Failed;
  }

  // Ensure the sensor has booted
  uint8_t sensorState;
  do {
    status = VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    sleep_ms(2);
  } while (sensorState == 0);

  // Initialize and configure sensor
  init_tof_parameters(I2C_DEV_ADDR);


  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_spin(&executor));
  

  return TofInit::Ok;
}