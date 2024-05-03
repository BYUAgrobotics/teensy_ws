#include <Arduino.h>
#include <agro_interfaces/srv/process.h>
#include <agro_interfaces/msg/sensors.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <AccelStepper.h>
#include <PWMServo.h>


#define LED_PIN 13
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 1

#define ROTATION_SPD 200
#define DEPLOY_SPD 100
#define SLIDE_SPD 800
#define TWIST_TIME 200
#define PICK_TIME 200
#define PICKER_HOME 10

#define STEPPER1_STEP 0
#define STEPPER1_DIR  1
#define STEPPER2_STEP 2
#define STEPPER2_DIR  3
#define STEPPER3_STEP 4
#define STEPPER3_DIR  5

#define SERVO1_SIG 6
#define SERVO2_SIG 7

#define SENSOR1_TRIG 24
#define SENSOR1_ECHO 8
#define SENSOR2_TRIG 25
#define SENSOR2_ECHO 9
#define SENSOR3_TRIG 26
#define SENSOR3_ECHO 10
#define SENSOR4_TRIG 27
#define SENSOR4_ECHO 11

#define PLANT_DECT 32

#define MAX_WAIT_US 5000

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER,STEPPER1_STEP,STEPPER1_DIR);; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER,STEPPER2_STEP,STEPPER2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER,STEPPER3_STEP,STEPPER3_DIR);
PWMServo twister;
PWMServo picker;



static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_service_t service;
static agro_interfaces__srv__Process_Response process_resp;
static agro_interfaces__srv__Process_Request process_req;
rcl_publisher_t publisher;
const char *topic_name = "distanceSensors";

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

void process_srv_callback(const void *request_msg, void *response_msg)
{
  agro_interfaces__srv__Process_Request *req_in =
      (agro_interfaces__srv__Process_Request *)request_msg;
  agro_interfaces__srv__Process_Response *res_in =
      (agro_interfaces__srv__Process_Response *)response_msg;

  int deploy = req_in->deploy;
  int rotation = req_in->rotation;
  int slide = req_in->slide;
  int twist = req_in->twist;
  bool pick = req_in->pick;
  int picker_loc = req_in->picker_loc;

  // deploy the arm
  stepper3.moveTo(deploy);
  stepper3.setSpeed(DEPLOY_SPD);
  while (stepper3.distanceToGo()) {
      stepper3.runSpeedToPosition();
  }

  // // Rotation
    stepper1.moveTo(rotation);
    stepper1.setSpeed(ROTATION_SPD);
    while (stepper1.distanceToGo()) {
        stepper1.runSpeedToPosition();
    }
    

  // twist wrist
    twister.write(twist);
    delay(TWIST_TIME);

  if (pick) {
    picker.write(PICKER_HOME);
    delay(PICK_TIME);
  }
  else {
    picker.write(picker_loc);
    delay(PICK_TIME);
  }

  // Slide in
    stepper2.moveTo(slide);
    stepper2.setSpeed(SLIDE_SPD);
    while (stepper2.distanceToGo()) {
        stepper2.runSpeedToPosition();
    }

    if (pick) {
    //  actuate picking
      picker.write(160);
      delay(PICK_TIME);

    // Slide arm back
      stepper2.moveTo(0);
      stepper2.setSpeed(SLIDE_SPD);
      while (stepper2.distanceToGo()) {
          stepper2.runSpeedToPosition();
      }

      // move picker to home posistion
      picker.write(PICKER_HOME);
      // Serial.printf("Done\n");
      delay(PICK_TIME);
    }
      
  res_in->finished = true;
}

void sendSensorInformation()
{
  agro_interfaces__msg__Sensors msg;
  digitalWrite(SENSOR1_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR1_TRIG,LOW);
  msg.right_back = pulseIn(SENSOR1_ECHO, HIGH,MAX_WAIT_US);

  digitalWrite(SENSOR2_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR2_TRIG,LOW);
  msg.right_front = pulseIn(SENSOR2_ECHO, HIGH,MAX_WAIT_US);
  
  digitalWrite(SENSOR3_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR3_TRIG,LOW);
  msg.front_right = pulseIn(SENSOR3_ECHO, HIGH,MAX_WAIT_US);

  digitalWrite(SENSOR4_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR4_TRIG,LOW);
  msg.front_left = pulseIn(SENSOR4_ECHO, HIGH,MAX_WAIT_US);

  msg.plant_detection = !digitalRead(PLANT_DECT);

  // msg.right_back = 100;
  // msg.right_front = 200;
  // msg.front_left = 300;
  // msg.front_right = 400;
  rcl_publish(&publisher, &msg, NULL);
}

// Creates micro-ROS entities
bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create service
  RCCHECK(rclc_service_init_best_effort(
      &service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(agro_interfaces, srv, Process), "process"));

  const rosidl_message_type_support_t *type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(agro_interfaces, msg, Sensors);
  rcl_ret_t rc = rclc_publisher_init_best_effort(
      &publisher, &node,
      type_support, topic_name);

  // create executor
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL,
                                 &allocator));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &service, &process_req,
                                        &process_resp, process_srv_callback));

  return true;
}

// Destroys micro-ROS entities
void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_service_fini(&service, &node);
  rcl_publisher_fini(&publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

    stepper1.setMaxSpeed(800.0);
    stepper1.setAcceleration(10.0);
    stepper1.setMinPulseWidth(8);
    stepper1.setCurrentPosition(100);
//    stepper1.moveTo(50);
//    stepper1.setSpeed(10);

    stepper2.setMaxSpeed(400.0);
    stepper2.setAcceleration(10.0);
    stepper2.setMinPulseWidth(8);
//    stepper2.moveTo(400);
//    stepper2.setSpeed(800);

    stepper3.setMaxSpeed(400.0);
    stepper3.setAcceleration(10.0);
    stepper3.setMinPulseWidth(8);

    twister.attach(SERVO1_SIG);
    picker.attach(SERVO2_SIG);
    twister.write(0);
    picker.write(170);

  pinMode(PLANT_DECT,INPUT);
  pinMode(SENSOR1_ECHO,INPUT);
  pinMode(SENSOR2_ECHO,INPUT);
  pinMode(SENSOR3_ECHO,INPUT);
  pinMode(SENSOR4_ECHO,INPUT);
  pinMode(SENSOR1_TRIG,OUTPUT);
  pinMode(SENSOR2_TRIG,OUTPUT);
  pinMode(SENSOR3_TRIG,OUTPUT);
  pinMode(SENSOR4_TRIG,OUTPUT);

  state = WAITING_AGENT;
}

void loop()
{
  // State machine to manage connecting and disconnecting the micro-ROS agent
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);

    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      sendSensorInformation();
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
  // other loopy stuff
  // EXECUTE_EVERY_N_MS(500, sendSensorInformation);
}