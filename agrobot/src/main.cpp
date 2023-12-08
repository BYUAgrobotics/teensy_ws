#include <Arduino.h>
#include <agro_interfaces/srv/process.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 1

// micro-ROS objects
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_service_t service;
static agro_interfaces__srv__Process_Response process_resp;
static agro_interfaces__srv__Process_Request process_req;

// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

// responds to errors with micro-ROS functions
static void error_loop() {
  while (1) {
    delay(100);
  }
}

void process_srv_callback(const void *request_msg, void *response_msg) {
  agro_interfaces__srv__Process_Response *res_in =
      (agro_interfaces__srv__Process_Response *)response_msg;

  ////////////////////////////////////////////////////////////
  // ADD CODE HERE TO EXECUTE WHEN A SERVICE IS REQUESTED
  ////////////////////////////////////////////////////////////

  x = request_msg->target_x;
  y = request_msg->target_y;
  z = request_msg->target_z;

  ////////////////////////////////////////////////////////////
  // END CODE HERE TO EXECUTE WHEN A SERVICE IS REQUESTED
  ////////////////////////////////////////////////////////////

  res_in->finished = true;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create service
  RCCHECK(rclc_service_init_best_effort(
      &service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(agro_interfaces, srv, Process), "process"));

  // create executor
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL,
                                 &allocator));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &service, &process_req,
                                        &process_resp, process_srv_callback));

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_service_fini(&service, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

  ////////////////////////////////////////
  // ADD MOTOR INIT CODE HERE
  ////////////////////////////////////////

  ////////////////////////////////////////
  // END MOTOR INIT CODE HERE
  ////////////////////////////////////////

  state = WAITING_AGENT;
}

void loop() {
  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);

    if (state == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}