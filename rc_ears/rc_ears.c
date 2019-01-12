/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc/servo.h>
#include "rc_balance_defs.h"

#define MAIN_HZ (200)
#define LOW_BATT (6.0)
#define SERVO_CHANS (4)
#define RC_SOCKET (5555)
#define UDP_BUFFER_SIZE (1420)
#define HEADER_MAGIC_SZ (8)
#define COMMAND_MAGIC ("apleseed")

typedef struct {
  char header[HEADER_MAGIC_SZ];
  double timestamp;
  double fwd;
  double turn;
  double servos[SERVO_CHANS];
} RCCommand;

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t {
  ARMED,
  DISARMED
} arm_state_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t {
  arm_state_t arm_state;  ///< see arm_state_t declaration
  double theta;   ///< body lean angle (rad)
  double phi;   ///< wheel position (rad)
  double phi_dot;   ///< rate at which phi reference updates (rad/s)
  double gamma;   ///< body turn angle (rad)
  double gamma_dot; ///< rate at which gamma setpoint updates (rad/s)
} setpoint_t;

/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t {
  double wheelAngleR; ///< wheel rotation relative to body
  double wheelAngleL;
  double theta;   ///< body angle radians
  double phi;   ///< average wheel angle in global frame
  double gamma;   ///< body turn (yaw) angle radians
  double vBatt;   ///< battery voltage
  double d1_u;    ///< output of balance controller D1 to motors
  double d2_u;    ///< output of position controller D2 (theta_ref)
  double d3_u;    ///< output of steering controller D3 to motors
  double mot_drive; ///< u compensated for battery voltage
} core_state_t;

// possible modes, user selected with command line arguments
typedef enum m_input_mode_t {
  NONE,
  DSM,
  STDIN
} m_input_mode_t;


// global variables
// Must be declared static so they get zero initalized
static core_state_t cstate;
static setpoint_t setpoint;
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
static rc_mpu_data_t mpu_data;

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
  rc_filter_reset(&D1);
  rc_filter_reset(&D2);
  rc_filter_reset(&D3);
  setpoint.theta = 0.0;
  setpoint.phi   = 0.0;
  setpoint.gamma = 0.0;
  rc_motor_set(0, 0.0);
  return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
  rc_motor_standby(1);
  rc_motor_free_spin(0);
  setpoint.arm_state = DISARMED;
  return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
  __zero_out_controller();
  rc_encoder_eqep_write(ENCODER_CHANNEL_L, 0);
  rc_encoder_eqep_write(ENCODER_CHANNEL_R, 0);
  // prefill_filter_inputs(&D1,cstate.theta);
  rc_motor_standby(0);
  setpoint.arm_state = ARMED;
  return 0;
}

static void enter_pause(void)
{
  rc_led_set(RC_LED_GREEN, 0);
  rc_led_set(RC_LED_RED, 1);
  rc_servo_power_rail_en(0);
  rc_set_state(PAUSED);
}

static void enter_running(void)
{
  rc_led_set(RC_LED_GREEN, 1);
  rc_led_set(RC_LED_RED, 0);
  rc_servo_power_rail_en(1);
  rc_set_state(RUNNING);
}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
  switch (rc_get_state()) {
  case EXITING:
    return;
  case RUNNING: {
    __disarm_controller();
    enter_pause();
    break;
  }
  case PAUSED: {
    __disarm_controller();
    enter_running();
    break;
  }
  default:
    return;
  };
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
  int i;
  const int samples = 100; // check for release 100 times in this period
  const int us_wait = 2000000; // 2 seconds

  // now keep checking to see if the button is still held down
  for (i = 0; i < samples; i++) {
    rc_usleep(us_wait / samples);
    if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED) { return; }
  }
  printf("long press detected, shutting down\n");
  rc_set_state(EXITING);
  return;
}


/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
static int __wait_for_starting_condition(void)
{
  int checks = 0;
  const int check_hz = 20;  // check 20 times per second
  int checks_needed = round(START_DELAY * check_hz);
  int wait_us = 1000000 / check_hz;

  // wait for MiP to be tipped back or forward first
  // exit if state becomes paused or exiting
  while (rc_get_state() == RUNNING) {
    // if within range, start counting
    if (fabs(cstate.theta) > START_ANGLE) { checks++; }
    // fell out of range, restart counter
    else { checks = 0; }
    // waited long enough, return
    if (checks >= checks_needed) { break; }
    rc_usleep(wait_us);
  }
  // now wait for MiP to be upright
  checks = 0;
  // exit if state becomes paused or exiting
  while (rc_get_state() == RUNNING) {
    // if within range, start counting
    if (fabs(cstate.theta) < START_ANGLE) { checks++; }
    // fell out of range, restart counter
    else { checks = 0; }
    // waited long enough, return
    if (checks >= checks_needed) { return 0; }
    rc_usleep(wait_us);
  }
  return -1;
}


/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__((unused)) void* ptr)
{
  // wait for mpu to settle
  __disarm_controller();
  rc_usleep(2500000);
  rc_set_state(RUNNING);
  rc_led_set(RC_LED_RED, 0);
  rc_led_set(RC_LED_GREEN, 1);

  while (rc_get_state() != EXITING) {
    // sleep at beginning of loop so we can use the 'continue' statement
    rc_usleep(1000000 / SETPOINT_MANAGER_HZ);

    // nothing to do if paused, go back to beginning of loop
    if (rc_get_state() != RUNNING) { continue; }

    // if we got here the state is RUNNING, but controller is not
    // necessarily armed. If DISARMED, wait for the user to pick MIP up
    // which will we detected by wait_for_starting_condition()
    if (setpoint.arm_state == DISARMED) {
      if (__wait_for_starting_condition() == 0) {
        __zero_out_controller();
        __arm_controller();
      }
      else { continue; }
    }
  }

  // if state becomes EXITING the above loop exists and we disarm here
  __disarm_controller();
  return NULL;
}

/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{
  static int inner_saturation_counter = 0;
  double dutyL, dutyR;
  /******************************************************************
  * STATE_ESTIMATION
  * read sensors and compute the state when either ARMED or DISARMED
  ******************************************************************/
  // angle theta is positive in the direction of forward tip around X axis
  cstate.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X] + BOARD_MOUNT_ANGLE;

  // collect encoder positions, right wheel is reversed
  cstate.wheelAngleR = (rc_encoder_eqep_read(ENCODER_CHANNEL_R) * 2.0 * M_PI) \
                       / (ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
  cstate.wheelAngleL = (rc_encoder_eqep_read(ENCODER_CHANNEL_L) * 2.0 * M_PI) \
                       / (ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);

  // Phi is average wheel rotation also add theta body angle to get absolute
  // wheel position in global frame since encoders are attached to the body
  cstate.phi = ((cstate.wheelAngleL + cstate.wheelAngleR) / 2) + cstate.theta;

  // steering angle gamma estimate
  cstate.gamma = (cstate.wheelAngleR - cstate.wheelAngleL) \
                 * (WHEEL_RADIUS_M / TRACK_WIDTH_M);

  /*************************************************************
  * check for various exit conditions AFTER state estimate
  ***************************************************************/
  if (rc_get_state() == EXITING) {
    rc_motor_set(0, 0.0);
    return;
  }
  // if controller is still ARMED while state is PAUSED, disarm it
  if (rc_get_state() != RUNNING && setpoint.arm_state == ARMED) {
    __disarm_controller();
    return;
  }
  // exit if the controller is disarmed
  if (setpoint.arm_state == DISARMED) {
    return;
  }

  // check for a tipover
  if (fabs(cstate.theta) > TIP_ANGLE) {
    __disarm_controller();
    printf("tip detected \n");
    return;
  }

  /************************************************************
  * OUTER LOOP PHI controller D2
  * Move the position setpoint based on phi_dot.
  * Input to the controller is phi error (setpoint-state).
  *************************************************************/
  if (ENABLE_POSITION_HOLD) {
    if (fabs(setpoint.phi_dot) > 0.001) { setpoint.phi += setpoint.phi_dot * DT; }
    cstate.d2_u = rc_filter_march(&D2, setpoint.phi - cstate.phi);
    setpoint.theta = cstate.d2_u;
  }
  else { setpoint.theta = 0.0; }

  /************************************************************
  * INNER LOOP ANGLE Theta controller D1
  * Input to D1 is theta error (setpoint-state). Then scale the
  * output u to compensate for changing battery voltage.
  *************************************************************/
  D1.gain = D1_GAIN * V_NOMINAL / cstate.vBatt;
  cstate.d1_u = rc_filter_march(&D1, (setpoint.theta - cstate.theta));

  /*************************************************************
  * Check if the inner loop saturated. If it saturates for over
  * a second disarm the controller to prevent stalling motors.
  *************************************************************/
  if (fabs(cstate.d1_u) > 0.95) { inner_saturation_counter++; }
  else { inner_saturation_counter = 0; }
  // if saturate for a second, disarm for safety
  if (inner_saturation_counter > (SAMPLE_RATE_HZ * D1_SATURATION_TIMEOUT)) {
    printf("inner loop controller saturated\n");
    __disarm_controller();
    inner_saturation_counter = 0;
    return;
  }

  /**********************************************************
  * gama (steering) controller D3
  * move the setpoint gamma based on user input like phi
  ***********************************************************/
  if (fabs(setpoint.gamma_dot) > 0.0001) { setpoint.gamma += setpoint.gamma_dot * DT; }
  cstate.d3_u = rc_filter_march(&D3, setpoint.gamma - cstate.gamma);

  /**********************************************************
  * Send signal to motors
  * add D1 balance control u and D3 steering control also
  * multiply by polarity to make sure direction is correct.
  ***********************************************************/
  dutyL = cstate.d1_u - cstate.d3_u;
  dutyR = cstate.d1_u + cstate.d3_u;
  rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
  rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);

  return;
}


/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__((unused)) void* ptr)
{
  double new_v;
  while (rc_get_state() != EXITING) {
    new_v = rc_adc_batt();
    // if the value doesn't make sense, use nominal voltage
    if (new_v > 9.0 || new_v < 5.0) { new_v = V_NOMINAL; }
    cstate.vBatt = new_v;
    rc_usleep(1000000 / BATTERY_CHECK_HZ);
  }
  return NULL;
}


/** Parse commands from incomming packets
 */
bool parse_commands(char* buffer, int bytes, RCCommand* dst)
{
  int i;
  // Verify packet size
  if (bytes != sizeof(RCCommand)) {
    fprintf(stderr, "Invalid packet size, %d != %d\n", bytes, sizeof(RCCommand));
    return false;
  }
  // Verify packet header magic
  for (i = 0; i < HEADER_MAGIC_SZ; ++i) {
    if (buffer[i] != COMMAND_MAGIC[i]) {
      fprintf(stderr, "Unexpected packet header\n");
      return false;
    }
  }
  // Just binary copy
  memcpy(dst, buffer, bytes);
  return true;
}

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
  pthread_t setpoint_thread = 0;
  pthread_t battery_thread = 0;
  int udp_socket, n_bytes;
  char sock_buffer[UDP_BUFFER_SIZE];
  struct sockaddr_in server_addr;
  struct sockaddr_storage server_storage;
  socklen_t addr_size;
  int i;
  // make sure another instance isn't running
  // if return value is -3 then a background process is running with
  // higher privaledges and we couldn't kill it, in which case we should
  // not continue or there may be hardware conflicts. If it returned -4
  // then there was an invalid argument that needs to be fixed.
  if (rc_kill_existing_process(2.0) < -2) { return -1; }

  // start signal handler so we can exit cleanly
  if (rc_enable_signal_handler() == -1) {
    fprintf(stderr, "ERROR: failed to start signal handler\n");
    return -1;
  }

  // initialize pause button
  if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                     RC_BTN_DEBOUNCE_DEFAULT_US)) {
    fprintf(stderr, "ERROR: failed to initialize pause button\n");
    return -1;
  }

  // Assign functions to be called when button events occur
  rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, on_pause_release);

  // make PID file to indicate your project is running
  // due to the check made on the call to rc_kill_existing_process() above
  // we can be fairly confident there is no PID file already and we can
  // make our own safely.
  rc_make_pid_file();

  if (rc_adc_init()) {
    fprintf(stderr, "ERROR: failed to run rc_adc_init()\n");
    return -1;
  }

  // initialize PRU
  if (rc_servo_init()) { return -1; }

  // initialize enocders
  if (rc_encoder_eqep_init() == -1) {
    fprintf(stderr, "ERROR: failed to initialize eqep encoders\n");
    return -1;
  }

  // initialize motors
  if (rc_motor_init() == -1) {
    fprintf(stderr, "ERROR: failed to initialize motors\n");
    return -1;
  }
  rc_motor_standby(1); // start with motors in standby

  printf("\nPress and release MODE button to toggle DSM drive mode\n");
  printf("Press and release PAUSE button to pause/start the motors\n");
  printf("hold pause button down for 2 seconds to exit\n");

  if (rc_led_set(RC_LED_GREEN, 0) == -1) {
    fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
    return -1;
  }
  if (rc_led_set(RC_LED_RED, 1) == -1) {
    fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
    return -1;
  }

  // set up mpu configuration
  rc_mpu_config_t mpu_config = rc_mpu_default_config();
  mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
  mpu_config.orient = ORIENTATION_Y_UP;

  // if gyro isn't calibrated, run the calibration routine
  if (!rc_mpu_is_gyro_calibrated()) {
    printf("Gyro not calibrated, automatically starting calibration routine\n");
    printf("Let your MiP sit still on a firm surface\n");
    rc_mpu_calibrate_gyro_routine(mpu_config);
  }

  // make sure setpoint starts at normal values
  setpoint.arm_state = DISARMED;

  // set up D1 Theta controller
  double D1_num[] = D1_NUM;
  double D1_den[] = D1_DEN;
  if (rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)) {
    fprintf(stderr, "ERROR in rc_balance, failed to make filter D1\n");
    return -1;
  }
  D1.gain = D1_GAIN;
  rc_filter_enable_saturation(&D1, -1.0, 1.0);
  rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

  // set up D2 Phi controller
  double D2_num[] = D2_NUM;
  double D2_den[] = D2_DEN;
  if (rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)) {
    fprintf(stderr, "ERROR in rc_balance, failed to make filter D2\n");
    return -1;
  }
  D2.gain = D2_GAIN;
  rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
  rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

  printf("Inner Loop controller D1:\n");
  rc_filter_print(D1);
  printf("\nOuter Loop controller D2:\n");
  rc_filter_print(D2);

  // set up D3 gamma (steering) controller
  if (rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4 * DT, DT)) {
    fprintf(stderr, "ERROR in rc_balance, failed to make steering controller\n");
    return -1;
  }
  rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

  /*Create UDP socket*/
  udp_socket = socket(PF_INET, SOCK_DGRAM, 0);
  /*Configure settings in address struct*/
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(RC_SOCKET);
  server_addr.sin_addr.s_addr = INADDR_ANY;
  memset(server_addr.sin_zero, '\0', sizeof(server_addr.sin_zero));
  /*Bind socket with address struct*/
  bind(udp_socket, (struct sockaddr *) &server_addr, sizeof(server_addr));
  /*Initialize size variable to be used later on*/
  addr_size = sizeof(server_storage);

  // start a thread to slowly sample battery
  if (rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)) {
    fprintf(stderr, "failed to start battery thread\n");
    return -1;
  }

  // wait for the battery thread to make the first read
  while (cstate.vBatt < 1.0 && rc_get_state() != EXITING) { rc_usleep(10000); }

  // start mpu
  if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
    fprintf(stderr, "ERROR: can't talk to IMU, all hope is lost\n");
    rc_led_blink(RC_LED_RED, 5, 5);
    return -1;
  }

  // start balance stack to control setpoints
  if (rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)) {
    fprintf(stderr, "failed to start battery thread\n");
    return -1;
  }

  // this should be the last step in initialization
  // to make sure other setup functions don't interfere
  rc_mpu_set_dmp_callback(&__balance_controller);

  // start in the RUNNING state, pressing the pause button will swap to
  // the PAUSED state then back again.
  printf("\nHold your MIP upright to begin balancing\n");

  // Keep looping until state changes to EXITING
  enter_running();
  while (rc_get_state() != EXITING) {
    RCCommand command;
    n_bytes = recvfrom(udp_socket, sock_buffer, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&server_storage, &addr_size);
    parse_commands(sock_buffer, n_bytes, &command);

    if (rc_get_state() == RUNNING) {
      const double batt = rc_adc_batt();
      if (batt < LOW_BATT) {
        fprintf(stderr, "ERROR: battery disconnected or insufficiently charged to drive servos, %f\n", batt);
        enter_pause();
      }

      setpoint.phi_dot = command.fwd;
      setpoint.gamma_dot = command.turn;

      for (i = 0; i < SERVO_CHANS; ++i) {
        if (rc_servo_send_pulse_normalized(i, command.servos[i]) == -1) {
          return -1;
        }
      }
    }
    // always sleep at some point
    rc_usleep(1000000 / MAIN_HZ);
  }

  // turn off LEDs and close file descriptors
  // join threads
  rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
  if (battery_thread) { rc_pthread_timed_join(battery_thread, NULL, 1.5); }

  // cleanup
  rc_filter_free(&D1);
  rc_filter_free(&D2);
  rc_filter_free(&D3);
  rc_mpu_power_off();
  rc_encoder_eqep_cleanup();
  rc_adc_cleanup();
  rc_servo_power_rail_en(0);
  rc_servo_cleanup();
  rc_led_set(RC_LED_GREEN, 0);
  rc_led_set(RC_LED_RED, 0);
  rc_led_cleanup();
  rc_button_cleanup();  // stop button handlers
  rc_remove_pid_file(); // remove pid file LAST
  return 0;
}
