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
  if (rc_get_state() == RUNNING) { enter_pause(); }
  else if (rc_get_state() == PAUSED) { enter_running(); }
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

/** Parse commands from incomming packets
 */
bool parse_commands(char* buffer, int bytes, RCCommand* dst) {
  int i;
  // Verify packet size
  if (bytes != sizeof(RCCommand)) {
    fprintf(stderr, "Invalid packet size, %d != %d\n", bytes, sizeof(RCCommand));
    return false;
  }
  // Verify packet header magic
  for (i=0; i<HEADER_MAGIC_SZ; ++i) {
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

      for (i=0; i<SERVO_CHANS; ++i) {
        if (rc_servo_send_pulse_normalized(i, command.servos[i]) == -1) {
          return -1;
        }
      }
    }
    // always sleep at some point
    rc_usleep(1000000 / MAIN_HZ);
  }

  // turn off LEDs and close file descriptors
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
