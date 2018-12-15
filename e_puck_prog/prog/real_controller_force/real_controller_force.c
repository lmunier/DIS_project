/*****************************************************************************/
/* File:         real_controller.c                                               */
/* Version:      1.1                                                         */
/* Date:         12.dec.2018                                                   */
/* Description:  Reynolds flocking with relative positions	           */
/*****************************************************************************/


#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>
#include <time.h>

/*Fancy definitions*/
typedef unsigned int uint16_t;  // 65300
typedef int int16_t;            //32768
typedef unsigned char uint8_t;  //255
typedef char int8_t;            //127

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define FLOCK_SIZE 3    // Size of flock
#define TIME_STEP 64    // [ms] Length of time step
#define DELTA_T 0.064   // Timestep (seconds)

#define AXLE_LENGTH 0.052         // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628   // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205       // Wheel radius (meters)
#define M_PI 3.14

#define NB_SENSORS 8            // Number of distance sensors
#define OBSTACLE_THRESHOLD 90   // Value to detect an obstacle
#define MIN_SENS 300            // Minimum sensibility value
#define MAX_SENS 4096           // Maximum sensibility value
#define MAX_SPEED 800           // Maximum speed

#define TRUE 1      // true value

#define RULE1_THRESHOLD  0.25           // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT (0.3 / 10)         // Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD  0.1            // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT (0.5 / 10)         // Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT (0.02 / 10)        // Weight of consistency rule. default 1.0/10
#define MARGINAL_THRESHOLD 40
#define MIGRATION_WEIGHT (0.03 / 10)    // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 1                // Tells the robots if they should just go forward or move towards a specific myself.migratory direction

#define TIMEOUT 50    // Limit time to consider a robot in the flock in 100ms
#define NB_TASK 2

/*Adding correction factor*/
#define K_X	1.0
#define K_Y	1.0
#define K_TH	1.0
#define K_U 	0.2   // Forward control coefficient
#define K_W 	1  	  // Rotational control coefficient

int e_puck_matrix[16] = {
    17,  29,  34,  10, 8,  -38, -56, -76,
    -72, -58, -36, 8,  10, 36,  28,  18};  // for obstacle avoidance

struct robot {
  int ID;    // ID of myself
  int init;  // Control if initialized, 0=not, 1=yes
  float distances[FLOCK_SIZE][2];  // Store the distances between myself and other robots
  float previousDistances[FLOCK_SIZE][2];  // Store the previous distances
                                           // between myself and other robots
  float relAngle[FLOCK_SIZE];  // Store the relative angle of the other robots
  float speed[FLOCK_SIZE][2];  // Store the relative speed of the other robots
                               // and the speed of myself
  float my_position[3];        // Initial position of myself: X, Z and theta
  float my_previous_position[3];  // Initial position of myself: X, Z and theta
  float migr[2];                  // Position for the myself.migratory urge
  int send;
} myself;

// float myself.distances[FLOCK_SIZE][3];  // relative X, Z, Theta of all robots
// float myself.previousDistances[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
// float myself.speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
// float myself.my_position[3];                 // X, Z, Theta of the current robot
// float myself.my_previous_position[3];   // X, Z, Theta of the current robot in the previous
//                              // time step
// float speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
// float initialized[FLOCK_SIZE];  // != 0 if initial positions have been received
// float myself.migr[2] = {0, 5};    // myself.migration vector
// char *robot_name;

uint16_t is_in_flock[FLOCK_SIZE][2];   //Store if a robot is out of range. first colomn is the time of the last message received, second 0 if out 1 if not.

float theta_robots[FLOCK_SIZE];
int get_initial_position = 1;     // Indicate if the robot should get its initial position from robot 0

// Structure to get time
struct time{
  uint16_t timer_count_ms;      // Time in ms
  uint16_t timer_count_10ms;    // Time in 10ms
  uint16_t timer_count_100ms;   // Time in 100ms
  int8_t   timer_reset;
  int8_t   timer_done[NB_TASK];
}myTime;

int nb = 0;                               // Variable to not saturate the communication canal
// Nasty arrays in our code
char tmp[128];


/*int myself.distances[FLOCK_SIZE][3];  // relative X, Z, Theta of all robots
int myself.previousDistances[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
int myself.speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
int myself.my_position[3];                 // X, Z, Theta of the current robot
int myself.my_previous_position[3];   // X, Z, Theta of the current robot in the previous
                             // time step
int speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];  // != 0 if initial positions have been received
int myself.migr[2] = {0, 5};    // myself.migration vector
char *robot_name;
int state_robot=0;

int theta_robots[FLOCK_SIZE];
int get_initial_position = 0;
int t;*/

//////////////////* Definition des fonctions *//////////////////////////////////

void reset(void);
int getselector(void);
void limit(int *, int);
void update_self_motion(int, int);
void compute_wheel_speeds(int *, int *);
void reynolds_rules(void);
void send_ping(void);
void process_received_ping_messages(void);
void wait(uint16_t ms);
void silly_timer(void);
void check_if_out_of_range(void);
void compute_obstacle(float *value_x, float *value_z);

/////////////////////////* the main function    */////////////
int main()
{
  float force_x = 0.0, force_z = 0.0;
  int msl, msr;  // Wheel speeds
  // int bmsl, bmsr, sum_sensors;  // Braitenberg parameters
  // int i;                    // Loop counter
  // int distances[NB_SENSORS];    // Array for the distance sensor readings
  // int max_sens;

  reset();

  wait(TIME_STEP);

  ircomStart();
  ircomEnableContinuousListening();
  ircomListen();

  while(TRUE)
  {
    /* state_robotand get information */
    if(nb%4 == 0){
      send_ping();  // sending a ping to other robot, so they can measure their
      nb = 0;
    }

    /// Compute self position-pedantic
    myself.my_previous_position[0] = myself.my_position[0];
    myself.my_previous_position[1] = myself.my_position[1];
    myself.my_previous_position[2] = myself.my_position[2];

    update_self_motion(msl, msr);

    process_received_ping_messages();

    check_if_out_of_range(); // Check if there is robots out of range regarding the timeout

    myself.speed[myself.ID][0] = (1 / DELTA_T) * (myself.my_position[0] - myself.my_previous_position[0]);
    myself.speed[myself.ID][1] = (1 / DELTA_T) * (myself.my_position[1] - myself.my_previous_position[1]);

    // Reynold's rules with all previous info (updates the speed[][] table)
    reynolds_rules();

    // Compute virtual force to avoid obstacle
    compute_obstacle(&force_x, &force_z);

    // Compute wheels speed from reynold's speed
    compute_wheel_speeds(&msl, &msr);

    limit(&msl, MAX_SPEED);
    limit(&msr, MAX_SPEED);

    e_set_speed_left(0);
    e_set_speed_right(0);

    nb++;
    wait(TIME_STEP);
  }
  return 0;
}

// Return the value of the selector
int getselector()
{
  return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}
/////////////////////////Implementation des fonctions /////////////////////////
void silly_timer(void)
{
  if(myTime.timer_reset == 0)
  {
    myTime.timer_done[0] = 0;
    myTime.timer_reset   = 1;
  }
  else
  {
    if((myTime.timer_count_ms+1)%10 == 0){
      myTime.timer_count_ms = 0;
      myTime.timer_count_10ms++;
    }
    if((myTime.timer_count_10ms+1)%10 == 0){
      myTime.timer_count_10ms = 0;
      myTime.timer_count_100ms++;
    }
    myTime.timer_count_ms++;
    myTime.timer_done[0] = 1;
  }
}

void wait_task(void)
{
  myTime.timer_done[1]=1;
}

void wait(uint16_t ms)
{
  myTime.timer_done[1] = 0;
  e_activate_agenda(wait_task,10*ms);
  while(myTime.timer_done[1]==0);
  e_destroy_agenda(wait_task);
  return;
}

///////////////* Reset le robot  et recupere l'ID du robot *///////////////////
void reset(void)
{
  e_init_port();
  e_init_ad_scan();
  e_init_uart1();
  e_led_clear();
  e_init_motors();

  myTime.timer_count_ms=0;
  myTime.timer_count_10ms=0;
  myTime.timer_count_100ms=0;
  myTime.timer_reset=0;
  myTime.timer_done[0] = 0;
  myTime.timer_done[1] = 0;

  e_start_agendas_processing();
  e_calibrate_ir();
  e_activate_agenda(silly_timer,10);

  int i;
  myself.ID = getselector();

  // sprintf(tmp, "reset_myself.ID=%d \n", myself.ID);
  // btcomSendString(tmp);

  for(i=0; i<FLOCK_SIZE; i++)
  {
    myself.my_position[i] = 0;
    myself.my_previous_position[i] = 0;
    is_in_flock[i][1] = 1;
    is_in_flock[i][0] = myTime.timer_count_100ms;
  }

  if(myself.ID == 0)
    get_initial_position = 0;
}

/////////////*Keep given int number within interval {-limit, limit}*////////

void limit(int *number, int limit)
{
  if (*number > limit) *number = limit;
  if (*number < -limit) *number = -limit;
}

void compute_obstacle(float *value_x, float *value_z){
  float angle_epuck[8] = {1.27, 0.77, 0, 5.21, 4.21, 3.14, 2.37, 1.87};
  int distances[NB_SENSORS] = {0};  // Array for the distance sensor readings
  int i = 0;
  float sum_angle = 0;
  float force_x = 0.0, force_z = 0.0;
  float norm = 0.0;
  float mean = 0, sigma = 0.2;

  for(i = 0; i < NB_SENSORS; i++){
    distances[i] = e_get_calibrated_prox(i);

    if(distances[i] > OBSTACLE_THRESHOLD){
      distances[i] += MAX_SENS*box_muller(mean, sigma)/2;

      force_x += cosf(angle_epuck[i])*distances[i] / (NB_SENSORS*MAX_SENS);
      force_z += sinf(angle_epuck[i])*distances[i] / (NB_SENSORS*MAX_SENS);
    }
  }
  *value_x = force_x;
  *value_z = force_z;
}
////////////* Updates robot position with wheel speeds *///////////////////////

void update_self_motion(int msl, int msr)
{
  float theta = myself.my_position[2];

  // Compute deltas of the robot
  float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float du = (dr + dl) / 2.0;
  float dtheta = (dr - dl) / (2.0 * AXLE_LENGTH);

  // Compute deltas in the environment
  float dx = -du * sinf(theta);
  float dz = -du * cosf(theta);

  // Update position
  myself.my_position[0] += K_X  * dx;
  myself.my_position[1] += K_Y  * dz;
  myself.my_position[2] += K_TH * dtheta;

  // Keep orientation within 0, 2pi
  if (myself.my_position[2] > 2 * M_PI) myself.my_position[2] -= 2.0 * M_PI;
  if (myself.my_position[2] < 0) myself.my_position[2] += 2.0 * M_PI;

}
////////////////////* Computes wheel speed given a certain X,Z speed *//////////////

void compute_wheel_speeds(int *msl, int *msr)
{
  // x in robot coordinates
  float x = myself.speed[myself.ID][0] * cosf(myself.my_position[2]) + myself.speed[myself.ID][1] * sinf(myself.my_position[2]);
  // z in robot coordinates
  float z = -myself.speed[myself.ID][0] * sinf(myself.my_position[2]) + myself.speed[myself.ID][1] * cosf(myself.my_position[2]);

  float range = sqrtf(x * x + z * z);  // Distance to the wanted position
  float bearing = -atan2(x, z);        // Orientation of the wanted position

  float u = K_U * range * cosf(bearing);  // Compute7 forward control
  float w = K_W * bearing;                // Compute rotational control

  // Convert to wheel speeds!int
  *msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
  *msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);

  limit(msl, MAX_SPEED);
  limit(msr, MAX_SPEED);
}

///////////////*Check if other robots are out of range *//////////////////
void check_if_out_of_range(){
  int i;
  for(i=0;i<FLOCK_SIZE;i++){
    if(i==myself.ID)
      continue;
    else{
      sprintf(tmp,"delta time: %d\n", myTime.timer_count_100ms - is_in_flock[i][0]);
      btcomSendString(tmp);
      if((myTime.timer_count_100ms - is_in_flock[i][0]) >= TIMEOUT){
        is_in_flock[i][1] = 0;
        sprintf(tmp,"IS OUT: %d\n", i);
        btcomSendString(tmp);
      }
      else{
        is_in_flock[i][1] = 1;
        sprintf(tmp,"IS BACK: %d\n", i);
        btcomSendString(tmp);
      }
    }
  }
}

///////////////*Update speed according to Reynold's rules *//////////////////

void reynolds_rules()
{
  int i, j;                      // Loop counters
  float rel_avg_loc[2] = {0, 0};    // Flock average positions
  float rel_avg_speed[2] = {0, 0};  // Flock average speeds
  float cohesion[2] = {0, 0};
  float dispersion[2] = {0, 0};
  float consistency[2] = {0, 0};
  float dist = 0;
  int nb_neighbours = 0;

  /* Compute averages over the whole flock */
  for (i = 0; i < FLOCK_SIZE; i++)
  {
    if (i != myself.ID)
    {
      dist = sqrtf(myself.distances[i][0] * myself.distances[i][0] +
                  myself.distances[i][1] * myself.distances[i][1]);
      if (dist < MARGINAL_THRESHOLD)
      {
        for (j = 0; j < 2; j++)
        {
          rel_avg_speed[j] += myself.speed[i][j];
          rel_avg_loc[j] += myself.distances[i][j];
          nb_neighbours++;
        }
      }
    }
  }

  // Calcul de la moyenne
  for (j = 0; j < 2; j++) {
    if (nb_neighbours != 0) {
      rel_avg_loc[j] /= (nb_neighbours + 1);
      rel_avg_speed[j] /= (nb_neighbours + 1);
    }
  }


  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  for (j = 0; j < 2; j++) {
    //If center of mass is too far
    if(fabs(rel_avg_loc[j]) > RULE1_THRESHOLD) {
        cohesion[j] = rel_avg_loc[j];
    }
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (myself.ID != i) {
      if(pow(myself.distances[i][0],2)+pow(myself.distances[i][1],2) < RULE2_THRESHOLD) {
        for (j=0;j<2;j++) {
            dispersion[j] -= myself.distances[i][j];
        }
      }
    }
  }

  /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
  for (j = 0; j < 2; j++) {
    consistency[j] = rel_avg_speed[j];
  }

  // aggregation of all behaviors with relative influence determined by weights
  for (j = 0; j < 2; j++) {
    myself.speed[myself.ID][j] = cohesion[j] * RULE1_WEIGHT;
    myself.speed[myself.ID][j] += dispersion[j] * RULE2_WEIGHT;
    myself.speed[myself.ID][j] += consistency[j] * RULE3_WEIGHT;
  }
  myself.speed[myself.ID][1] *= -1;  // z axis of webots is inverted
}

/////////////////////* Envoie du message *///////////////////////////////////

void send_ping(void)
{
    ircomSend(myself.ID);
    while (ircomSendDone() == 0);
}

////////////////////* Processs receive message *////////////////////////////////

void process_received_ping_messages(void)
{
  float message_direction;
  float message_rssi;  // Received Signal Strength indicator
  float range;
  int emitter_id;

  IrcomMessage imsg;
  ircomPopMessage(&imsg);

  while(imsg.error != -1)
  {
    message_direction = (float)imsg.direction;
    message_rssi = (float)imsg.distance;
    emitter_id = (int)imsg.value;

    if(emitter_id == myself.ID || emitter_id >= FLOCK_SIZE){
      ircomPopMessage(&imsg);
      continue;
    }

    is_in_flock[emitter_id][0] = myTime.timer_count_100ms;

    myself.distances[emitter_id][2] = message_direction;
    myself.distances[emitter_id][2] += myself.my_position[2];  // find the relative theta;
    range = message_rssi;
    if(get_initial_position == 1) {
      myself.my_position[0] = range*cos(myself.distances[emitter_id][2]);
      myself.my_position[1] = -1.0 * range*sin(myself.distances[emitter_id][2]);
      myself.my_position[2] = 0;
      myself.my_previous_position[0] = myself.my_position[0];   // relative x pos
      myself.my_previous_position[1] = myself.my_position[1];   // relative y pos
      myself.my_previous_position[2] = myself.my_position[2];
      get_initial_position = 0;
    } else {
      myself.previousDistances[emitter_id][0] = myself.distances[emitter_id][0];
      myself.previousDistances[emitter_id][1] = myself.distances[emitter_id][1];
      myself.previousDistances[emitter_id][2] = myself.distances[emitter_id][2];
      myself.distances[emitter_id][0] = range*cos(myself.distances[emitter_id][2]);  // relative x pos
      myself.distances[emitter_id][1] = -1.0 * range*sin(myself.distances[emitter_id][2]);   // relative y pos
      myself.speed[emitter_id][0] = 1.0*(1/DELTA_T)*(myself.distances[emitter_id][0]-myself.previousDistances[emitter_id][0]);
      myself.speed[emitter_id][1] = 1.0*(1/DELTA_T)*(myself.distances[emitter_id][1]-myself.previousDistances[emitter_id][1]);
    }
    ircomPopMessage(&imsg);
  }
}
