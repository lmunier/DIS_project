/*****************************************************************************/
/* File:         real_controller_force_crossing.c                                               */
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
#include <stdlib.h>
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

#define FLOCK_SIZE 6    // Size of flock
#define TIME_STEP 64    // [ms] Length of time step
#define DELTA_T 0.064   // Timestep (seconds)

#define AXE_LENGTH 0.052          // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628   // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205       // Wheel radius (meters)
#define PI 3.14

#define NB_SENSORS 8            // Number of distance sensors
#define OBSTACLE_THRESHOLD 80   // Value to detect an obstacle
#define MIN_SENS 300            // Minimum sensibility value
#define MAX_SENS 4095           // Maximum sensibility value
#define MAX_SPEED 800           // Maximum speed

#define TRUE 1                  // true value

#define RULE1_THRESHOLD  0.05           // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT (9 / 10)           // Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD  (0.1 / 10)     // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT (0.15 / 10)        // Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT (0.03 / 10)        // Weight of consistency rule. default 1.0/10
#define MARGINAL_THRESHOLD 40
#define MIGRATION_WEIGHT (0.03 / 10)    // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 1                // Tells the robots if they should just go forward or move towards a specific myself.migratory direction
#define MIGRATORY_TARGET_X -0.5

#define TIMEOUT 50    // Limit time to consider a robot in the flock in 100ms
#define NB_TASK 2

/*Adding correction factor*/
#define K_X	1.0
#define K_Y	1.0
#define K_TH	1.0
#define K_U 	0.2   // Forward control coefficient
#define K_W 	1  	  // Rotational control coefficient
#define K_OLD 0.2

// Factor to tune the obstacle avoidance compared to the Reynolds rules
#define OBSTACLE_WEIGHT 20.0
#define REYNOLDS_WEIGHT 80.0
#define REYNOLDS_WEIGHT_NO_OBSTACLE 70

struct robot {
  int ID;                                   // ID of myself
  int get_initial_position;                 // Control if initialized, 0=not, 1=yes
  float distances[FLOCK_SIZE][2];           // Store the distances between myself and other robots
  float previousDistances[FLOCK_SIZE][2];   // Store the previous distances
                                            // between myself and other robots
  float relAngle[FLOCK_SIZE];               // Store the relative angle of the other robots
  float speed[FLOCK_SIZE][2];               // Store the relative speed of the other robots
                                            // and the speed of myself
  float my_position[3];                     // Initial position of myself: X, Z and theta
  float my_previous_position[3];            // Initial position of myself: X, Z and theta
  float migr[2];                            // Position for the myself.migratory urge
  int flock_ID[FLOCK_SIZE];                 // ID of my flock
} myself;

uint16_t is_in_flock[FLOCK_SIZE][2];   //Store if a robot is out of range. first colomn is the time of the last message received, second 0 if out 1 if not.

// Structure to get time
struct time{
  uint16_t timer_count_100ms;   // Time in 100ms
  int8_t   timer_reset;         // Timer reset flag
  int8_t   timer_done[NB_TASK]; // Task done
}myTime;

//////////////////* Definition des fonctions *//////////////////////////////////

void reset(void);                                           // Reset the robot
int getselector(void);                                      // Get the value of the selector
void limit(int *, int);                                     // Check if the value exced the boundaries
void update_self_motion(int, int);                          // Update the position of myself
void compute_wheel_speeds(int *, int *, float, float);      // Compute the motors speed
void reynolds_rules(void);                                  // Apply Reynolds rules
void send_ping(void);                                       // Send a message
void process_received_ping_messages(void);                  // Process the received message
void wait(uint16_t);                                        // Wait
void timer(void);                                           // Compute time in 100ms
void check_if_out_of_range(void);                           // Check if a robot is out of range
void compute_obstacle(float *, float *);                    // Compute the force of an obstacle
float box_muller(float, float);                             // Add noise

/////////////////////////* the main function    */////////////
int main()
{
  float force_x = 0.0, force_z = 0.0;   // Virtual forces
  int msl, msr;                         // Wheel speeds
  int nb = 1;                           // Message counter

  reset();

  wait(2000);

  ircomStart();
  ircomEnableContinuousListening();
  ircomListen();

  // Reset timer
  myTime.timer_reset = 0;
  while(myTime.timer_reset == 0);

  while(TRUE)
  {
    // Send a message each 4 cycles
    if(nb%4 == 0){
      send_ping();  // sending a ping to other robot, so they can measure their
      nb = 1;
    }

    /// Compute self position-pedantic
    myself.my_previous_position[0] = myself.my_position[0];
    myself.my_previous_position[1] = myself.my_position[1];
    myself.my_previous_position[2] = myself.my_position[2];

    // Update self positions
    update_self_motion(msl, msr);

    // Process the received message
    process_received_ping_messages();

    // Check if there is robots out of range regarding the timeout
    check_if_out_of_range();

    // Update the speeds of myself
    myself.speed[myself.ID][0] = (1 / DELTA_T) * (myself.my_position[0] - myself.my_previous_position[0]);
    myself.speed[myself.ID][1] = (1 / DELTA_T) * (myself.my_position[1] - myself.my_previous_position[1]);

    // Reynold's rules with all previous info (updates the speed[][] table)
    reynolds_rules();

    // Compute virtual force to avoid obstacle
    compute_obstacle(&force_x, &force_z);

    // Compute wheels speed from reynold's speed
    compute_wheel_speeds(&msl, &msr, force_x, force_z);

    // Check the speeds values
    limit(&msl, MAX_SPEED);
    limit(&msr, MAX_SPEED);

    // Set motor speeds
    e_set_speed_left(msl);
    e_set_speed_right(msr);

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
// Timer in 100ms
void timer(void)
{
  if(myTime.timer_reset == 0)
  {
    myTime.timer_done[0] = 0;
    myTime.timer_count_100ms = 0;
    myTime.timer_reset   = 1;
  }
  else
  {
    myTime.timer_count_100ms++;
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

  // Initialize timer variables
  myTime.timer_count_100ms=0;
  myTime.timer_reset=0;
  myTime.timer_done[0] = 0;
  myTime.timer_done[1] = 0;

  e_start_agendas_processing();
  e_calibrate_ir();
  e_activate_agenda(timer,1000);        // Timer call each 100ms

  int i;

  // Get the ID of the robot
  myself.ID = getselector();

  // Initialization of myself
  for(i=0; i<FLOCK_SIZE; i++)
  {
    myself.distances[i][0] = 0;          // Initialize distance tab X
    myself.distances[i][1] = 0;          // Initialize distance tab Z
    myself.previousDistances[i][0] = 0;  // Initialize distance tab X
    myself.previousDistances[i][1] = 0;  // Initialize distance tab Z
    myself.relAngle[i] = 0;              // Initialize relative angle tab
    myself.speed[i][0] = 0;              // Initialize speed tab X
    myself.speed[i][1] = 0;              // Initialize speed tab Z
    is_in_flock[i][0] = 0;               // Initialze last message time to 0
    is_in_flock[i][1] = 1;               // Initialized the robot as "in flock"
    switch(i){
      case 0:
      case 1:
      case 2: myself.flock_ID[i]=0; break;
      case 3:
      case 4:
      case 5: myself.flock_ID[i]=1; break;
    }
  }

  // My initial positions
  for(i=0;i<2;i++){
    myself.my_position[i] = 0;
    myself.my_previous_position[i] = 0;
  }

  // My initial orientation
  myself.my_position[2] = PI/2;
  myself.my_previous_position[2] = PI/2;

  // Set of the migratory urge
  myself.migr[0] = MIGRATORY_TARGET_X;//30;//(myself.my_position[0]-MIGRATORY_TARGET_X);
  myself.migr[1] = 0;//15;//(myself.my_position[1]-MIGRATORY_TARGET_Z);//0;

  // Define if the robot has to get its initial position from robot 0
  if(myself.ID == 0)
    myself.get_initial_position = 0;
  else
    myself.get_initial_position = 1;
}

/////////////*Keep given int number within interval {-limit, limit}*////////
void limit(int *number, int limit)
{
  if (*number > limit) *number = limit;
  if (*number < -limit) *number = -limit;
}

// compute the force of the obstacle
void compute_obstacle(float *value_x, float *value_z){
  float angle_epuck[8] = {1.27, 0.77, 0, 5.21, 4.21, 3.14, 2.37, 1.87};
  unsigned int distances[NB_SENSORS] = {0};  // Array for the distance sensor readings
  int i = 0;
  float force_x = 0.0, force_z = 0.0;
  float mean = 0, sigma = 0.2;

  // Get sensors values
  for(i = 0; i < NB_SENSORS; i++){
    distances[i] = abs(e_get_calibrated_prox(i));

    // Keep only close obstacles
    if(distances[i] > OBSTACLE_THRESHOLD){
      distances[i] += MAX_SENS*box_muller(mean, sigma)/2;

      // Compute forces
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
  float dist_left, dist_right;
  float dx, dz, dtheta;
  float R;

  // Compute the traveled distance for each wheels
  dist_left = e_get_steps_left()*2.0*PI*WHEEL_RADIUS/1000.0;
  e_set_steps_left(0);
  dist_right = e_get_steps_right()*2.0*PI*WHEEL_RADIUS/1000.0;
  e_set_steps_right(0);

  // Compute the angle variation
  dtheta = (dist_left-dist_right)/AXE_LENGTH;

  // Control if there is or not a rotation
  if(dtheta != 0){
    R = (dist_left+dist_right)/(2.0*dtheta);
    dx = R*sin(dtheta);
    dz = R*(1-cos(dtheta));
  }else{
    dx = 0;
    dz = dist_left;
  }

  // Save and update myself positions
  myself.my_previous_position[0] = myself.my_position[0];
  myself.my_previous_position[1] = myself.my_position[1];
  myself.my_previous_position[2] = myself.my_position[2];
  myself.my_position[0] += dx;
  myself.my_position[1] += dz;
  myself.my_position[2] -= dtheta;

  // Keep orientation within 0, 2pi
  if (myself.my_position[2] > PI) myself.my_position[2] -= 2.0 * PI;
  if (myself.my_position[2] < -PI) myself.my_position[2] += 2.0 * PI;
}

// Compute random numbers
// normal random variate generator
float box_muller(float m, float s)
{
  // mean m, standard deviation s
	float x1, x2, w, y1;
	static float y2;
	static int use_last = 0;

  // use value from previous call
	if (use_last)
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * (double)rand() / (double)RAND_MAX - 1.0;
			x2 = 2.0 * (double)rand() / (double)RAND_MAX - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( m + y1 * s );
}

////////////////////* Computes wheel speed given a certain X,Z speed *//////////////
void compute_wheel_speeds(int *msl, int *msr, float force_x, float force_z) {
  // Speed X in robot coordinates from global coordinates
  float x = myself.speed[myself.ID][0] * cosf(myself.my_position[2]) + myself.speed[myself.ID][1] * sinf(myself.my_position[2]);
  // Speed Z in robot coordinates from global coordinates
  float z = -myself.speed[myself.ID][0] * sinf(myself.my_position[2]) + myself.speed[myself.ID][1] * cosf(myself.my_position[2]);

  float val_x = 0;
  float val_z = 0;

  // Adapt forces
  if(force_x != 0 || force_z != 0 ){
    val_x = REYNOLDS_WEIGHT*x - OBSTACLE_WEIGHT*force_x;
    val_z = REYNOLDS_WEIGHT*z - OBSTACLE_WEIGHT*force_z;
  } else {
   val_x = REYNOLDS_WEIGHT_NO_OBSTACLE*x;
   val_z = REYNOLDS_WEIGHT_NO_OBSTACLE*z;
  }

  x = val_x;
  z = val_z;

  float range = sqrtf(x * x + z * z);     // Norm of the wanted speed vector in robot coordinate
  float bearing = -atan2f(x, z);

  float u = K_U * range * cosf(bearing);  // Compute forward control
  float w = K_W * bearing;                // Compute rotational control

  // Convert to wheel speeds (number of steps for each wheels)!
  *msl = (u - AXE_LENGTH * w / 2.0) * (50.0 / WHEEL_RADIUS);
  *msr = (u + AXE_LENGTH * w / 2.0) * (50.0 / WHEEL_RADIUS);

  limit(msl, MAX_SPEED);
  limit(msr, MAX_SPEED);
}

///////////////*Check if other robots are out of range *//////////////////
// If the last message is older than TIMEOUT then the robot is out of range
void check_if_out_of_range(){
  int i;
  for(i=0;i<FLOCK_SIZE;i++){
    if(i==myself.ID)
      continue;
    else{
      if((myTime.timer_count_100ms - is_in_flock[i][0]) >= TIMEOUT){
        is_in_flock[i][1] = 0;
      }
      else{
        is_in_flock[i][1] = 1;
      }
    }
  }
}

///////////////*Update speed according to Reynold's rules *//////////////////

void reynolds_rules() {
  int i, j;                         // Loop counters
  float rel_avg_loc[2] = {0, 0};    // Flock average positions
  float rel_avg_speed[2] = {0, 0};  // Flock average speeds
  float cohesion[2] = {0, 0};       // Cohesion factor
  float dispersion[2] = {0, 0};     // Dispersion factor
  float consistency[2] = {0, 0};    // Consistency factor
  float dist = 0;                   // Distance from other robots
  int nb_neighbours = 0;            // Number of robots in range

  /* Compute averages over the whole flock */
  for (i = 0; i < FLOCK_SIZE; i++) {
    // Considered only if other robot is in my group
    if (i != myself.ID && myself.flock_ID[i] == myself.flock_ID[myself.ID])
    {
      dist = sqrtf(myself.distances[i][0] * myself.distances[i][0] +
                   myself.distances[i][1] * myself.distances[i][1]);

      if (dist < MARGINAL_THRESHOLD) {
        for (j = 0; j < 2; j++) {

          // Is consider only if is_in_flock[i][1] == 1
          rel_avg_speed[j] += is_in_flock[i][1]*myself.speed[i][j];
          rel_avg_loc[j] += is_in_flock[i][1]*myself.distances[i][j];

          // Increment the number of robots around
          if(is_in_flock[i][1] == 1)
            nb_neighbours++;
        }
      }
    }
  }

  // Compute the mean
  for (j = 0; j < 2; j++) {
    if (nb_neighbours != 0) {
      rel_avg_loc[j] /= (nb_neighbours + 1);
      rel_avg_speed[j] /= (nb_neighbours);
    }
  }

  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  // If center of mass is too far
  for (j = 0; j < 2; j++) {
    if (fabs(rel_avg_loc[j]) > RULE1_THRESHOLD) {
      cohesion[j] = rel_avg_loc[j];
    }
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (i != myself.ID && myself.flock_ID[i] == myself.flock_ID[myself.ID]) {
      if (sqrtf(myself.distances[i][0] * myself.distances[i][0] +
                myself.distances[i][1] * myself.distances[i][1]) <
          RULE2_THRESHOLD) {
        for (j = 0; j < 2; j++) {
          // Is consider only if is_in_flock[i][1] == 1
          if (myself.distances[i][j] != 0)
            dispersion[j] -= is_in_flock[i][1]*myself.distances[i][j];
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
    myself.speed[myself.ID][j] += myself.migr[j] * MIGRATION_WEIGHT;//- myself.my_position[j]) * MIGRATION_WEIGHT;
  }
  myself.speed[myself.ID][1] *= -1;  // Inverted as we use a coordinate system like in Webots
}

/////////////////////* Envoie du message *///////////////////////////////////

void send_ping(void)
{
    ircomSend(myself.ID);           // Send my ID
    while (ircomSendDone() == 0);
}

////////////////////* Processs receive message *////////////////////////////////

void process_received_ping_messages(void)
{
  float message_direction;    // Angle of the emitter
  float distance;             // Received Signal Strength indicator
  int emitter_id;             // ID of the emmiter
  float deltaT;               // Time between two messages

  IrcomMessage imsg;
  ircomPopMessage(&imsg);     // Get the incoming message

  // While there is a message
  while(imsg.error != -1)
  {
    // Get message direction
    message_direction = (double)imsg.direction;

    // Keep the angle between -PI and PI
    if(message_direction > PI/2)
      message_direction = (message_direction + PI/2.0) - 2.0*PI;
    else
      message_direction = PI/2 + message_direction;

    // Get the distance in cm
    distance = (double)imsg.distance/100.0;   // Distance in cm
    // Get the emitter ID
    emitter_id = (int)imsg.value;

    // Control if signal is corrupted or if it's not from my group
    if(emitter_id == myself.ID || emitter_id >= FLOCK_SIZE || myself.flock_ID[emitter_id] == myself.flock_ID[myself.ID]){
      ircomPopMessage(&imsg);
      continue;
    }

    // Time between two messages
    deltaT = (myTime.timer_count_100ms-is_in_flock[emitter_id][0])/10.0;
    // Save the time
    is_in_flock[emitter_id][0] = myTime.timer_count_100ms;

    // Compute the coordinate of the emitter
    myself.distances[emitter_id][2] = message_direction;
    myself.distances[emitter_id][2] += myself.my_position[2];  // find the relative theta;

    // If start, get the initial position from robot 0.
    if(myself.get_initial_position == 1) {
      myself.my_position[0] = 1.0 * distance*cos(myself.distances[emitter_id][2]);
      myself.my_position[1] = -1.0 * distance*sin(myself.distances[emitter_id][2]);
      myself.my_previous_position[0] = myself.my_position[0];   // relative x pos
      myself.my_previous_position[1] = myself.my_position[1];   // relative y pos
      myself.my_previous_position[2] = myself.my_position[2];   // Relative Angle
      myself.get_initial_position = 0;
    } else {
      myself.previousDistances[emitter_id][0] = myself.distances[emitter_id][0];
      myself.previousDistances[emitter_id][1] = myself.distances[emitter_id][1];
      myself.previousDistances[emitter_id][2] = myself.distances[emitter_id][2];
      myself.distances[emitter_id][0] = 1.0 * distance*cos(myself.distances[emitter_id][2]);    // relative x pos
      myself.distances[emitter_id][1] = -1.0 * distance*sin(myself.distances[emitter_id][2]);   // relative y pos
      myself.speed[emitter_id][0] = 1.0*(1/deltaT)*(myself.distances[emitter_id][0]-myself.previousDistances[emitter_id][0]); // Relative speed X
      myself.speed[emitter_id][1] = 1.0*(1/deltaT)*(myself.distances[emitter_id][1]-myself.previousDistances[emitter_id][1]); // Relative speed Z
    }
    // Get next message
    ircomPopMessage(&imsg);
  }
}
