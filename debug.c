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

#define M_PI 3.14
#define NB_SENSORS 8   // Number of distance sensors
#define MIN_SENS 300   // Minimum sensibility value
#define MAX_SENS 4096  // Maximum sensibility value
#define MAX_SPEED 800  // Maximum speed
#define FLOCK_SIZE 5  // Size of flock
#define TIME_STEP 64  // [ms] Length of time step
#define TRUE 1        // true value
#define AXLE_LENGTH 0.052  // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205  // Wheel radius (meters)
#define DELTA_T 0.064        // Timestep (seconds)

#define RULE1_THRESHOLD  0.25  // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT (0.3 / 10)  // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD  0.1  // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT (0.5 / 10)  // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT (0.02 / 10)  // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT (0.03 / 10)  // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1  // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define MARGINAL_THRESHOLD 40

/*Adding correction factor*/
#define K_X	1.0
#define K_Y	1.0
#define K_TH	1.0
#define K_U 	0.2   // Forward control coefficient
#define K_W 	1  	  // Rotational control coefficient

// for obstacle avoidance
//int e_puck_matrix[16] = {17,  29,  34,  10, 8,  -38, -56, -76, -72, -58, -36, 8,  10, 36,  28,  18};
int e_puck_matrix[16] = {18,  28,  36,  10, 8,  -36, -58, -72, -72, -58, -36, 8,  10, 36,  28,  18};

// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_id;

float relative_pos[FLOCK_SIZE][3];  // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
float relative_speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
float my_position[3];                 // X, Z, Theta of the current robot
float prev_my_position[3];   // X, Z, Theta of the current robot in the previous
                             // time step
float speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
float initialized[FLOCK_SIZE];  // != 0 if initial positions have been received
float migr[2] = {0, 5};    // Migration vector
char *robot_name;
int send =0;

float theta_robots[FLOCK_SIZE];
int init_pos = 0;
int t;

/*int relative_pos[FLOCK_SIZE][3];  // relative X, Z, Theta of all robots
int prev_relative_pos[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
int relative_speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
int my_position[3];                 // X, Z, Theta of the current robot
int prev_my_position[3];   // X, Z, Theta of the current robot in the previous
                             // time step
int speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];  // != 0 if initial positions have been received
int migr[2] = {0, 5};    // Migration vector
char *robot_name;
int send =0;

int theta_robots[FLOCK_SIZE];
int init_pos = 0;
int t;*/

int getselector()
{
  return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void wait(unsigned long num)
{
	while (num > 0) {num--;}
}
//////////////////* Definition des fonctions *//////////////////////////////////

void reset(void);
void limit(int *, int);
void update_self_motion(int, int);
void compute_wheel_speeds(int *, int *);
void reynolds_rules(void);
void send_ping(void);
void process_received_ping_messages(void);

/////////////////////////* the main function    */////////////
int main()
{
  int msl, msr;  // Wheel speeds
  int bmsl, bmsr, sum_sensors;  // Braitenberg parameters
  int i;                    // Loop counter
  int distances[NB_SENSORS];    // Array for the distance sensor readings
  int max_sens;

  reset();

  ircomStart();
  ircomEnableContinuousListening();
  ircomListen();

  wait(100000);
    // rely on selector to define the role
  /* int selector = getselector();
   char tmp[128];
   sprintf(tmp, "selector=%d \n", selector);
   btcomSendString(tmp);
    // show selector choosen

    e_set_speed_left(500);
    e_set_speed_right(500);
    btcomSendString("COINCOIN\n");
    wait(100000);
    btcomSendString("ALLUMAGE\n");*/

  while(TRUE)
  {
    bmsl = 0; bmsr = 0; sum_sensors = 0; max_sens = 0;

    /* Braitenberg */
    for (i = 0; i < NB_SENSORS; i++)
    {
      distances[i] = e_get_calibrated_prox(i);
      sum_sensors += distances[i];  // Add up sensor values
      /* Test detection obstacle*/
        if(distances[i]>300)
        {
            e_set_led(i,1);
            char tmp[128];
            sprintf(tmp, "obstacle_intensite=%d \n", distances[i]);
            btcomSendString(tmp);
        }
        else
            e_set_led(i,0);
      // Check if new highest sensor value
      max_sens = max_sens > distances[i] ? max_sens : distances[i];

      // Weighted sum of distance sensor values for Braitenburg vehicle
      bmsr += e_puck_matrix[i] * distances[i];
      bmsl += e_puck_matrix[i + NB_SENSORS] * distances[i];
    }
      // Adapt Braitenberg values (empirical tests)
      bmsl /= MIN_SENS;
      bmsr /= MIN_SENS;
      bmsl += 200;//66;
      bmsr += 200;//72;

      /* Send and get information */
    if(send == 1)
      send_ping();  // sending a ping to other robot, so they can measure their
                      // distance to this robot-pedantic

    /// Compute self position-pedantic
    prev_my_position[0] = my_position[0];
    prev_my_position[1] = my_position[1];
    prev_my_position[2] = my_position[2];

    update_self_motion(msl, msr);

    process_received_ping_messages();

    speed[robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
    speed[robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

    // Reynold's rules with all previous info (updates the speed[][] table)
    reynolds_rules();

    // Compute wheels speed from reynold's speed
    compute_wheel_speeds(&msl, &msr);

    // Adapt speed instinct to distance sensor values
    if (sum_sensors > NB_SENSORS * MIN_SENS)
    {
      msl -= msl * max_sens / (2 * MAX_SENS);
      msr -= msr * max_sens / (2 * MAX_SENS);
    }

    // Add Braitenberg
    msl += bmsl;
    msr += bmsr;

    e_set_speed_left(msl);
    e_set_speed_right(msr);

    wait(100000);
  }

  return 0;

}
/////////////////////////Implementation des fonctions /////////////////////////

///////////////* Reset le robot  et recupere l'ID du robot *///////////////////

void reset(void)
{

    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();
    e_init_motors();
    e_start_agendas_processing();
    e_calibrate_ir();

  int i;
  robot_id = getselector();

  char reset[128];
  sprintf(reset, "reset_robot_id=%d \n", robot_id);
  btcomSendString(reset);

  for(i=0; i<3; i++)
  {
    my_position[i] = 0;
    prev_my_position[i] = 0;
  }

  for (i = 0; i < FLOCK_SIZE; i++)
  {
    initialized[i] = 0;  // Set initialization to 0 (= not yet initialized)
  }

  if(robot_id != 0)
     init_pos = 1;

//  btcomSendString("Reset: robot %d\n", robot_id);
  if(robot_id ==0)
    send =1;

}

/////////////*Keep given int number within interval {-limit, limit}*////////

void limit(int *number, int limit)
{
  if (*number > limit) *number = limit;
  if (*number < -limit) *number = -limit;
}

////////////* Updates robot position with wheel speeds *///////////////////////

void update_self_motion(int msl, int msr)
{
  float theta = my_position[2];

  // Compute deltas of the robot
  float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float du = (dr + dl) / 2.0;
  float dtheta = (dr - dl) / (2.0 * AXLE_LENGTH);

  // Compute deltas in the environment
  float dx = -du * sinf(theta);
  float dz = -du * cosf(theta);

  // Update position
  my_position[0] += K_X  * dx;
  my_position[1] += K_Y  * dz;
  my_position[2] += K_TH * dtheta;

  // Keep orientation within 0, 2pi
  if (my_position[2] > 2 * M_PI) my_position[2] -= 2.0 * M_PI;
  if (my_position[2] < 0) my_position[2] += 2.0 * M_PI;

  //printf("ID: %d , X : %f , Z: %f, TH : %f\n",robot_id,my_position[0],my_position[1],my_position[2]);
}
////////////////////* Computes wheel speed given a certain X,Z speed *//////////////

void compute_wheel_speeds(int *msl, int *msr)
{
  // x in robot coordinates
  float x = speed[robot_id][0] * cosf(my_position[2]) + speed[robot_id][1] * sinf(my_position[2]);
  // z in robot coordinates
  float z = -speed[robot_id][0] * sinf(my_position[2]) + speed[robot_id][1] * cosf(my_position[2]);

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
    if (i != robot_id)
    {
      dist = sqrtf(relative_pos[i][0] * relative_pos[i][0] +
                  relative_pos[i][1] * relative_pos[i][1]);
      if (dist < MARGINAL_THRESHOLD)
      {
        for (j = 0; j < 2; j++)
        {
          rel_avg_speed[j] += relative_speed[i][j];
          rel_avg_loc[j] += relative_pos[i][j];
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
    if (robot_id != i) {
      if(pow(relative_pos[i][0],2)+pow(relative_pos[i][1],2) < RULE2_THRESHOLD) {
        for (j=0;j<2;j++) {
            dispersion[j] -= relative_pos[i][j];
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
    speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
    speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
    speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;
  }
  speed[robot_id][1] *= -1;  // z axis of webots is inverted

  // move the robot according to some migration rule
  if (MIGRATORY_URGE == 0) {
    speed[robot_id][0] += 0.01 * cos(my_position[2] + M_PI / 2);
    speed[robot_id][1] += 0.01 * sin(my_position[2] + M_PI / 2);
  } else {
    speed[robot_id][0] += (migr[0] - my_position[0]) * MIGRATION_WEIGHT;
    speed[robot_id][1] -= (migr[1] - my_position[1]) * MIGRATION_WEIGHT;  // z axis of webots is inverted
  }
}

/////////////////////* Envoie du message *///////////////////////////////////

void send_ping(void)
{
  send = 0;
  ircomSend(robot_id);
  while (ircomSendDone() == 0);
}

////////////////////* Processs receive message *////////////////////////////////

void process_received_ping_messages(void)
{
  float message_direction;
  float message_rssi;  // Received Signal Strength indicator
  float range;
  int other_robot_id;
  int nextID;
  int emmiter_id;

 IrcomMessage imsg;
 ircomPopMessage(&imsg);

 while(imsg.error != -1)
 {
    message_direction = (float)imsg.direction;
    message_rssi = (float)imsg.distance;
    other_robot_id = (int)imsg.value;
    nextID = (emmiter_id+1)%FLOCK_SIZE;

		if(nextID == robot_id){
			send = 1;
		}

    relative_pos[other_robot_id][2] = message_direction;
    relative_pos[other_robot_id][2] += my_position[2];  // find the relative theta;
    range = message_rssi;

    if(init_pos == 1) {
      my_position[0] = range*cos(relative_pos[other_robot_id][2]);
      my_position[1] = -1.0 * range*sin(relative_pos[other_robot_id][2]);
      my_position[2] = 0;
      prev_my_position[0] = my_position[0];   // relative x pos
      prev_my_position[1] = my_position[1];   // relative y pos
      prev_my_position[2] = my_position[2];
      init_pos = 0;
    } else {
      prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
      prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
      prev_relative_pos[other_robot_id][2] = relative_pos[other_robot_id][2];
      relative_pos[other_robot_id][0] = range*cos(relative_pos[other_robot_id][2]);  // relative x pos
      relative_pos[other_robot_id][1] = -1.0 * range*sin(relative_pos[other_robot_id][2]);   // relative y pos
      relative_speed[other_robot_id][0] = 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
      relative_speed[other_robot_id][1] = 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);
    }
    ircomPopMessage(&imsg);
  }
}
