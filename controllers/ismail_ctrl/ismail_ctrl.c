/*****************************************************************************/
/* File:         obstacle_WIPc                                                 */
/* Version:      1.1                                                         */
/* Date:         12.dec.2018                                                   */
/* Description:  Reynolds flocking with relative positions	           */
/*****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/compass.h>

#define NB_SENSORS 8   // Number of distance sensors
#define MIN_SENS   250   // Minimum sensibility value
#define MAX_SENS   4096  // Maximum sensibility value
#define MAX_SPEED  800  // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB 6.28  // Maximum speed webots


#define MAGIC_FACTOR MAX_SPEED_WEB/MAX_SPEED

/*Webots 2018b*/
#define FLOCK_SIZE 10  // Size of flock
#define TIME_STEP 64   // [ms] Length of time step

#define AXLE_LENGTH     0.052    // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS    0.0205   // Wheel radius (meters)
#define DELTA_T         0.064    // Timestep (seconds)

#define RULE1_THRESHOLD  0.2  // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT     0.3  // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD  0.05 // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT     1.1  // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT     0.01  // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT 0.01  // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1  // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define MARGINAL_THRESHOLD 10

/*Adding correction factor*/
#define K_X	  1.0
#define K_Y	  1.0
#define K_TH	  1.0
#define K_U 	  0.2   // Forward control coefficient
#define K_W 	  10.0  // Rotational control coefficient
#define K_BR            1.2
#define K_C             0.8
/* Timestamp */
#define BUFF_SIZE    100
#define MAX_TIMEDIFF 1000

/*Webots 2018b*/
WbDeviceTag left_motor;   // handler for left wheel of the robot
WbDeviceTag right_motor;  // handler for the right wheel of the robot
/*Webots 2018b*/

// for obstacle avoidance
int e_puck_matrix[16] = {17,  29,  34,  10, 8, -38, -56, -76, -72, -58, -36, 8,  10, 36,  28,  18};
//int e_puck_matrix[c16] = {30,  40,  40,  15, 8, -38, -60, -85, -85, -60, -36, 8,  13, 36,  34,  30};

WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
WbDeviceTag receiver2;       // Handle for the receiver node
WbDeviceTag emitter2;        // Handle for the emitter node
WbDeviceTag compass;

// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_id_u, robot_id;

float relative_pos[FLOCK_SIZE][3];  // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
float relative_speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
float my_position[3];                 // X, Z, Theta of the current robot
float prev_my_position[3];   // X, Z, Theta of the current robot in the previous
                             // time step
float speed[FLOCK_SIZE][2];  // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE]; // != 0 if initial positions have been received
int irobot_time[FLOCK_SIZE]; // Internal robot time 
int irobot_time_max;         // Maximum  robot time
float migr[2] = {10, -50};      // Migration vector
char *robot_name;

float theta_robots[FLOCK_SIZE];
int init_pos = 0;


/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
  wb_robot_init();

  receiver2 = wb_robot_get_device("receiver2");
  emitter2 = wb_robot_get_device("emitter2");
  compass = wb_robot_get_device("compass");

  /*Webots 2018b*/
  // get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  /*Webots 2018b*/

  int i;
  char s[4] = "ps0";
  for (i = 0; i < NB_SENSORS; i++) {
    ds[i] = wb_robot_get_device(
        s);  // the device name is specified in the world file
    s[2]++;  // increases the device number
  }
  robot_name = (char *)wb_robot_get_name();

  for (i = 0; i < NB_SENSORS; i++) wb_distance_sensor_enable(ds[i], TIME_STEP);

  wb_receiver_enable(receiver2, TIME_STEP);
  wb_compass_enable(compass,TIME_STEP);
  // Reading the robot's name. Pay attention to name specification when adding
  // robots to the simulation!

  // read robot id from the robot's name
  sscanf(robot_name, "epuck%d", &robot_id_u);                 
  robot_id = robot_id_u % FLOCK_SIZE;  // normalize between 0 and FLOCK_SIZE-1

  for(i=0; i<3; i++) {
    my_position[i] = 0;
    prev_my_position[i] = 0;
    relative_pos[robot_id][i] = 0;
    prev_relative_pos[robot_id][i] = 0;
    
  }

  for (i = 0; i < FLOCK_SIZE; i++) {
    initialized[i] = 0;  // Set initialization to 0 (= not yet initialized)
    irobot_time[i] = 0; // Init all the times
  }

  for(i = 0; i < 2; i++)
  {
    speed[robot_id][i] = 0.0;
    relative_speed[robot_id][i] = 0.0;
  }

  if(robot_id != 0)
     init_pos = 1;
  irobot_time_max = 1;
  printf("Reset: robot %d\n", robot_id_u);
}


// Get angle of the compass
// Return angle in
double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}
/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
  if (*number > limit) *number = limit;
  if (*number < -limit) *number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {
  float theta = my_position[2];

  // Compute deltas of the robot
  float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  float du = (dr + dl) / 2.0;
  float dtheta = (dr - dl) / (AXLE_LENGTH);

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

  printf("id = %d, pos_x = %f ,pos_y = %f, pos_th = %f\n", robot_id,my_position[0],my_position[1],my_position[2]);

}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
  // x in robot coordinates
  float x = speed[robot_id][0] * cosf(my_position[2]) + speed[robot_id][1] * sinf(my_position[2]);
  // z in robot coordinates
  float z = -speed[robot_id][0] * sinf(my_position[2]) + speed[robot_id][1] * cosf(my_position[2]);

  
  float range = sqrtf(x * x + z * z);  // Distance to the wanted position
  float bearing = -atan2(x, z);        // Orientation of the wanted position

  float u = K_U * range * cosf(bearing);  // Compute7 forward control
  float w = K_W * bearing;                // Compute rotational control

  // Convert to wheel speeds!
  *msl = 50.0 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
  *msr = 50.0 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
  limit(msl, MAX_SPEED);
  limit(msr, MAX_SPEED);

}

/*
 *  Update speed according to Reynold's rules
 */
void reynolds_rules() {
  int i, j;                      // Loop counters
  float rel_avg_loc[2] = {0, 0};    // Flock average positions
  float rel_avg_speed[2] = {0, 0};  // Flock average speeds
  float cohesion[2] = {0, 0};
  float dispersion[2] = {0, 0};
  float consistency[2] = {0, 0};
  float dist = 0;
  int nb_neighbours = 0;

  /* Compute averages over the whole flock */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (i != robot_id) {
      dist = sqrtf(relative_pos[i][0]*relative_pos[i][0] + relative_pos[i][1]*relative_pos[i][1]);
      if (dist < MARGINAL_THRESHOLD) {
        if((irobot_time_max - irobot_time[i]) < MAX_TIMEDIFF)
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
  }

  // Calcul de la moyenne
  /*for (j = 0; j < 2; j++) {
    if (nb_neighbours != 0) {
      rel_avg_loc[j] /= (nb_neighbours + 1);
      rel_avg_speed[j] /= (nb_neighbours);
    }
  }*/
  
  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  //If center of mass is too far
  for(j=0; j < 2; j++){
    if(fabs(rel_avg_loc[j]) > RULE1_THRESHOLD) {
        cohesion[j] = rel_avg_loc[j];
    }
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (robot_id != i) {
      if(sqrt(pow(relative_pos[i][0],2)+pow(relative_pos[i][1],2)) < RULE2_THRESHOLD) {
        for (j=0;j<2;j++) 
        {
          if(relative_pos[i][j]!=0.0)
          {
            dispersion[j] -= relative_pos[i][j];
          }
        }
      }
    }
  }

  /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
  for (j = 0; j < 2; j++) {
    consistency[j] = rel_avg_speed[j];
  }

  printf("H. id = %d, avd = %f ,avl = %f \n", robot_id,rel_avg_speed[0],rel_avg_loc[0]);
  // aggregation of all behaviors with relative influence determined by weights
  for (j = 0; j < 2; j++) {
    speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
    speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
    speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;
    speed[robot_id][j] += (migr[j] - my_position[j]) * MIGRATION_WEIGHT;
  }
  speed[robot_id][1] *= -1;  // z axis of webots is inverted
  printf("H. id = %d, sp_x = %f ,sp_y = %f \n", robot_id,speed[robot_id][0],speed[robot_id][1]);
}

/*
 *  each robot sends a ping message, so the other robots can measure relative
 * range and bearing to the sender. the message contains the robot's name the
 * range and bearing will be measured directly out of message RSSI and direction
 */
void send_ping(void) {
  char out[10];
  strcpy(out, robot_name);  // in the ping message we send the name of the robot.
  wb_emitter_send(emitter2, out, strlen(out) + 1);
}

void send_robot_info(int p_robot_id) {
  char out_buf[BUFF_SIZE];

  if(p_robot_id != robot_id)
  {
    sprintf(out_buf,"%d:%d:%3.4f:%3.4f",p_robot_id,irobot_time[p_robot_id],relative_pos[p_robot_id][0],relative_pos[p_robot_id][1]); 
  }
  else
  {
    sprintf(out_buf,"%d:%d:%3.4f:%3.4f",p_robot_id,irobot_time[p_robot_id],(float)0,(float)0);
  }
  printf("--SEND INFO\n");
  printf(out_buf);
  printf("\n");
  printf("--END INFO\n");
  wb_emitter_send(emitter2, out_buf, strlen(out_buf) + 1);
}


/*
 * processing all the received ping messages, and calculate range and bearing to
 * the other robots the range and bearing are measured directly out of message
 * RSSI and direction
 */
void process_received_ping_messages(void) {
  const double *message_direction;
  double message_rssi;  // Received Signal Strength indicator
  double range;
  char *inbuffer;  // Buffer for the receiver node
  int other_robot_id;
  int rec_time;
  float rec_rel_pos[2] = {0.0,0.0};
  

  while (wb_receiver_get_queue_length(receiver2) > 0) {
    inbuffer = (char *)wb_receiver_get_data(receiver2);
    message_direction = wb_receiver_get_emitter_direction(receiver2);
    message_rssi = wb_receiver_get_signal_strength(receiver2);

    double z = message_direction[2];
    double x = message_direction[1];

    sscanf(inbuffer,"%d:%d:%f:%f",&other_robot_id,&rec_time,&rec_rel_pos[0],&rec_rel_pos[1]);

    relative_pos[other_robot_id][2] = -atan2(z, x);
    relative_pos[other_robot_id][2] += my_position[2];  // find the relative theta;
    range = sqrt((1 / message_rssi));
    //printf("ID: %d, REC: %d, Y: %lf, X: %lf, ANGLe: %lf, OTHER: %lf\n ",robot_id, other_robot_id, message_direction[2], message_direction[0], theta, message_direction[1]);

    if( irobot_time[other_robot_id] < rec_time)
    {
      if(irobot_time_max < rec_time)
      {
        irobot_time_max = rec_time;
      } 
      
      printf("--ID: %d, X: %lf, Z: %lf, Theta: %lf\n", robot_id, my_position[0], my_position[1], my_position[2]);
      if(init_pos == 1) 
      {
        my_position[0] = range*cos(relative_pos[other_robot_id][2]);
        my_position[1] = -1.0 * range*sin(relative_pos[other_robot_id][2]);
        my_position[2] = 0;

        prev_my_position[0] = my_position[0];   // relative x pos
        prev_my_position[1] = my_position[1];   // relative y pos
        prev_my_position[2] = my_position[2];
        init_pos = 0;
      } 
      else 
      {
        prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
        prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
        prev_relative_pos[other_robot_id][2] = relative_pos[other_robot_id][2];

        relative_pos[other_robot_id][0] = range*cos(relative_pos[other_robot_id][2]);          // relative x pos
        relative_pos[other_robot_id][1] = -1.0 * range*sin(relative_pos[other_robot_id][2]);   // relative y pos
        relative_pos[other_robot_id][0] += rec_rel_pos[0];          // relative x pos
        relative_pos[other_robot_id][1] += rec_rel_pos[1];   // relative y pos
        relative_speed[other_robot_id][0] = 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
        relative_speed[other_robot_id][1] = 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);
      }

      // Update the time and send info to other robots
      irobot_time[other_robot_id] = rec_time;
      send_robot_info(other_robot_id);
    }



    wb_receiver_next_packet(receiver2);
  }
}

// the main function
int main(int argc, const char *argv[]) {
  int msl, msr;  // Wheel speeds

  /*Webots 2018b*/
  float msl_w, msr_w;
  /*Webots 2018b*/

  int bmsl, bmsr, sum_sensors;  // Braitenberg parameters
  int i;                        // Loop counter
  int distances[NB_SENSORS];    // Array for the distance sensor readings
  int max_sens;                 // Store highest sensor value

  reset();  // Resetting the robot

  msl = 0;
  msr = 0;
  max_sens = 0;

  // Forever
  for (;;) {
    bmsl = 0;
    bmsr = 0;
    sum_sensors = 0;
    max_sens = 0;

    /* Braitenberg */
    for (i = 0; i < NB_SENSORS; i++) {
      distances[i] = wb_distance_sensor_get_value(ds[i]);  // Read sensor values
      sum_sensors += distances[i];  // Add up sensor values

      // Check if new highest sensor value
      max_sens = max_sens > distances[i] ? max_sens : distances[i];

      // Weighted sum of distance sensor values for Braitenburg vehicle
      bmsr += e_puck_matrix[i] * distances[i];
      bmsl += e_puck_matrix[i + NB_SENSORS] * distances[i];
    }

    // Adapt Braitenberg values (empirical tests)
    bmsl /= MIN_SENS;
    bmsr /= MIN_SENS;
    bmsl += 66;//66;
    bmsr += 72;//72;

     
    /* Send and get information */
    irobot_time[robot_id]++;
    send_robot_info(robot_id); 
    
    process_received_ping_messages();

    /// Compute self position
    prev_my_position[0] = my_position[0];
    prev_my_position[1] = my_position[1];
    prev_my_position[2] = my_position[2];
  
    update_self_motion(msl, msr);
    
    speed[robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
    speed[robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

    // Reynold's rules with all previous info (updates the speed[][] table)
    reynolds_rules();
    
    // Compute wheels speed from reynold's speed
    compute_wheel_speeds(&msl, &msr);

    // Adapt speed instinct to distance sensor values
    if (sum_sensors > NB_SENSORS * MIN_SENS) {
      msl -= msl * max_sens / (2 * MAX_SENS);
      msr -= msr * max_sens / (2 * MAX_SENS);
    }

    // Add Braitenberg
    msl = K_C * msl + K_BR * bmsl;
    msr = K_C * msr + K_BR * bmsr;
    printf("sl : %d and sr : %d\n",msl,msr);
    //limit(&msl, MAX_SPEED);
    //limit(&msr, MAX_SPEED);
    
    // Print positions
    //printf("ID: %d, time %d position_X: %f, position_Z: %f\n",robot_id,t,my_position[0], my_position[1]);
    //printf("ID: %d, time %d relative_X: %f, relative_Z: %f, relative_Theta: %f\n",(robot_id+1)%FLOCK_SIZE,t,relative_pos[(robot_id+1)%FLOCK_SIZE][0],relative_pos[(robot_id+1)%FLOCK_SIZE][1],relative_pos[(robot_id+1)%FLOCK_SIZE][2]);

    /*Webots 2018b*/
    // Set speed
    msl_w = (float) MAGIC_FACTOR * msl ;
    msr_w = (float) MAGIC_FACTOR * msr ;

    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    // wb_differential_wheels_set_speed(msl,msr);
    /*Webots 2018b*/

    // Continue one step
    wb_robot_step(TIME_STEP);
  }
}

