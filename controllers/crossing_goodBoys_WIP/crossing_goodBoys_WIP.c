/*****************************************************************************/
/* File:         obstacle_WIPc */
/* Version:      1.1                                                         */
/* Date:         12.dec.2018 */
/* Description:  Reynolds flocking with relative positions	           */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define ABS(x) ((x>=0)?(x):-(x))

#define FLOCK_SIZE		5		// Number of robots
#define TIME_STEP 		64		// Step duration time
#define DELTA_T			0.064	         // Timestep (seconds)

#define AXE_LENGTH 		0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 		0.0205		// Wheel radius (meters)
#define PI			3.1415

#define NB_SENSORS		8		// Number of IR sensors
#define MIN_SENS          	300     	// Minimum sensibility value
#define MAX_SENS          	4096    	// Maximum sensibility value
#define MAX_SPEED         	800     	// Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB 		6.28 		// Maximum speed webots
/*Webots 2018b*/

#define RULE1_THRESHOLD     	0.20   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT       	0.2	// Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD     	0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT       	(0.03/10)	// Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT        	(1.0/10)   	// Weight of consistency rule. default 1.0/10
#define MARGINAL_THRESHOLD       0.22        // Distance to take an e-puck into a group
#define MIGRATION_WEIGHT    	(0.03/10)   	// Wheight of attraction towards the common goal. default 0.01/10

/*Adding correction factor*/
#define K_X	1.0
#define K_Z	1.0
#define K_TH 1.0
#define K_U 0.2 // Forward control coefficient
#define K_W 1.0   // Rotational control coefficient
int t;
// Define the group of the e-puck were we are
#define BAD_BOYS // If we are not a bad boys, we are a good boys
bool bad_group;

WbDeviceTag left_motor; 			// Handler for left wheel of the robot
WbDeviceTag right_motor; 			// Handler for the right wheel of the robot

int e_puck_matrix[16] = {17,  29,  34,  10, 8, -38, -56, -76, -72, -58, -36, 8,  10, 36,  28,  18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];			// Handle for the infrared distance sensors
WbDeviceTag receiver2;				// Handle for the receiver node
WbDeviceTag emitter2;				// Handle for the emitter node

struct robot{
	int ID;						// ID of myself
	int init;					// Control if initialized, 0=not, 1=yes
	float distances[FLOCK_SIZE][2];		// Store the distances between myself and other robots
	float previousDistances[FLOCK_SIZE][2];	// Store the previous distances between myself and other robots
	float relAngle[FLOCK_SIZE];			// Store the relative angle of the other robots
	float speed[FLOCK_SIZE][2];			// Store the relative speed of the other robots and the speed of myself
	float my_position[3];				// Initial position of myself: X, Z and theta
	float my_previous_position[3];			// Initial position of myself: X, Z and theta
	float migr[2];					// Position for the migratory urge
	int send;	       		
}myself;

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
  #ifdef BAD_BOYS
    bad_group = true;
  #else
    bad_group = false;
  #endif

  char* robot_name;
  wb_robot_init();

  receiver2 = wb_robot_get_device("receiver2");
  emitter2 = wb_robot_get_device("emitter2");

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
    // the device name is specified in the world file
    ds[i] = wb_robot_get_device(s);
    s[2]++;  // increases the device number
  }
  robot_name = (char *)wb_robot_get_name();

  for (i = 0; i < NB_SENSORS; i++)
    wb_distance_sensor_enable(ds[i], 64);

  wb_receiver_enable(receiver2, 64);

  // Reading the robot's name. Pay attention to name specification when adding
  // robots to the simulation!

  // read robot id from the robot's name
  sscanf(robot_name, "epuck%d", &(myself.ID));
  myself.ID %= FLOCK_SIZE;  // normalize between 0 and FLOCK_SIZE-1

  for(i=0;i<FLOCK_SIZE;i++) {
		myself.distances[i][0] = 0;            	// Initialize distance tab X
    myself.distances[i][1] = 0;            	// Initialize distance tab Z
    myself.previousDistances[i][0] = 0;   	// Initialize distance tab X
    myself.previousDistances[i][1] = 0;   	// Initialize distance tab Z
    myself.relAngle[i] = 0;                	// Initialize relative angle tab
    myself.speed[i][0] = 0;                	// Initialize speed tab X
    myself.speed[i][1] = 0;                	// Initialize speed tab Z
  }

  myself.my_position[0] = 0;			// Set initial X position of myself to 0
	myself.my_position[1] = 0;			// Set initial Z position of myself to 0
	myself.my_position[2] = 0;			// Set initial theta position of myself to 0
	myself.my_previous_position[0] = 0;			// Set previous X position of myself to 0
	myself.my_previous_position[1] = 0;			// Set previous Z position of myself to 0
	myself.my_previous_position[2] = 0;			// Set previous theta position of myself to 0

	if(myself.ID == 0){// || myself.ID == 1)	
		myself.send = 1;
    myself.init = 0;
	} else {
		myself.send = 0;
    myself.init = 1;
  }
        
  myself.migr[0] = 0;	 // Set the X migratory urge
  myself.migr[1] = 5; // Set the Z migratory urge
  
  printf("Reset: robot %d\n",myself.ID);
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
  // Compute deltas of the robot
  float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;	// Compute the translational displacement of the right wheel
  float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;	// Compute the translational displacement of the left wheel

  float du = (dr + dl) / 2.0;						// Compute the translational displacement of the epuck
  float dtheta = (dr - dl) / (2.0*AXE_LENGTH);					// Compute the angular displacement of the right wheel

  // Compute deltas in the environment
  float dx = -du * sinf(myself.my_position[2]);						// Compute the X translational displacement of the epuck
  float dz = -du * cosf(myself.my_position[2]);						// Compute the Z translational displacement of the epuck

  // Update position
  myself.my_position[0] += K_X  * dx;
  myself.my_position[1] += K_Z  * dz;
  myself.my_position[2] += K_TH * dtheta;

  // Keep orientation within 0, 2pi
  if (myself.my_position[2] > 2.0 * M_PI)
      myself.my_position[2] -= 2.0 * M_PI;
  if (myself.my_position[2] < 0)
      myself.my_position[2] += 2.0 * M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
  // Speed X in robot coordinates from global coordinates
  float x = myself.speed[myself.ID][0] * cosf(myself.my_position[2]) + myself.speed[myself.ID][1] * sinf(myself.my_position[2]);
  // Speed Z in robot coordinates from global coordinates
  float z = -myself.speed[myself.ID][0] * sinf(myself.my_position[2]) + myself.speed[myself.ID][1] * cosf(myself.my_position[2]);   
	
  // float range = sqrtf(x * x + z * z);    // Distance to the wanted position
  float range = sqrtf(x * x + z * z);    // Norm of the wanted speed vector in robot coordinate
  float bearing = -atan2f(x, z);          // Orientation of the wanted speed vector

  float u = K_U * range * cosf(bearing);  // Compute7 forward control
  float w = K_W * bearing;                // Compute rotational control

  // Convert to wheel speeds (number of steps for each wheels)!
  *msl = (u - AXE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
  *msr = (u + AXE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);

  limit(msl, MAX_SPEED);
  limit(msr, MAX_SPEED);
}

/*
 *  Update speed according to Reynold's rules
 */
void reynolds_rules() {
  int i, j;                         // Loop counters
  float rel_avg_loc[2] = {0, 0};    // Flock average positions
  float rel_avg_speed[2] = {0, 0};  // Flock average speeds
  float cohesion[2] = {0, 0};
  float dispersion[2] = {0, 0};
  float consistency[2] = {0, 0};
  float dist = 0;
  int nb_neighbours = 0;

  /* Compute averages over the whole flock */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (i != myself.ID) {
      dist = sqrtf(myself.distances[i][0] * myself.distances[i][0] + myself.distances[i][1] * myself.distances[i][1]);

      if (dist < MARGINAL_THRESHOLD) {
        for (j = 0; j < 2; j++) {
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
    if (i != myself.ID) {
      if (sqrtf(myself.distances[i][0]*myself.distances[i][0] + myself.distances[i][1]*myself.distances[i][1]) < RULE2_THRESHOLD) {
        for (j = 0; j < 2; j++) {
          if (myself.distances[i][j] != 0)
            dispersion[j] -= 1/myself.distances[i][j];
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
    myself.speed[myself.ID][j] += (myself.migr[j] - myself.my_position[j]) * MIGRATION_WEIGHT;
  }
  myself.speed[myself.ID][1] *= -1;  // z axis of webots is inverted
}

/*
 *  each robot sends a ping message, so the other robots can measure relative
 * range and bearing to the sender. the message contains the robot's name the
 * range and bearing will be measured directly out of message RSSI and direction
 */
void send_ping(void) {
  char out[10] = {'0'};
  myself.send = 0;
  sprintf(out, "%d", myself.ID);  // in the ping message we send the name of the robot.
  wb_emitter_send(emitter2, out, strlen(out) + 1);
}

/*
 * processing all the received ping messages, and calculate range and bearing to
 * the other robots the range and bearing are measured directly out of message
 * RSSI and direction
 */
void process_received_ping_messages(void) {
  const double *message_direction = NULL;
  double message_rssi = 0.0;  // Received Signal Strength indicator
  double range = 0.0;
  char *inbuffer;  // Buffer for the receiver node
  int emiter_id = 0, nextID = 0;

  while (wb_receiver_get_queue_length(receiver2) > 0) {
    inbuffer = (char *)wb_receiver_get_data(receiver2);
    message_direction = wb_receiver_get_emitter_direction(receiver2);
    message_rssi = wb_receiver_get_signal_strength(receiver2);

    double z = message_direction[2];
    double x = message_direction[1];

    // since the name of the sender is in the received message.
    // Note: this does not work for robots having id bigger than 9!
    emiter_id = (int)(inbuffer[0] - '0');
    nextID = (emiter_id+1)%FLOCK_SIZE;	

		if(nextID == myself.ID){
			myself.send = 1;
    }

    myself.relAngle[emiter_id] = -atan2f(z, x);
    myself.relAngle[emiter_id] += myself.my_position[2]; // Find the absolute theta
    range = sqrtf((1 / message_rssi));
    // printf("ID: %d, REC: %d, Y: %lf, X: %lf, ANGLe: %lf, OTHER: %lf\n
    // ",robot_id, other_robot_id, message_direction[2], message_direction[0],
    // theta, message_direction[1]);

    if (myself.init == 1) {
      myself.my_position[0] = range * cosf(myself.relAngle[emiter_id]);
      myself.my_position[1] = -1.0 * range * sinf(myself.relAngle[emiter_id]);
      myself.my_position[2] = 0;

      // printf("ID: %d, X: %lf, Z: %lf, Theta: %lf\n", robot_id,
      // my_position[0], my_position[1], my_position[2]);

      myself.my_previous_position[0] = myself.my_position[0];  // relative x pos
      myself.my_previous_position[1] = myself.my_position[1];  // relative y pos
      myself.my_previous_position[2] = myself.my_position[2];

      myself.init = 0;
    } else {
      // Get position update
      // theta += dtheta_g[other_robot_id];
      // theta_robots[other_robot_id] = 0.8*theta_robots[other_robot_id] +
      // 0.2*theta;
      myself.previousDistances[emiter_id][0] = myself.distances[emiter_id][0];
      myself.previousDistances[emiter_id][1] = myself.distances[emiter_id][1];

      myself.distances[emiter_id][0] = range * cosf(myself.relAngle[emiter_id]);  // relative x pos
      myself.distances[emiter_id][1] = -1.0 * range * sinf(myself.relAngle[emiter_id]);   // relative y pos

      // printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta
      // %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

      myself.speed[emiter_id][0] = 1.0 * (1 / DELTA_T) * (myself.distances[emiter_id][0] - myself.previousDistances[emiter_id][0]);
      myself.speed[emiter_id][1] = 1.0 * (1 / DELTA_T) * (myself.distances[emiter_id][1] - myself.previousDistances[emiter_id][1]);
    }

    /*if(myself.ID == 0){
			printf("Rec : %d, emm: %d \n", myself.ID, emiter_id);
			//printf("Directions: %lf, %lf\n", message_direction[0],message_direction[1]);
			//printf("Message rssi: %lf\n", message_rssi);
			//printf("Rel angle: %lf\n", myself.relAngle[emmiter_id]);
			//printf("Abs angle: %lf\n", abs_theta);
			//printf("Range: %lf\n", range);
			printf("Dist X: %lf\n", myself.my_position[0]);
			printf("Dist Z: %lf\n", myself.my_position[1]);
			//printf("Prev dist X: %lf\n", myself.previousDistances[emmiter_id][0]);
			//printf("Prev dist Z: %lf\n", myself.previousDistances[emmiter_id][1]);
			printf("Speed X: %lf\n", 1.0*(1.0/DELTA_T)*(myself.distances[emiter_id][0]-myself.previousDistances[emiter_id][0]));
			printf("Speed Z: %lf\n", 1.0*(1.0/DELTA_T)*(myself.distances[emiter_id][1]-myself.previousDistances[emiter_id][1]));
    }*/

    wb_receiver_next_packet(receiver2);
  }
}

// the main function
int main() {
  int msl = 0, msr = 0;  // Wheel speeds

  /*Webots 2018b*/
  float msl_w = 0.0, msr_w = 0.0;
  /*Webots 2018b*/

  int bmsl = 0, bmsr = 0, sum_sensors = 0;  // Braitenberg parameters
  int i = 0;                        // Loop counter
  int distances[NB_SENSORS] = {0};    // Array for the distance sensor readings
  int max_sens = 0;                 // Store highest sensor value

  reset();  // Resetting the robot

  // Forever
  while(true) {
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
    bmsl += 150;  // 66;
    bmsr += 150;  // 72;

    /* Send and get information */
    send_ping();  // sending a ping to other robot, so they can measure their
                  // distance to this robot

    /// Compute self position
    myself.my_previous_position[0] = myself.my_position[0];
    myself.my_previous_position[1] = myself.my_position[1];
    myself.my_previous_position[2] = myself.my_position[2];

    update_self_motion(msl, msr);

    process_received_ping_messages();

    myself.speed[myself.ID][0] = (1 / DELTA_T) * (myself.my_position[0] - myself.my_previous_position[0]);
    myself.speed[myself.ID][1] = (1 / DELTA_T) * (myself.my_position[1] - myself.my_previous_position[1]);

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
    msl += bmsl;
    msr += bmsr;

    limit(&msl, MAX_SPEED);
    limit(&msr, MAX_SPEED);

    // Print positions
    printf("ID: %d, time %d position_X: %f, position_Z:  %f\n",myself.ID,t,myself.my_position[0], myself.my_position[1]);
    printf("ID: %d, time %d relative_X: %f, relative_Z: %f, relative_Theta:  %f\n",(myself.ID+1)%FLOCK_SIZE,t,myself.distances[(myself.ID+1)%FLOCK_SIZE][0],myself.distances[(myself.ID+1)%FLOCK_SIZE][1],myself.relAngle[(myself.ID+1)%FLOCK_SIZE]);

    /*Webots 2018b*/
    // Set speed
    msl_w = (float)msl * MAX_SPEED_WEB / 1000;
    msr_w = (float)msr * MAX_SPEED_WEB / 1000;

    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    // wb_differential_wheels_set_speed(msl,msr);
    /*Webots 2018b*/

    // Continue one step
    wb_robot_step(TIME_STEP);
    t += TIME_STEP;
  }
}
