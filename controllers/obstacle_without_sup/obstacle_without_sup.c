/*****************************************************************************/
/* File:         obstacle.c                                                 */
/* Version:      2.0                                                         */
/* Date:         18-nov-2018                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	18-nov-2018 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
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

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			2*0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/
//RIGHT,LEFT
int e_puck_matrix[16] = {10,30,35,40,10,-40,-60,-70,-70,-60,-40,10,40,35,30,10}; // for obstacle avoidance


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,0};	        // Migration vector
char* robot_name;


float theta_robots[FLOCK_SIZE];

struct robot{
  int ID;
  int init;
  double distances[FLOCK_SIZE][2];
  double previousDistances[FLOCK_SIZE][2];
  double relAngle[FLOCK_SIZE];
  double speed[FLOCK_SIZE][2];
  //double absSpeed[FLOCK_SIZE][2];        A faire?
}myself;

/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	wb_robot_init();

	receiver2 = wb_robot_get_device("receiver2");
	emitter2 = wb_robot_get_device("emitter2");
	
	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
         right_motor = wb_robot_get_device("right wheel motor");
         wb_motor_set_position(left_motor, INFINITY);
         wb_motor_set_position(right_motor, INFINITY);
         /*Webots 2018b*/
	
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		printf("%d\n",ds[i]);
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
	{
          wb_distance_sensor_enable(ds[i],64);
	}

	wb_receiver_enable(receiver2,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
         
         myself.ID = robot_id;
         
	for(i=0; i<FLOCK_SIZE; i++) 
	{
                	myself.distances[i][0] = 0;            //Initialize distance tab x
                	myself.distances[i][1] = 0;            //Initialize distance tab y
                	myself.previousDistances[i][0] = 0;    //Initialize distance tab x
              	myself.previousDistances[i][1] = 0;    //Initialize distance tab y
                	myself.relAngle[i] = 0;                //Initialize relative angle tab
                	myself.speed[i][0] = 0;                //Initialize speed tab x
                	myself.speed[i][1] = 0;                //Initialize speed tab y
                	myself.init = 0;
		initialized[i] = 0;		   //Set initialization to 0 (= not yet initialized)
	}
  
        printf("Reset: robot %d\n",robot_id_u);
        
        migr[0] = 25;
        migr[1] = -25;
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]);  // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates
//	printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
	float Ku = 0.2;                       // Forward control coefficient
	float Kw = 1;                         // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
//	printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i=0;i<FLOCK_SIZE;i++){
          	if(i==myself.ID)
			continue;
			
		for (j=0;j<2;j++) {
          		rel_avg_speed[j] += myself.speed[i][j];
          		rel_avg_loc[j] += myself.distances[i][j];
          	}
	}
	
	for (j=0;j<2;j++) {
          	rel_avg_speed[j] /=FLOCK_SIZE-1;
          	rel_avg_loc[j] /=FLOCK_SIZE-1;
         }
         
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
         for (j=0;j<2;j++) 
	{	
          	cohesion[j] = rel_avg_loc[j];
	}
	
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != myself.ID){ // Loop on flockmates only
		// If neighbor k is too close (Euclidean distance)
		if(pow(myself.distances[k][0],2)+pow(myself.distances[k][1],2) < RULE2_THRESHOLD)
		{
			for (j=0;j<2;j++) {
    	   		dispersion[j] -= 1/myself.distances[k][j]; 
				}
			}
		}
	}		
  
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	for (j=0;j<2;j++) {
		consistency[j] = rel_avg_speed[j];
         }

         //aggregation of all behaviors with relative influence determined by weights
         for (j=0;j<2;j++) 
	{
                 speed[myself.ID][j] =   cohesion[j]    * RULE1_WEIGHT;
                 speed[myself.ID][j] +=  dispersion[j]  * RULE2_WEIGHT;
                 speed[myself.ID][j] +=  consistency[j] * RULE3_WEIGHT;
         }
        speed[myself.ID][1] *= -1; //y axis of webots is inverted
        
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[myself.ID][0] += 0.01*cos(my_position[2] + M_PI/2);
          speed[myself.ID][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
            speed[myself.ID][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
            speed[myself.ID][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted
        }
}


/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
        char out[10];
        //out[0] = myself.ID;
        
        if(myself.init){
          
        }else{
        
        }
        strcpy(out,robot_name);  // in the ping message we send the name of the robot.
        wb_emitter_send(emitter2,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/



void process_received_ping_messages(void)
{
        const double *message_direction;
        double message_rssi; // Received Signal Strength indicator
        double theta;
        double range;
        char *inbuffer;	// Buffer for the receiver node
        int emmiter_id;
        printf("Queue length: %d\n",wb_receiver_get_queue_length(receiver2));
        
        while (wb_receiver_get_queue_length(receiver2) > 0) {
                  
                  //Getting the message. All data are in the cordinate system of the receiver.
              	inbuffer = (char*) wb_receiver_get_data(receiver2);                          //Get message
		message_direction = wb_receiver_get_emitter_direction(receiver2);            //Get direction : X & Y
		message_rssi = wb_receiver_get_signal_strength(receiver2);                   //Get strength
		double y = message_direction[2];                                             //X direction
		double x = message_direction[1];                                             //Y direction


                  theta =	-atan2(y,x);
                  theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		
		
                  //Processing of the message
                  //Identification of the sender
		emmiter_id = (inbuffer[5]-'0');                                    //Get emmiter ID
		printf("Rec : %d, emm: %d \n", robot_id, emmiter_id);
		
		//Computation of the distances
		prev_relative_pos[emmiter_id][0] = relative_pos[emmiter_id][0];     //Store previous X relative distances
		prev_relative_pos[emmiter_id][1] = relative_pos[emmiter_id][1];     //Store previous Y relative distances
		relative_pos[emmiter_id][0] = range*cos(theta);                         //Compute relative X pos
		relative_pos[emmiter_id][1] = -1.0 * range*sin(theta);                  //Compute relative Y pos

                  //Computation of the speeds
		relative_speed[emmiter_id][0] = 1.0*(1/DELTA_T)*(relative_pos[emmiter_id][0]-prev_relative_pos[emmiter_id][0]);  //Compute relative X speed
		relative_speed[emmiter_id][1] = 1.0*(1/DELTA_T)*(relative_pos[emmiter_id][1]-prev_relative_pos[emmiter_id][1]);  //Compute relative Y speed	
		
		
		//Use of the structure
		myself.relAngle[emmiter_id] = -atan2(y,x);                                     //Store relative angle
		
		myself.previousDistances[emmiter_id][0] = myself.distances[emmiter_id][0];     //Store previous X relative distances
		myself.previousDistances[emmiter_id][1] = myself.distances[emmiter_id][1];     //Store previous Y relative distances
		
		myself.distances[emmiter_id][0] = relative_pos[emmiter_id][0];                 //Compute relative X pos
		myself.distances[emmiter_id][1] = relative_pos[emmiter_id][1];                 //Compute relative Y pos
		
		myself.speed[emmiter_id][0] = (1.0/DELTA_T)*(myself.distances[emmiter_id][0]-myself.previousDistances[emmiter_id][0]);
		myself.speed[emmiter_id][1] = (1.0/DELTA_T)*(myself.distances[emmiter_id][1]-myself.previousDistances[emmiter_id][1]);
		
		wb_receiver_next_packet(receiver2);
	}
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	         // Array for the distance sensor readings
	int max_sens;
	
 	reset();			         // Resetting the robot
 	
 	//Shifting the time of each robots
 	wb_robot_step(TIME_STEP+((robot_id*DELTA_T)/FLOCK_SIZE));
 	send_ping();
 	printf("Robot: %d\n", robot_id);
 	process_received_ping_messages();
 	
	msl = 0; msr = 0; 
	max_sens = 0; 
	
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
          		distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
                           sum_sensors += distances[i]; // Add up sensor values
                           max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

                            // Weighted sum of distance sensor values for Braitenburg vehicle
                            bmsr += e_puck_matrix[i] * distances[i];
                            bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
                 }

		 // Adapt Braitenberg values (empirical tests)
                 bmsl/=MIN_SENS; bmsr/=MIN_SENS;
                 bmsl+=200; bmsr+=200;
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
		
		update_self_motion(msl,msr);
		
		process_received_ping_messages();

		// Reynold's rules
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
                  
		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  
  
