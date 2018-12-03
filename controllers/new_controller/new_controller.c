#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define ABS(x) ((x>=0)?(x):-(x))

#define FLOCK_SIZE		5		// Number of robots
#define TIME_STEP 		2*0.64		// Step duration time
#define DELTA_T			2*0.064	         // Timestep (seconds)

#define NB_SENSORS		8		// Number of IR sensors
#define MIN_SENS          	350     	// Minimum sensibility value
#define MAX_SENS          	4096    	// Maximum sensibility value
#define MAX_SPEED         	800     	// Maximum speed

#define RULE1_THRESHOLD     	0.20   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT       	(0.6/10)	// Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD     	0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT       	(0.02/10)	// Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT        	(1.0/10)   	// Weight of consistency rule. default 1.0/10
#define MIGRATION_WEIGHT    	(0.01/10)   	// Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 		1		// Tells the robots if they should just go forward or move towards a specific migratory direction

WbDeviceTag left_motor; 			//handler for left wheel of the robot
WbDeviceTag right_motor; 			//handler for the right wheel of the robot

//RIGHT,LEFT
int e_puck_matrix[16] = {10,30,35,40,10,-40,-60,-70,-70,-60,-40,10,40,35,30,10}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];			// Handle for the infrared distance sensors
WbDeviceTag receiver2;				// Handle for the receiver node
WbDeviceTag emitter2;				// Handle for the emitter node

struct robot{
  int ID;					// ID of myself
  int init;					// Control if initialized, 0=not, 1=yes
  double distances[FLOCK_SIZE][2];		// Store the distances between myself and other robots
  double previousDistances[FLOCK_SIZE][2];	// Store the previous distances between myself and other robots
  double relAngle[FLOCK_SIZE];			// Store the relative angle of the other robots
  double speed[FLOCK_SIZE][2];			// Store the relative speed of the other robots
  //double absSpeed[FLOCK_SIZE][2];        A faire?
}myself;


static void reset(){

         char* robot_name;
         
         
	wb_robot_init();

	receiver2 = wb_robot_get_device("receiver2");
	emitter2 = wb_robot_get_device("emitter2");
	
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
        right_motor = wb_robot_get_device("right wheel motor");
        //wb_motor_set_position(left_motor, INFINITY);
        //wb_motor_set_position(right_motor, INFINITY);
        
        int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);				// the device name is specified in the world file
		printf("%d\n",ds[i]);
		s[2]++;							// increases the device number
	}

	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
	{
          wb_distance_sensor_enable(ds[i],64);
	}

	wb_receiver_enable(receiver2,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&myself.ID); 			// read robot id from the robot's name
	myself.ID = myself.ID%FLOCK_SIZE;	  			// normalize between 0 and FLOCK_SIZE-1

	for(i=0;i<FLOCK_SIZE;i++) 
	{
		myself.distances[i][0] = 0;            //Initialize distance tab X
               	myself.distances[i][1] = 0;            //Initialize distance tab Y
               	myself.previousDistances[i][0] = 0;    //Initialize distance tab X
              	myself.previousDistances[i][1] = 0;    //Initialize distance tab Y
               	myself.relAngle[i] = 0;                //Initialize relative angle tab
               	myself.speed[i][0] = 0;                //Initialize speed tab X
              	myself.speed[i][1] = 0;                //Initialize speed tab Y
           	myself.init = 0;
	}
  
        printf("Reset: robot %d\n",myself.ID);
        
        //migr[0] = 0;
        //migr[1] = 0;
}

void send_ping(void)  
{
        char out[10];
        sprintf(out,"%d",myself.ID);			// In the ping message we send the name of the robot.
        wb_emitter_send(emitter2,out,strlen(out)+1); 		// Send Message
}

void process_received_ping_messages(void)
{
        const double *message_direction;							// Direction of the message
        double message_rssi; 									// Received Signal Strength indicator
        double abs_theta;									// Absolute angle of the other robots
        double rel_theta;									// Relative angle of the other robots
        double range;										// Range of the message
        char *inbuffer;										// Buffer for the receiver node
        int emmiter_id;										// ID of the emmiter robot

        printf("Queue length: %d\n",wb_receiver_get_queue_length(receiver2));
        
        while (wb_receiver_get_queue_length(receiver2) > 0) {

		// Getting the message. All data are in the cordinate system of the receiver.
              	inbuffer = (char*) wb_receiver_get_data(receiver2);                          	// Get message
		message_direction = wb_receiver_get_emitter_direction(receiver2);            	// Get direction : X & Y
		message_rssi = wb_receiver_get_signal_strength(receiver2);                  	// Get strength
		double y = message_direction[2];                                             	// X direction
		double x = message_direction[1];                                             	// Y direction
		
		// Identification of the sender
		emmiter_id = (inbuffer[0]-'0');                                    		// Get emmiter ID
		printf("Rec : %d, emm: %d \n", myself.ID, emmiter_id);
                
                myself.relAngle[emmiter_id] = -atan2(y,x);							// Find the relative theta
                abs_theta = myself.relAngle[emmiter_id];// + my_position[2]; 						// Find the absolute theta;
		range = sqrt((1/message_rssi));							// Find the range of the message
		
                // Processing of the message
		// Computation of the distances
		myself.previousDistances[emmiter_id][0] = myself.distances[emmiter_id][0];     		// Store previous X relative distances
		myself.previousDistances[emmiter_id][1] = myself.distances[emmiter_id][1];     		// Store previous Y relative distances
		myself.distances[emmiter_id][0] = range*cos(abs_theta);             	            	// Compute relative X pos
		myself.distances[emmiter_id][1] = -1.0 * range*sin(abs_theta);      	            	// Compute relative Y pos

                // Computation of the speeds
		myself.speed[emmiter_id][0] = 1.0*(1/DELTA_T)*(myself.distances[emmiter_id][0]-myself.previousDistances[emmiter_id][0]);  // Compute relative X speed
		myself.speed[emmiter_id][1] = 1.0*(1/DELTA_T)*(myself.distances[emmiter_id][1]-myself.previousDistances[emmiter_id][1]);  // Compute relative Y speed	
		
		wb_receiver_next_packet(receiver2);
	}
}

int main(){

	reset();
	
	wb_robot_step(TIME_STEP+((myself.ID*DELTA_T)/FLOCK_SIZE));		// Shift in time the robots depending on their ID
	
	for(;;){
		
		send_ping();							// Send ping message
		process_received_ping_messages();				// Process the received messages

		wb_robot_step(TIME_STEP);					// Wait a time step
	}

}