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
	#define TIME_STEP 		64		// Step duration time
	#define DELTA_T			0.064	         // Timestep (seconds)

	#define AXE_LENGTH 		0.052		// Distance between wheels of robot (meters)
	#define SPEED_UNIT_RADS 	0.00628 	// Conversion factor from speed unit to radian per second
	#define WHEEL_RADIUS 		0.0205		// Wheel radius (meters)
	#define PI			3.1415

	#define NB_SENSORS		8		// Number of IR sensors
	#define MIN_SENS          	350     	// Minimum sensibility value
	#define MAX_SENS          	4096    	// Maximum sensibility value
	#define MAX_SPEED         	800     	// Maximum speed
	#define MAX_SPEED_WEB 		6.28 		// Maximum speed webots

	#define RULE1_THRESHOLD     	0.20   		// Threshold to activate aggregation rule. default 0.20
	#define RULE1_WEIGHT       	(0.6/10)	// Weight of aggregation rule. default 0.6/10
	#define RULE2_THRESHOLD     	0.15   		// Threshold to activate dispersion rule. default 0.15
	#define RULE2_WEIGHT       	(0.02/10)	// Weight of dispersion rule. default 0.02/10
	#define RULE3_WEIGHT        	(1.0/10)   	// Weight of consistency rule. default 1.0/10
	#define MIGRATION_WEIGHT    	(0.01/10)   	// Wheight of attraction towards the common goal. default 0.01/10
	#define MIGRATORY_URGE 		1		// Tells the robots if they should just go forward or move towards a specific migratory direction

	WbDeviceTag left_motor; 			// Handler for left wheel of the robot
	WbDeviceTag right_motor; 			// Handler for the right wheel of the robot

	//RIGHT,LEFT
	int e_puck_matrix[16] = {10,30,35,40,10,-40,-60,-70,-70,-60,-40,10,40,35,30,10}; // for obstacle avoidance

	WbDeviceTag ds[NB_SENSORS];			// Handle for the infrared distance sensors
	WbDeviceTag receiver2;				// Handle for the receiver node
	WbDeviceTag emitter2;				// Handle for the emitter node

	struct robot{
		int ID;						// ID of myself
		int init;					// Control if initialized, 0=not, 1=yes
		double distances[FLOCK_SIZE][2];		// Store the distances between myself and other robots
		double previousDistances[FLOCK_SIZE][2];	// Store the previous distances between myself and other robots
		double relAngle[FLOCK_SIZE];			// Store the relative angle of the other robots
		double speed[FLOCK_SIZE][2];			// Store the relative speed of the other robots and the speed of myself
		double my_position[3];				// Initial position of myself: X, Z and theta
		double my_previous_position[3];			// Initial position of myself: X, Z and theta
		double migr[2];					// Position for the migratory urge
		int send;
		double absSpeed[FLOCK_SIZE][2];			       		
	}myself;

	static void reset(){
		
		char* robot_name;         
		 
		wb_robot_init();
		
		// Initialize communication
		receiver2 = wb_robot_get_device("receiver2");
		emitter2 = wb_robot_get_device("emitter2");
		
		// Get motors
		left_motor = wb_robot_get_device("left wheel motor");
		right_motor = wb_robot_get_device("right wheel motor");
		wb_motor_set_position(left_motor, INFINITY);
	    	wb_motor_set_position(right_motor, INFINITY);
		
		int i;
		char s[4]="ps0";
		for(i=0;i<NB_SENSORS;i++) {
			ds[i]=wb_robot_get_device(s);				// The device name is specified in the world file
			s[2]++;							// Increases the device number
		}

		robot_name=(char*) wb_robot_get_name(); 

		for(i=0;i<NB_SENSORS;i++)
		{
			wb_distance_sensor_enable(ds[i],64);
		}

		wb_receiver_enable(receiver2,64);

		// Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
		sscanf(robot_name,"epuck%d",&myself.ID); 			// Read robot id from the robot's name
		myself.ID = myself.ID%FLOCK_SIZE;	  			// Normalize between 0 and FLOCK_SIZE-1

		for(i=0;i<FLOCK_SIZE;i++) 
		{
			myself.distances[i][0] = 0;            	// Initialize distance tab X
		       	myself.distances[i][1] = 0;            	// Initialize distance tab Z
		       	myself.previousDistances[i][0] = 0;   	// Initialize distance tab X
		      	myself.previousDistances[i][1] = 0;   	// Initialize distance tab Z
		       	myself.relAngle[i] = 0;                	// Initialize relative angle tab
		       	myself.speed[i][0] = 0;                	// Initialize speed tab X
		      	myself.speed[i][1] = 0;                	// Initialize speed tab Z
			myself.absSpeed[i][0] = 0;
			myself.absSpeed[i][1] = 0;
		   	myself.init = 0;
		}

		myself.my_position[0] = 0;			// Set initial X position of myself to 0
		myself.my_position[1] = 0;			// Set initial Z position of myself to 0
		myself.my_position[2] = 0;			// Set initial theta position of myself to 0
		myself.my_previous_position[0] = 0;			// Set previous X position of myself to 0
		myself.my_previous_position[1] = 0;			// Set previous Z position of myself to 0
		myself.my_previous_position[2] = 0;			// Set previous theta position of myself to 0
		if(myself.ID == 0)// || myself.ID == 1)	
			myself.send = 1;
		else
			myself.send = 0;
	  
		//printf("Reset: robot %d\n",myself.ID);
		
		myself.migr[0] = -50;				// Set the X migratory urge
		myself.migr[1] = 25;				// Set the Y migratory urge
	}

	void limit(int *number, int limit)
	{
		if (*number > limit)
			*number = limit;
	    	if (*number < -limit)
			*number = -limit;
	}

	void update_self_motion(int msl, int msr)
	{
		float theta = myself.my_position[2];

	    	// Compute deltas of the robot
	    	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;	// Compute the translational displacement of the right wheel
	    	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;	// Compute the translational displacement of the left wheel
		
	    	float du = (dr + dl) / 2.0;						// Compute the translational displacement of the epuck
	    	float dtheta = (dr - dl) / (2.0*AXE_LENGTH);					// Compute the angular displacement of the right wheel

	    	// Compute deltas in the environment
	    	float dx = -du * sinf(theta);						// Compute the X translational displacement of the epuck
	    	float dz = -du * cosf(theta);						// Compute the Z translational displacement of the epuck

	    	// Update position
	    	myself.my_position[0] += dx;
	    	myself.my_position[1] += dz;
	    	myself.my_position[2] += dtheta;

	    	// Keep orientation within 0, 2pi
	    	if (myself.my_position[2] > 2 * M_PI)
	       		myself.my_position[2] -= 2.0 * M_PI;
	    	if (myself.my_position[2] < 0)
			myself.my_position[2] += 2.0 * M_PI;

		myself.absSpeed[myself.ID][0] = (1.0/DELTA_T)*(myself.my_position[0]-myself.my_previous_position[0]);
		myself.absSpeed[myself.ID][1] = (1.0/DELTA_T)*(myself.my_position[1]-myself.my_previous_position[1]);
	}

	void compute_wheel_speeds(int *msl, int *msr)
	{
	    	float x = myself.speed[myself.ID][0] * cosf(myself.my_position[2]) + myself.speed[myself.ID][1] * sinf(myself.my_position[2]);    // Speed X in robot coordinates from global coordinates
	    	float z = -myself.speed[myself.ID][0] * sinf(myself.my_position[2]) + myself.speed[myself.ID][1] * cosf(myself.my_position[2]);   // Speed Z in robot coordinates from global coordinates
		
	    	float Ku = 0.2;                        // Forward control coefficient
	    	float Kw = 1;                          // Rotational control coefficient
	    	// float range = sqrtf(x * x + z * z);    // Distance to the wanted position
		float range = sqrtf(x * x + z * z);    // Norm of the wanted speed vector in robot coordinate
	    	//float bearing = -atan2(x, z);          // Orientation of the wanted speed vector
		float bearing = -atan2(x, z);          // Orientation of the wanted speed vector

	    	// Compute forward control
	    	float u = Ku * range * cosf(bearing);
		// ADD PID CONTROL?
	    
		// Compute rotational control
	    	float w = Kw * bearing;
		// ADD PID CONTROL?

	    	// Convert to wheel speeds (number of steps for each wheels)!
	    	*msl = (u - AXE_LENGTH * w / 2.0) * (1000.0 / (WHEEL_RADIUS));		// *msl = (u - AXE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
	    	*msr = (u + AXE_LENGTH * w / 2.0) * (1000.0 / (WHEEL_RADIUS));		// *msl = (u + AXE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
		
	    	limit(msl, MAX_SPEED);
	   	limit(msr, MAX_SPEED);
	}

	void reynolds_rules()
	{
		int i, j, k;                     	// Loop counters
	    	float rel_avg_loc[2] = {0, 0};   	// Flock average positions
	    	float rel_avg_speed[2] = {0, 0}; 	// Flock average speeds
	    	float cohesion[2] = {0, 0};		// Cohesion factor
	    	float dispersion[2] = {0, 0};		// Dispersion factor
	    	float consistency[2] = {0, 0};		// Consistency factor

	    	 //Compute averages over the whole flock 
	    	for (i=0;i<FLOCK_SIZE;i++){

			if (i == myself.ID)
		    		continue;

			for (j = 0; j < 2; j++){
		   		rel_avg_speed[j] += myself.speed[i][j];
		    		rel_avg_loc[j] += myself.distances[i][j];
		 	}
	    	}
		
	    	for (j=0;j<2;j++){
			rel_avg_speed[j] /= FLOCK_SIZE - 1;
			rel_avg_loc[j] /= FLOCK_SIZE - 1;
	    	}
		
	    	 //Rule 1 - Aggregation/Cohesion: move towards the center of mass 
	    	for (j=0;j<2;j++){
			cohesion[j] = rel_avg_loc[j];
	    	}

	    	 //Rule 2 - Dispersion/Separation: keep far enough from flockmates 
	    	for (k=0;k<FLOCK_SIZE;k++){
			if (k != myself.ID){ // Loop on flockmates only
		    		// If neighbor k is too close (Euclidean distance)
		    		if ((myself.distances[k][0]*myself.distances[k][0] + myself.distances[k][1]*myself.distances[k][1]) < RULE2_THRESHOLD){
		        		for (j=0;j<2;j++){
		            			dispersion[j] -= 1/myself.distances[k][j];
		        		}
		    		}
			}
	    	}

	    	 //Rule 3 - Consistency/Alignment: match the speeds of flockmates 
	    	for (j=0;j<2;j++){
			consistency[j] = rel_avg_speed[j];
	    	}

	    	//aggregation of all behaviors with relative influence determined by weights
	    	for (j=0;j<2;j++){
			myself.speed[myself.ID][j] =   cohesion[j] 	* RULE1_WEIGHT;
			myself.speed[myself.ID][j] +=  dispersion[j] 	* RULE2_WEIGHT;
			myself.speed[myself.ID][j] +=  consistency[j] 	* RULE3_WEIGHT;
	    	}
	    	//myself.speed[myself.ID][1] *= -1; 		//y axis of webots is inverted

	    	//move the robot according to some migration rule
	    	if (MIGRATORY_URGE == 0){
			myself.speed[myself.ID][0] += 0.01 * cos(myself.my_position[2] + M_PI / 2);
			myself.speed[myself.ID][1] += 0.01 * sin(myself.my_position[2] + M_PI / 2);
	    	} else {
			myself.speed[myself.ID][0] += (myself.migr[0] - myself.my_position[0]) * MIGRATION_WEIGHT;
			myself.speed[myself.ID][1] -= (myself.migr[1] - myself.my_position[1]) * MIGRATION_WEIGHT; 		//y axis of webots is inverted
	    	}	
	}

	void initial_pos(void)
	{
		char *inbuffer;
	    	//int rob_nb;
	    	//float rob_x, rob_z, rob_theta; // Robot position and orientation
		int emmiter_id;
		const double *message_direction;							// Direction of the message
		double message_rssi; 									// Received Signal Strength indicator
		double range;										// Range of the message								

	    	while (myself.init == 0){
			// wait for message
			while (wb_receiver_get_queue_length(receiver2) == 0)
		    		wb_robot_step(TIME_STEP);

			inbuffer = (char *)wb_receiver_get_data(receiver2);
			
			emmiter_id = (inbuffer[0]-'0');
			
			if(emmiter_id != 0)
				continue;

			message_direction = wb_receiver_get_emitter_direction(receiver2);            	// Get direction : X & Z
			message_rssi = wb_receiver_get_signal_strength(receiver2);                  	// Get strength
			double z = message_direction[2];                                             	// Z direction
			double x = message_direction[1];                                             	// X direction

			myself.my_position[2] = -atan2(x,z);					// Find the relative theta
		        //abs_theta = myself.relAngle[emmiter_id] + myself.my_position[2]; 		// Find the absolute theta;
			range = sqrt((1/message_rssi));

			
			myself.my_position[0] = range*cos(myself.my_position[2]);             	            		// Compute relative X pos
			myself.my_position[1] = -1.0*range*sin(myself.my_position[2]);           	            		// Compute relative Z pos
			myself.my_previous_position[0] = myself.my_position[0];     		// Store previous X relative distances
			myself.my_previous_position[1] = myself.my_position[1];     		// Store previous Z relative distances

			myself.init = 1;		
			printf("X: %lf, Z: %lf, Theta: %lf\n",myself.my_position[0],myself.my_position[1],myself.my_position[2]);

			wb_receiver_next_packet(receiver2);
	    	}
	}

	void send_ping(void)  
	{
		char out[10];
		myself.send = 0;
		sprintf(out,"%d",myself.ID);				// In the ping message we send the name of the robot.
		wb_emitter_send(emitter2,out,strlen(out)+1); 		// Send Message
	}

	void process_received_ping_messages(void)
	{
		const double *message_direction;							// Direction of the message
		double message_rssi; 									// Received Signal Strength indicator
		double abs_theta;									// Absolute angle of the other robots
		//double rel_theta;									// Relative angle of the other robots
		double range;										// Range of the message
		char *inbuffer;										// Buffer for the receiver node
		int emmiter_id;										// ID of the emmiter robot
		int nextID;

		//printf("Queue length: %d\n",wb_receiver_get_queue_length(receiver2));
		
		while (wb_receiver_get_queue_length(receiver2) > 0) {

			// Getting the message. All data are in the cordinate system of the receiver.
		      	inbuffer = (char*) wb_receiver_get_data(receiver2);                          	// Get message
			message_direction = wb_receiver_get_emitter_direction(receiver2);            	// Get direction : X & Z
			message_rssi = wb_receiver_get_signal_strength(receiver2);                  	// Get strength
			double z = message_direction[2];                                             	// Z direction
			double x = message_direction[1];                                             	// X direction
			//ROUGE = X, BLEU = Z, VERT = Y
			
			// Identification of the sender
			emmiter_id = (inbuffer[0]-'0');                                    		// Get emmiter ID
			//printf("EMM: %d, REC: %d, Z: %lf, X: %lf\n",emmiter_id, myself.ID,z,x);
			//printf("EMM: %d, REC: %d, A: %lf, B: %lf, C: %lf\n",emmiter_id, myself.ID,message_direction[0],message_direction[1],message_direction[2]);
			nextID = (emmiter_id+1)%FLOCK_SIZE;	

			if(nextID == myself.ID){
				myself.send = 1;
			}
		        
		        myself.relAngle[emmiter_id] = -atan2(x,z);					// Find the relative theta
			//myself.relAngle[emmiter_id] = atan2(x,z);					// Find the relative theta
		        abs_theta = myself.relAngle[emmiter_id] + myself.my_position[2]; 		// Find the absolute theta;
			range = sqrt((1/message_rssi));							// Find the range of the message

			myself.previousDistances[emmiter_id][0] = myself.distances[emmiter_id][0];     		// Store previous X relative distances
			myself.previousDistances[emmiter_id][1] = myself.distances[emmiter_id][1];     		// Store previous Z relative distances
			myself.distances[emmiter_id][0] = range*cos(myself.relAngle[emmiter_id]);             	            		// Compute relative X pos
			myself.distances[emmiter_id][1] = -1.0 * range*sin(myself.relAngle[emmiter_id]);           	            		// Compute relative Z pos
			//myself.distances[emmiter_id][1] = range*cos(myself.relAngle[emmiter_id]);             	            		// Compute relative X pos
			//myself.distances[emmiter_id][0] = range*sin(myself.relAngle[emmiter_id]);           	            		// Compute relative Z pos

			//if(myself.ID == 0 && emmiter_id == 2)
				//printf("EMM: %d, REC: %d, X: %lf, Z: %lf, Theta: %lf\n",emmiter_id, myself.ID,myself.distances[emmiter_id][0],myself.distances[emmiter_id][1],myself.relAngle[emmiter_id]);

		        // Computation of the speeds
			myself.speed[emmiter_id][0] = 1.0*(1.0/DELTA_T)*(myself.distances[emmiter_id][0]-myself.previousDistances[emmiter_id][0]);  // Compute relative X speed
			myself.speed[emmiter_id][1] = 1.0*(1.0/DELTA_T)*(myself.distances[emmiter_id][1]-myself.previousDistances[emmiter_id][1]);  // Compute relative Z speed	

			// Computation of the absolute speeds
			myself.absSpeed[emmiter_id][0] = myself.absSpeed[myself.ID][0]+myself.speed[emmiter_id][0];
			myself.absSpeed[emmiter_id][1] = myself.absSpeed[myself.ID][1]+myself.speed[emmiter_id][1];

			//if(myself.ID == 0){
				//printf("Rec : %d, emm: %d \n", myself.ID, emmiter_id);
				//printf("Directions: %lf, %lf\n", message_direction[0],message_direction[1]);
				//printf("Message rssi: %lf\n", message_rssi);
				//printf("Rel angle: %lf\n", myself.relAngle[emmiter_id]);
				//printf("Abs angle: %lf\n", abs_theta);
				//printf("Range: %lf\n", range);
				//printf("Dist X: %lf\n", myself.my_position[0]);
				//printf("Dist Z: %lf\n", myself.my_position[1]);
				//printf("Prev dist X: %lf\n", myself.previousDistances[emmiter_id][0]);
				//printf("Prev dist Z: %lf\n", myself.previousDistances[emmiter_id][1]);
				//printf("Speed X: %lf\n", 1.0*(1.0/DELTA_T)*(myself.distances[emmiter_id][0]-myself.previousDistances[emmiter_id][0]));
				//printf("Speed Z: %lf\n", 1.0*(1.0/DELTA_T)*(myself.distances[emmiter_id][1]-myself.previousDistances[emmiter_id][1]));
			//}		

			wb_receiver_next_packet(receiver2);
		}
	}

	int main(){
		
		int msl = 0, msr = 0; 							// Wheel speeds [-800 800]

		reset();								// Reset the robot
		
		//wb_robot_step(TIME_STEP);

		//if(myself.send == 1){
			//myself.init = 1;
			//send_ping();
			//myself.send = 1;
		//}else
			//initial_pos();
			
		for(;;){
			
			if(myself.send == 1)
				send_ping();							// Send ping message

			// Compute self position
			myself.my_previous_position[0] = myself.my_position[0];
			myself.my_previous_position[1] = myself.my_position[1];
		
			update_self_motion(msl, msr);

			process_received_ping_messages();				// Process the received messages

			myself.speed[myself.ID][0] = (1 / DELTA_T) * (myself.my_position[0] - myself.my_previous_position[0]);		// Store myself X speed
			myself.speed[myself.ID][1] = (1 / DELTA_T) * (myself.my_position[1] - myself.my_previous_position[1]);		// Store myself Y speed

			// Reynold's rules with all previous info (updates the myself.speed[][] table)
			reynolds_rules();

			msl = myself.speed[myself.ID][0];
			msr = myself.speed[myself.ID][1];

			// Compute wheels speed from reynold's speed
			compute_wheel_speeds(&msl, &msr);

			msl = msl * MAX_SPEED_WEB / 1000;		// Conversion to webot speed, not needed for real situation MAX_SPEED_WEB = 1000 in real!
			msr = msr * MAX_SPEED_WEB / 1000;		// Conversion to webot speed, not needed for real situation!
			wb_motor_set_velocity(left_motor, msl);		// Apply speed command
			wb_motor_set_velocity(right_motor, msr);	// Apply speed command

			//printf("ID : %d, %lf, %lf %lf, ms: %d, %d\n",myself.ID, myself.my_position[0],myself.my_position[1],myself.my_position[2]*180.0/M_PI,msl,msr);

			wb_robot_step(TIME_STEP);					// Wait a time step
		}

	}
















/*
#define MAX_SPEED_WEB	6280	// Max speed in webot in mm/s
#define FLOCK_SIZE	5
#define WHEEL_RADIUS	20.5	// Radius in mm
#define MAX_SPEED_REAL	130	// Max real speed in mm/s
#define AXE_WHEEL	53	// Distance between the two wheels in mm
#define TIME_STEP	64	// Time step in ms
#define NB_SENSORS	8
#define DELTA_T		0.064	// Time step in s
#define SPEED_UNIT_RADS 	0.00628 

WbDeviceTag left_motor; 			
WbDeviceTag right_motor;
WbDeviceTag ds[NB_SENSORS];

struct robot{
	int ID;						// ID of myself
	int init;					// Control if initialized, 0=not, 1=yes
	double distances[FLOCK_SIZE][2];		// Store the distances between myself and other robots
	double previousDistances[FLOCK_SIZE][2];	// Store the previous distances between myself and other robots
	double relAngle[FLOCK_SIZE];			// Store the relative angle of the other robots
	double speed[FLOCK_SIZE][2];			// Store the relative speed of the other robots and the speed of myself
	double my_position[3];				// Initial position of myself: X, Z and theta
	double my_previous_position[3];			// Initial position of myself: X, Z and theta
	double migr[2];					// Position for the migratory urge
	int send;
	double absSpeed[FLOCK_SIZE][2];			       		
}myself;

void reset(){
	char* robot_name;         
		 
	wb_robot_init();
	
	// Initialize communication
	//receiver2 = wb_robot_get_device("receiver2");
	//emitter2 = wb_robot_get_device("emitter2");
	
	// Get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
    	wb_motor_set_position(right_motor, INFINITY);
	
	int i;
	char s[4]="ps0";
	for(i=0;i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);				// The device name is specified in the world file
		s[2]++;							// Increases the device number
	}

	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
	{
		wb_distance_sensor_enable(ds[i],64);
	}

	//wb_receiver_enable(receiver2,64);

	// Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&myself.ID); 			// Read robot id from the robot's name
	myself.ID = myself.ID%FLOCK_SIZE;	  			// Normalize between 0 and FLOCK_SIZE-1
}

// Input: speed in mm/s, Output: speed in rad/s
void convert_speed(int* msl, int* msr){
	*msl = *msl/WHEEL_RADIUS*100;	// 1000 factor here to not have 0 as the result is int and 0.something
	*msr = *msr/WHEEL_RADIUS*100;
}

void compute_wheel_speed(int* msl, int* msr){
	
}

void send_ping(){
}

void precess_received_ping_message(){
}

int main(){
	
	int msl=10, msr=10;	// Speed in mm/s
float dr;
float dl;
float dtheta;
double angle = 0;

	reset();
	
	myself.my_position[0] = 0;
	myself.my_position[1] = 0;
	myself.my_position[2] = 0;

	for(;;){

		msl = 10;
		msr = 5;		

		convert_speed(&msl, &msr);

		dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;	// Compute the translational displacement of the right wheel
	    	dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
		dtheta = (dr - dl) / (AXE_WHEEL);

		//angle+=(msl*2.0*DELTA_T*WHEEL_RADIUS)/(AXE_WHEEL*100.0);
		angle+=dtheta;

		printf("%lf\n",angle*180.0/M_PI);	
		
		printf("%d, %d\n",msl,msr);		
		
		wb_motor_set_velocity(left_motor, msl/100.0);		// Apply speed command
		wb_motor_set_velocity(right_motor, msr/100.0);

		wb_robot_step(TIME_STEP);
	}
}*/





