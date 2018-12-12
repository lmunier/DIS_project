// Supervisor of the project
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

//#define METRICS
#define FLOCK_SIZE	5		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0
#define MAX_SPEED         6.28     	// Maximum speed [m/s]

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float first_loc[FLOCK_SIZE][3] = {0};		// First location of everybody in the flock
float position[FLOCK_SIZE][3] = {0};		// Location of everybody in the flock
float loc[FLOCK_SIZE][3] = {0};		// Location of everybody in the flock
float relative_loc[FLOCK_SIZE][3] = {0};          // Relative location of everybody in the flock

int offset;			// Offset of robots number
float migr[2] = {0, 0};	           // Migration vector
float orient_migr; 			// Migration orientation
int t;
bool init = false;

/*
 * Initialize flock position and devices
 */
void reset(void) {
    wb_robot_init();
    char rob[7] = "epuck0";
    
    for (int i=0; i<FLOCK_SIZE; i++) {
        sprintf(rob,"epuck%d",i+offset);
        robs[i] = wb_supervisor_node_get_from_def(rob);
        robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
}

/*
 * Send initialize relative position
 */
void compute_positions(void) {
    if(!init){
        relative_loc[0][0] = 0;
        relative_loc[0][1] = 0;
        relative_loc[0][2] = 0;
    }
                
    for (int i=0; i<FLOCK_SIZE; i++) {
        // Get data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
        
        if(!init){
            first_loc[i][0] = loc[i][0];
            first_loc[i][1] = loc[i][1];
            first_loc[i][2] = loc[i][2];
            
            init = true;
        }
    
        if(i > 0){
            relative_loc[i][0] = loc[i][0] - loc[0][0];
            relative_loc[i][1] = loc[i][1] - loc[0][1];
            relative_loc[i][2] = loc[i][2] - loc[0][2];
        }
        
        position[i][0] = loc[i][0] - first_loc[i][0];
        position[i][1] = loc[i][1] - first_loc[i][1];
        position[i][2] = loc[i][2] - first_loc[i][2];
        
        // Keep orientation within 0, 2pi
        if (position[i][2] > 2 * M_PI)
            position[i][2] -= 2.0 * M_PI;
            
        if (position[i][2] < 0)
            position[i][2] += 2.0 * M_PI;
    
        
        printf("ID: %d, time %d position_X: %f, position_Z: %f\n",i,t,position[i][1],-position[i][0]);
        printf("ID: %d, time %d relative_X: %f, relative_Z: %f, relative_Theta: %f\n",i,t,relative_loc[i][1],-relative_loc[i][0],relative_loc[i][2]-M_PI/2);
    }
}

/*
 * Compute performance metric.
 */
void compute_fitness(float *p_t, float *p_mean) {
    float mean_x = 0.0, mean_z = 0.0;
    static float old_mean_x = 0.0, old_mean_z = 0.0;
    
    float dist = 0.0, max_a = 0.0;
    float fit_c = 0.0, fit_o = 0.0, fit_s = 0.0;
    float im_o = 0.0, re_o = 0.0;
    
    // Compute center of mass of the flock (mean of position x and z)
    for(int i=0; i<FLOCK_SIZE; i++){
        mean_x += loc[i][0];
        mean_z += loc[i][1];
    }
    
    mean_x /= FLOCK_SIZE;
    mean_z /= FLOCK_SIZE;
    
    for(int i=0; i<FLOCK_SIZE; i++){
        // Compute sum of real and imaginary number for the orintation metric
        re_o += cosf(loc[i][2]);
        im_o += sinf(loc[i][2]);
        
        // Compute sum of distance for the cohesion metric
        dist += sqrtf((loc[i][0] - mean_x)*(loc[i][0] - mean_x) + (loc[i][1] - mean_z)*(loc[i][1] - mean_z));
    }
    
    // Orientation between robots
    fit_o = sqrtf(re_o*re_o + im_o*im_o)/FLOCK_SIZE;
    
    #ifdef METRICS
        printf("fit_o= %f\n",fit_o);
    #endif
    
    // Distance between robots    
    fit_c = 1/((dist/FLOCK_SIZE) + 1);
    
    #ifdef METRICS
        printf("fit_c = %f\n",fit_c);
    #endif
    
    // Velocity of the team towards the goal direction
    max_a = cosf(orient_migr)*sqrtf((mean_x - old_mean_x)*(mean_x - old_mean_x) +
                                    (mean_z - old_mean_z)*(mean_z - old_mean_z))/MAX_SPEED;
                                    
    fit_s = (max_a < 0) ? 0 : max_a;
    
    #ifdef METRICS
        printf("fit_s = %f\n",fit_s);
    #endif
    
    old_mean_x = mean_x;            
    old_mean_z = mean_z;
    
    // Return performance metrics
    *p_t = fit_c*fit_o*fit_s;
    *p_mean = ((*p_mean)*(t/TIME_STEP) + (*p_t))/(t/TIME_STEP + 1);
}

/*
 * Main function.
 */
int main(int argc, char *args[]) {
    // Initialize reference fitness values
    static float p_t = 0.0, p_mean = 0.0;
    orient_migr = -atan2f(migr[1],migr[0]);
    
    reset();
    		
    for(;;) {
        // Compute and print the relative and globale positions
        compute_positions();
        
        // Compute and normalize fitness values
        compute_fitness(&p_t, &p_mean);
        
        #ifdef METRICS	
            printf("p_t = %f \t p_mean = %f\n", p_t, p_mean);
        #endif
    		
        t += TIME_STEP;
        wb_robot_step(TIME_STEP);
    }

}

