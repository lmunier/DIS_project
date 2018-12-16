// Supervisor of the project
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

// Variables to print some values if uncommented
#define VERBOSE_METRICS
//#define VERBOSE_POSITIONS

// Groups of robots
#define NB_GROUPS 2

#define GOOD_BOYS 5
#define BAD_BOYS 0

#define FLOCK_SIZE	10		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0
#define MAX_SPEED         0.1287     	// Maximum speed [m/s]
#define MAX_SPEED_WEB         6.28     	// Maximum speed [rad/s]

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float first_loc[FLOCK_SIZE][3] = {0};		// First location of everybody in the flock
float position[FLOCK_SIZE][3] = {0};		// Location of everybody in the flock
float loc[FLOCK_SIZE][3] = {0};		// Location of everybody in the flock
float relative_loc[FLOCK_SIZE][3] = {0};          // Relative location of everybody in the flock

float fit_c = 0.0, fit_o = 0.0, fit_s = 0.0;
int offset;			// Offset of robots number
float migr[2] = {0, -5};	           // Migration vector
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
    
        // Print positions
        #ifdef VERBOSE_POSITIONS
          printf("ID: %d, time %d position_X: %f, position_Z: %f\n",i,t,position[i][1],-position[i][0]);
          printf("ID: %d, time %d relative_X: %f, relative_Z: %f, relative_Theta: %f\n",i,t,relative_loc[i][1],-relative_loc[i][0],relative_loc[i][2]-M_PI/2);
        #endif
    }
}

/*
 * Compute performance metric.
 */
void compute_fitness(float *p_t, float *p_mean, int offset, float *old_mean_x, float *old_mean_z) {
    float mean_x = 0.0, mean_z = 0.0;
    float dist = 0.0, max_a = 0.0;
    float im_o = 0.0, re_o = 0.0;

    fit_c = 0.0;
    fit_o = 0.0;
    fit_s = 0.0;
    
    // Compute center of mass of the flock (mean of position x and z)
    for(int i=0; i<FLOCK_SIZE/NB_GROUPS; i++){
        mean_x += loc[i+offset][0];
        mean_z += loc[i+offset][1];
    }
    
    mean_x /= (FLOCK_SIZE/NB_GROUPS);
    mean_z /= (FLOCK_SIZE/NB_GROUPS);
    
    for(int i=0; i<FLOCK_SIZE/NB_GROUPS; i++){
        // Compute sum of real and imaginary number for the orintation metric
        re_o += cosf(loc[i+offset][2]);
        im_o += sinf(loc[i+offset][2]);
        
        // Compute sum of distance for the cohesion metric
        dist += sqrtf((loc[i+offset][0] - mean_x)*(loc[i+offset][0] - mean_x) + (loc[i+offset][1] - mean_z)*(loc[i+offset][1] - mean_z));
    }
    
    // Orientation between robots
    fit_o = sqrtf(re_o*re_o + im_o*im_o)/(FLOCK_SIZE/NB_GROUPS);
    
    // Distance between robots    
    fit_c = 1/((dist/(FLOCK_SIZE/NB_GROUPS)) + 1);
    
    if(t > 0){
        // Velocity of the team towards the goal direction
        max_a = 1000.0*((mean_x - *old_mean_x)*migr[1] - (mean_z - *old_mean_z)*migr[0]);
        max_a /= sqrtf(migr[0]*migr[0] + migr[1]*migr[1])*MAX_SPEED*TIME_STEP;
        
        if(offset == 0)
        fit_s = (max_a < 0) ? 0 : max_a;
        else
        fit_s = (-max_a < 0) ? 0 : -max_a;
    }

    #ifdef VERBOSE_METRICS
      if(offset == 0)
        printf("BAD_BOYS: fit_o = %f fit_c = %f fit_s = %f\n", fit_o, fit_c, fit_s);
      else
        printf("GOOD_BOYS: fit_o = %f fit_c = %f fit_s = %f\n", fit_o, fit_c, fit_s);
    #endif
    
    *old_mean_x = mean_x;            
    *old_mean_z = mean_z;
    
    // Return performance metrics
    *p_t = fit_c*fit_o*fit_s;
    *p_mean = ((*p_mean)*(t/TIME_STEP) + (*p_t))/(t/TIME_STEP + 1);
}

/*
 * Function to write metrics in file
 */
void write_file(char *name, int time, float cohesion, float orientation,
                float velocity, float p_time, float p_mean, bool init_column) {
  char filename[70];
  FILE *fp;

  sprintf(filename, "../../../../results/crossing_simulation/%s.csv", name);

  if ((fp = fopen(filename, "a")) == NULL) {
    printf("Cannot open file.\n");
    exit(1);
  }

  if (!init_column)
    fprintf(fp, "time,cohesion,orientation,velocity,p_time,p_mean\n");

  fprintf(fp, "%d,%f,%f,%f,%f,%f\n", time, cohesion, orientation, velocity, p_time,p_mean);
  fclose(fp);
}

/*
 * Main function.
 */
int main(int argc, char *args[]) {
    // Initialize reference fitness values
    static float p_t_goodboys = 0.0, p_mean_goodboys = 0.0;
    static float p_t_badboys = 0.0, p_mean_badboys = 0.0;
    static float old_mean_x_goodboys = 0.0, old_mean_z_goodboys = 0.0;
    static float old_mean_x_badboys = 0.0, old_mean_z_badboys = 0.0;
    bool init_write = false;
    
    reset();
    		
    for(;;) {
        // Compute and print the relative and globale positions
        compute_positions();
        
        // Compute and normalize fitness values
        compute_fitness(&p_t_badboys, &p_mean_badboys, BAD_BOYS, &old_mean_x_badboys, &old_mean_z_badboys);
        write_file("badboys", t, fit_c, fit_o, fit_s, p_t_badboys, p_mean_badboys, init_write);

        #ifdef VERBOSE_METRICS
            printf("BAD_BOYS: p_t = %f \t p_mean = %f\n\n", p_t_badboys, p_mean_badboys);
        #endif
        
        compute_fitness(&p_t_goodboys, &p_mean_goodboys, GOOD_BOYS, &old_mean_x_goodboys, &old_mean_z_goodboys);
        write_file("goodboys", t, fit_c, fit_o, fit_s, p_t_goodboys, p_mean_goodboys, init_write);

        if(!init_write)
            init_write = true;

        #ifdef VERBOSE_METRICS
            printf("GOOD_BOYS: p_t = %f \t p_mean = %f\n\n", p_t_goodboys, p_mean_goodboys);
        #endif
    		
        t += TIME_STEP;
        wb_robot_step(TIME_STEP);
    }

}
