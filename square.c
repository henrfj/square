/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
	double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('r'); i++)
		if (strcmp(tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('w'); i++)
		if (strcmp(tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 8000 //24902
#define MAXINT 65536
#define TIMETIC 0.01
#define P_GAIN_ANGLE 0.05
#define MAXLINE 128
#define MINLINE 0

typedef struct
{							 //input signals
	int left_enc, right_enc; // encoderticks
	// parameters
	double w;	   // wheel separation
	double cr, cl; // meters per encodertick
				   //output signals
	double right_pos, left_pos;
	// internal variables
	int left_enc_old, right_enc_old;
	// incremental displacement of wheels
	double dUL;
	double dUR;
	// position variables
	double x_pos, y_pos;
	// angle variable
	double theta;
	// Linesensor array
	int linesensor[8];
	// IR-sensor array
	//int linesensor[8];
	// total distance traveled
	double traveldist;
} odotype;

typedef struct
{ //input
	int cmd;
	int curcmd;
	double speedcmd;
	double dist;
	double angle;
	double left_pos, right_pos;
	// parameters
	double w;
	//output
	double motorspeed_l, motorspeed_r;
	int finished;
	// internal variables
	double startpos;
	// Current speed of the system
	double currentspeed;
} motiontype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
* Motion control
*/

enum
{
	mot_stop = 1,
	mot_move,
	mot_turn,
	mot_direction_control,
	mot_line_follow
	
};

void update_motcon(motiontype *p, odotype *o);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
int dc(double dist, double speed, double angle, int time);
int fol_dist(double dist,double speed, int time);


void linesensor_normalizer(int  linedata[8]);
float center_of_gravity(int linedata[8]);
int lowest_intensity(int linedata[8], char followleft);

typedef struct
{
	int state, oldstate;
	int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
	ms_init,
	ms_fwd,
	ms_turn,
	ms_end,
	ms_direction_control,
	ms_followline
};


void write_log(double log[MAXINT][7]) //here
{
    FILE * fPtr;
    fPtr = fopen("/home/smr/offline/square/log.dat", "w");
    if(fPtr == NULL)
    {
        printf("Unable to create file.\n"); // File not created hence exit
    }
    
    for(int i = 0; i < MAXINT; i++){
      fprintf(fPtr,"%f %f %f %f %f %f %f\n",
	   log[i][0],log[i][1],log[i][2], log[i][3], log[i][4], log[i][5], log[i][6]); // Write data to file
   }
    fclose(fPtr); // Close file to save file data
}


void write_laser_log(double laserpar[10]){
	//TODO clear the file before run.
    FILE * fPtr;
    fPtr = fopen("/home/smr/offline/square/laserpar.dat", "a"); //a for appending not overwriting
    if(fPtr == NULL)
    {
        printf("Unable to create file.\n"); // File not created hence exit
    }
    fprintf(fPtr,"%f %f %f %f %f %f %f %f %f %f\n",
	   laserpar[0], laserpar[1], laserpar[2], laserpar[3], laserpar[4], laserpar[5], laserpar[6],
	    laserpar[7], laserpar[8], laserpar[9]); // Write data to file
    fclose(fPtr); // Close file to save file data
}


int main(){
	int running, n = 0, arg, time = 0, m = 0;
	double dist = 0, angle = 0, speed = 0;
	double control_angle = 0;
	remove("/home/smr/offline/square/laserpar.dat"); //remove file for write_laser_log

	/* Establish connection to robot sensors and actuators.
   */
	if (rhdConnect('w', "localhost", ROBOTPORT) != 'w')
	{
		printf("Can't connect to rhd \n");
		exit(EXIT_FAILURE);
	}

	printf("connected to robot \n");
	if ((inputtable = getSymbolTable('r')) == NULL)
	{
		printf("Can't connect to rhd \n");
		exit(EXIT_FAILURE);
	}
	if ((outputtable = getSymbolTable('w')) == NULL)
	{
		printf("Can't connect to rhd \n");
		exit(EXIT_FAILURE);
	}
	// connect to robot I/O variables
	lenc = getinputref("encl", inputtable);
	renc = getinputref("encr", inputtable);
	linesensor = getinputref("linesensor", inputtable);
	irsensor = getinputref("irsensor", inputtable);

	speedl = getoutputref("speedl", outputtable);
	speedr = getoutputref("speedr", outputtable);
	resetmotorr = getoutputref("resetmotorr", outputtable);
	resetmotorl = getoutputref("resetmotorl", outputtable);
	// **************************************************
	//  Camera server code initialization
	// :)

	/* Create endpoint */
	lmssrv.port = 24919;
	strcpy(lmssrv.host, "127.0.0.1");
	strcpy(lmssrv.name, "laserserver");
	lmssrv.status = 1;
	camsrv.port = 24920;
	strcpy(camsrv.host, "127.0.0.1");
	camsrv.config = 1;
	strcpy(camsrv.name, "cameraserver");
	camsrv.status = 1;

	if (camsrv.config)
	{
		int errno = 0;
		camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (camsrv.sockfd < 0)
		{
			perror(strerror(errno));
			fprintf(stderr, " Can not make  socket\n");
			exit(errno);
		}

		serverconnect(&camsrv);

		xmldata = xml_in_init(4096, 32);
		printf(" camera server xml initialized \n");
	}

	// **************************************************
	//  LMS server code initialization
	//

	/* Create endpoint */
	lmssrv.config = 1;
	if (lmssrv.config)
	{
		char buf[256];
		int errno = 0, len;
		lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (lmssrv.sockfd < 0)
		{
			perror(strerror(errno));
			fprintf(stderr, " Can not make  socket\n");
			exit(errno);
		}

		serverconnect(&lmssrv);
		if (lmssrv.connected)
		{
			xmllaser = xml_in_init(4096, 32);
			printf(" laserserver xml initialized \n");
			//len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
			len=sprintf(buf,"scanpush cmd='zoneobst'\n");
			send(lmssrv.sockfd, buf, len, 0);
		}
	}

	/* Read sensors and zero our position.
   */
	rhdSync();

	odo.w = 0.256;
	odo.cr = DELTA_M;
	odo.cl = odo.cr;
	odo.left_enc = lenc->data[0];
	odo.right_enc = renc->data[0];
	// linesensor
	memcpy(odo.linesensor, linesensor->data, sizeof(odo.linesensor));
	reset_odo(&odo);

	mot.w = odo.w;
	mot.currentspeed = 0;
	running = 1;
	mission.state = ms_init;
	mission.oldstate = -1;

	/*
	* Assuming a 16 bit max integer size. With 100Hz => can save 11 minutes of log.
	* Each row holds three datapoints: Mission time, motor speed left, motor speed right
	* Later fix the overflow.
	*/

	double logg[MAXINT][7]; 
	int log_index = 0;

	clock_t start_time = clock();

	while (running){
		
		if (lmssrv.config && lmssrv.status && lmssrv.connected)
		{
			while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
				xml_proca(xmllaser);
		}

		if (camsrv.config && camsrv.status && camsrv.connected)
		{
			while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
				xml_proc(xmldata);
		}

		rhdSync(); // updates the HW sensor data

		odo.left_enc = lenc->data[0];
		odo.right_enc = renc->data[0];
		memcpy(odo.linesensor, linesensor->data, sizeof(odo.linesensor));
		
		/*
		linesensor_normalizer(odo.linesensor);
		int line_index;
		line_index = lowest_intensity(odo.linesensor);
		
		for (int i=0; i<8; i++){
			printf("%d\t", odo.linesensor[i]);
		}
		printf("=> %d\n", line_index);
		*/

		// the rest of the odo updates
		update_odo(&odo);

		/****************************************
		/ mission statemachine   
		****************************************/
		sm_update(&mission);
		switch (mission.state)
		{
		case ms_init:
			n = 1; //4
			m = 1;
			dist = 20;
			speed = 0.2; // 0.2 0.4 0.6
			// angle = 90.0 / 180 * M_PI; // CW
			//angle = -90.0 / 180 * M_PI; //CCW
			angle = 149 * M_PI/180;
			control_angle = 75 * M_PI/180;
			//mission.state = ms_direction_control;
			mission.state = ms_followline;
			break;

		case ms_fwd:
			//printf("ms_fwd!\n");
			if (fwd(dist, speed, mission.time))
				//mission.state = ms_turn;
				//mission.state = ms_direction_control;
				mission.state = ms_direction_control;
			break;

		case ms_followline:
			if (fol_dist(dist,speed, mission.time))
				//mission.state = ms_turn;
				//mission.state = ms_direction_control;
				mission.state = ms_end;
			break;


		case ms_direction_control:
			//printf("ms_direction_control!\n");
			if (dc(dist, speed, control_angle, mission.time)){
				m = m - 1;
				if (m == 0){
					mission.state = ms_end; 
				}else{
					mission.state = ms_fwd;
				}
			}
			break;

		case ms_turn:
			//printf("ms_turn!\n");
			if (turn(angle, speed, mission.time))
			{
				n = n - 1;
				if (n == 0)
					mission.state = ms_direction_control; //ms_end
				else
					mission.state = ms_fwd;
			}
			break;

		case ms_end:
			mot.cmd = mot_stop;
			running = 0;
			break;
		}
		/*  end of mission  */

		mot.left_pos = odo.left_pos;
		mot.right_pos = odo.right_pos;
		update_motcon(&mot, &odo); // motion controller state machine
		speedl->data[0] = 100 * mot.motorspeed_l;
		speedl->updated = 1;
		speedr->data[0] = 100 * mot.motorspeed_r;
		speedr->updated = 1;

		write_laser_log(laserpar);
		
		if (time % 100 == 0)
		//	   printf(" laser %f \n",laserpar[3]);
			time++;
		/* stop if keyboard is activated*/
		ioctl(0, FIONREAD, &arg);
		if (arg != 0)
			running = 0;


		// Do some logging
		if (log_index == MAXINT){
			printf("looped around\n");
			log_index = 0;
		}
		
		clock_t current_time = clock();
		double time_spent = (double)(current_time - start_time) / 5000; //TODO: use time() function instead

		logg[log_index][0] = mission.time; 		// no. tics for the current mission.
		logg[log_index][1] = mot.motorspeed_l;	// 
		logg[log_index][2] = mot.motorspeed_r;	//
		logg[log_index][3] = time_spent;		// "time spent" - using clock and a no. pulled out of our asses.
		logg[log_index][4] = odo.x_pos;			// x pos of robot
		logg[log_index][5] = odo.y_pos;			// y pos of robot
		logg[log_index][6] = odo.theta;			// absolute theta of robot
		log_index++;

	} /* end of main control loop */
	
	speedl->data[0] = 0;
	speedl->updated = 1;
	speedr->data[0] = 0;
	speedr->updated = 1;

	// write the log to a .dat file
	write_log(logg);
	//

	rhdSync();
	rhdDisconnect();
	exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p){
	p->right_pos = p->left_pos = 0.0;
	p->right_enc_old = p->right_enc;
	p->left_enc_old = p->left_enc;
	//
	p->x_pos = 0;
	p->y_pos = 0;
	p->theta = 0;
	//
	p->traveldist = 0;
}

void update_odo(odotype *p){
	
	// Don't delete it - we need it
	int delta;
	delta = p->right_enc - p->right_enc_old;
	if (delta > 0x8000)
		delta -= 0x10000;
	else if (delta < -0x8000)
		delta += 0x10000;
	p->right_enc_old = p->right_enc;
	p->right_pos += delta * p->cr;
	
	// 1 - The incremental displacement of each wheel 
	p->dUR = delta * p->cr;

	delta = p->left_enc - p->left_enc_old;
	if (delta > 0x8000)
		delta -= 0x10000;
	else if (delta < -0x8000)
		delta += 0x10000;
	p->left_enc_old = p->left_enc;
	p->left_pos += delta * p->cl;

	// 1 - The incremental displacement of each wheel 
	p->dUL = delta * p->cl;
	
	//printf("deltas: (%f) \n", p->dUL-p->dUR);

	// 2 - The incremental displacement of the robot center point delta U[i]
	float dU = (p->dUR + p->dUL)/2;
	p->traveldist += dU;
	//printf("traveldist: %f\n", p->traveldist);

	// 3 - The incremental change in orientation delta theta[i]
	float dtheta = (p->dUR - p->dUL) / WHEEL_SEPARATION;

	// 4 - New pose of the robot
	p->theta += dtheta;
	p->x_pos += dU * cos(p->theta);
	p->y_pos += dU * sin(p->theta);
	
	//printf("Position: (%f %f %f) \n", p->x_pos, p->y_pos, p->theta);

}
// MOTOR STATE MACHINE
void update_motcon(motiontype *p, odotype *o){
	//static double currentspeed = 0; // curren speed of the system
	double clock_acceleration;
	double max_acceleration = 0.5;
	double driven_dist;
	double d;
	static int deaccel_flag = 0;
	double current_angle = 0;
	double dV;

	if (p->cmd != 0)
	{ // initialize the motor commands
		p->finished = 0;
		switch (p->cmd)
		{
		case mot_stop:
			p->curcmd = mot_stop;
			break;
		case mot_move:

			p->startpos = (p->left_pos + p->right_pos) / 2;
			p->curcmd = mot_move;
			break;

		case mot_turn:

			if (p->angle > 0)
				p->startpos = p->right_pos;
			else
				p->startpos = p->left_pos;
			p->curcmd = mot_turn;
			break;

		case mot_direction_control:
			p->startpos = (p->left_pos + p->right_pos) / 2;
			p->curcmd = mot_direction_control;
			break;
		
		case mot_line_follow:
				o->traveldist = 0;
				p->curcmd = mot_line_follow;
			break;
		}
		p->cmd = 0;
	}

	switch (p->curcmd){
	case mot_stop:
		p->motorspeed_l = 0;
		p->motorspeed_r = 0;
		break;

	case mot_move:
		driven_dist = (p->right_pos + p->left_pos) / 2 - p->startpos; 
		d = p->dist - driven_dist; 						// remaining distance
		clock_acceleration = max_acceleration * TIMETIC; // Incremental speed/program run

		if ((deaccel_flag) || (p->currentspeed >= sqrt(2 * max_acceleration * d))){ /// We need to deaccelerate
			if (!deaccel_flag){
				printf("The flag has been hoisted!\n");
				deaccel_flag = 1;
			}
			
			if (fabs(0 - p->currentspeed) > clock_acceleration){
				p->currentspeed -= clock_acceleration;
			}else{
				p->currentspeed = 0;
				p->finished = 1;
				deaccel_flag = 0;
			}
			p->motorspeed_l = p->currentspeed;
			p->motorspeed_r = p->currentspeed;

		}else{ // Keep accelerating or moving forward
			if (fabs(p->speedcmd - p->currentspeed) > clock_acceleration){ // Accelerate
				p->currentspeed += clock_acceleration;
			}else{ // Max speed 
				p->currentspeed = p->speedcmd;
			}
			p->motorspeed_l = p->currentspeed;
			p->motorspeed_r = p->currentspeed;
		}
		printf("Current speed: %f\n", p->currentspeed);	
		break;

	case mot_turn: // HERE WE ALREADY KNOW OUR DESIRED ANGLE!
		// This will make a hard turn
		if (p->angle > 0) //Left turn

		{
			//p->motorspeed_l = 0;
			if (p->right_pos - p->startpos < (p->angle)/2 * p->w)
			{ // Havent reach desired angle yet.
				p->motorspeed_r = (p->speedcmd)/2;
				//
				p->motorspeed_l = -(p->speedcmd)/2;
				//
			}
			else
			{// Reach desired angle.
				p->motorspeed_r = 0;
				// ---------------------------
				p->motorspeed_l = 0;
				// ---------------------------
				p->finished = 1;
			}
		}
		else // doing a right turn
		{
			//p->motorspeed_r = 0;
			if (p->left_pos - p->startpos < fabs(p->angle)/2 * p->w)
			{ // Havent reach desired angle yet.
				p->motorspeed_l = (p->speedcmd)/2;
				//
				p->motorspeed_r = -(p->speedcmd)/2;
				//
			}
			else
			{ // Reach desired angle.
				p->motorspeed_l = 0;
				//
				p->motorspeed_r = 0;
				//
				p->finished = 1;
			}
		}
		break;

	case mot_line_follow:
		/*Follow a line*/

		// 1 - normalize the incomming data
		linesensor_normalizer(odo.linesensor);

		// 2 - do the simple lowest intensity algorithm "fl"

		
		// IF you want to hug the side
		
		int line_index;
		line_index = lowest_intensity(odo.linesensor, 1); // 1 for left 0 for right
		dV = 0.1 * (3.5 - line_index);
		//printf("dV: %f \t line_index: %d  \t travel_dist: %f |||||| \t %d \t %d\t %d\t %d\t %d\t %d\t %d\t %d\n", dV, line_index, o->traveldist,
		// odo.linesensor[0], odo.linesensor[1], odo.linesensor[2], odo.linesensor[3], odo.linesensor[4], odo.linesensor[5], odo.linesensor[6], odo.linesensor[7]);
		
		/*
		// center of gravity
		float cg;
		cg = center_of_gravity(odo.linesensor);
		dV = 0.1 * (3.5 - cg);S
		//printf("dV: %f \t cg: %f |||||| \t %d \t %d\t %d\t %d\t %d\t %d\t %d\t %d\n", dV, cg,
		// odo.linesensor[0], odo.linesensor[1], odo.linesensor[2], odo.linesensor[3], odo.linesensor[4], odo.linesensor[5], odo.linesensor[6], odo.linesensor[7]);
		*/

		// 3 - Calulcate remaining distance.
		d = p->dist - o->traveldist; 						
		
		if (d>0){
			p->motorspeed_l = p->speedcmd + dV / 2;
			p->motorspeed_r = p->speedcmd - dV / 2;
		}else{
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
			p->finished = 1;
		}

		// TODO: what if there is no more line? --- fix it
		// -- follow LEFT/RIGHT will just go in a circle. as line_index will be 0 or 7.
		// -- CG will go straight, but will actually 

		break;
	
	case mot_direction_control:
		/*

		TODO: give the forward motion some acceleration.

		know direction but not the angle. Does the same as the turn, 
		but with the turn you give a relative angle - now we get an absolute angle.  
		
		p->angle = reference angle, o->theta = target angle.
		*/

		// make the current/target angle into a +-180 degree angle.
		current_angle = fmod(o->theta,(2*M_PI));
		if (current_angle > M_PI){ 
			current_angle -= 2*M_PI;
		}

		// we assume the input angle is also between +-180 degree.
		double angle_diff;
		angle_diff = p->angle - current_angle;
		
		if (angle_diff < -M_PI){
			angle_diff+=2*M_PI;
		}else if(angle_diff > M_PI){
			angle_diff-=2*M_PI;
		}

		// speed levels: 0.1, 0.25, 0.5
		dV = 0.1 * (angle_diff); //2.125


		// So the motor voltage is still operational
		double sm = 0.01;
		if (fabs(dV/2) < sm){
			if (dV<0){
				dV = -2 * sm;
			}else{
				dV = 2 * sm;
			}	
		}

		p->motorspeed_l = -dV / 2;
		p->motorspeed_r = dV / 2;

		//printf("Difference: %f \t Current: %f \t RMOTOR SPEED: %f\n", fabs(p->angle-current_angle)*180/M_PI, current_angle*180/M_PI, dV/2);
		// && (fabs(p->motorspeed_r) == sm) && (fabs(p->motorspeed_l) == sm)
		if (fabs(p->angle-current_angle) < (0.1*M_PI/180)){
			driven_dist = (p->right_pos + p->left_pos) / 2 - p->startpos; 
			d = p->dist - driven_dist; 						// remaining distance
			//printf("Driven_distance = %f \t Startpos = %f \t p->dist = %f \t d = %f \n", driven_dist, p->startpos, p->dist, d);
			if (d>0){
				p->motorspeed_l = p->speedcmd;
				p->motorspeed_r = p->speedcmd;
			}else{
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
		}
		break;
	}
}

void linesensor_normalizer(int linedata[8]){
	for (int i = 0; i < 8; i ++){
		linedata[i] = (linedata[i] - MINLINE) / (MAXLINE - MINLINE);
		// Boolan values
		if (linedata[i]>0.5){linedata[i]=1;}
		else{linedata[i] = 0;}
	}
}

int lowest_intensity(int linedata[8], char followleft){
	/* Not assuming boolean values.
	NB! linedata is from right to left.

	If boolean it will just return the first/last index it finds.
	If we use linedata[i] <= lowest_value we get "follow left", if we use < we get "follow right"
	*/ 

	double lowest_value = 1;
	int center_index = 0;
	for (int i = 0; i < 8; i ++){
		if (followleft){
			if (linedata[i] <= lowest_value){ 
				lowest_value = linedata[i];
				center_index = i;
			}
		} else{
			if (linedata[i] < lowest_value){ 
				lowest_value = linedata[i];
				center_index = i;
			}
		}
	}
	return center_index;
}

float center_of_gravity(int linedata[8]){
	//TODO: what happens when the line is lost?
	// Assuming boolean intensity values
	// Finds the average index of zero values.
	int min_index_sum = 0;
	int min_index_count = 0;
	for (int i = 0; i < 8; i ++){
		if (linedata[i] == 0){
			min_index_sum += i;
			min_index_count++;
		}
	}

	// Avoid dividing by zero.
	if (!min_index_count){
		//return 4.5; // lost the line, spin slowly to one side
		return 3.5; // lost the line, go straight
	}

	return ((float)min_index_sum)/((float)min_index_count);
}


int dc(double dist, double speed, double angle, int time){
	if (time == 0){
		mot.cmd = mot_direction_control;
		mot.angle = angle;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}else{
		return mot.finished;
	}
}

int fol_dist(double dist, double speed, int time){
	if (time == 0){
		mot.cmd = mot_line_follow;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}else{
		return mot.finished;
	}
}

int fwd(double dist, double speed, int time)
{
	if (time == 0)
	{
		mot.cmd = mot_move;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}
	else
		return mot.finished;
}

int turn(double angle, double speed, int time)
{
	if (time == 0)
	{
		mot.cmd = mot_turn;
		mot.speedcmd = speed;
		mot.angle = angle;
		return 0;
	}
	else
		return mot.finished;
}




void sm_update(smtype *p)
{
	if (p->state != p->oldstate)
	{
		p->time = 0;
		p->oldstate = p->state;
	}
	else
	{
		p->time++;
	}
}
