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
#define TIMETIC 0.005
#define P_GAIN_ANGLE 0.05

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
	mot_line_follow_control,
	mot_direction_control	
	
};

void update_motcon(motiontype *p, odotype *o);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
int dc(double angle, int time);

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
	ms_direction_control
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


int main()
{
	int running, n = 0, arg, time = 0, m = 0;
	double dist = 0, angle = 0, speed = 0;
	double control_angle = 0;

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
			len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
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
	reset_odo(&odo);
	printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
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

		rhdSync();
		odo.left_enc = lenc->data[0];
		odo.right_enc = renc->data[0];
		update_odo(&odo);

		/****************************************
		/ mission statemachine   
		****************************************/
		sm_update(&mission);
		switch (mission.state)
		{
		case ms_init:
			n = 1; //4
			m = 2;
			dist = 2;
			speed = 0.2; // 0.2 0.4 0.6
			// angle = 90.0 / 180 * M_PI; // CW
			//angle = -90.0 / 180 * M_PI; //CCW
			angle = 149 * M_PI/180;
			control_angle = -120 * M_PI/180;
			//mission.state = ms_direction_control;
			mission.state = ms_fwd;
			break;

		case ms_fwd:
			//printf("ms_fwd!\n");
			if (fwd(dist, speed, mission.time))
				//mission.state = ms_turn;
				//mission.state = ms_direction_control;
				mission.state = ms_direction_control;

			break;

		case ms_direction_control:
			//printf("ms_direction_control!\n");
			if (dc(control_angle, mission.time))
				{
				m = m - 1;
				if (m == 0)
					mission.state = ms_end; 
				else
					mission.state = ms_turn;
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
		if (time % 100 == 0)
			//    printf(" laser %f \n",laserpar[3]);
			time++;
		/* stop if keyboard is activated
*
*/
		ioctl(0, FIONREAD, &arg);
		if (arg != 0)
			running = 0;


		// Do some logging
		if (log_index == MAXINT){
			printf("looped around\n");
			log_index = 0;
		}
		
		clock_t current_time = clock();
		double time_spent = (double)(current_time - start_time) / 10000;

		logg[log_index][0] = mission.time;
		logg[log_index][1] = mot.motorspeed_l;
		logg[log_index][2] = mot.motorspeed_r;
		logg[log_index][3] = time_spent;
		logg[log_index][4] = odo.x_pos;
		logg[log_index][5] = odo.y_pos;
		logg[log_index][6] = odo.theta;
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
			p->curcmd = mot_direction_control;
			break;
		}
		p->cmd = 0;
	}

	switch (p->curcmd)
	{
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

	case mot_line_follow_control:
		/*Follow a line*/ 
		break;
	
	case mot_direction_control:
		/* 
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


		double dV = 0.5 * (angle_diff); //2.125


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

		printf("Difference: %f \t Current: %f \t RMOTOR SPEED: %f\n", fabs(p->angle-current_angle)*180/M_PI, current_angle*180/M_PI, dV/2);
		//((round(p->angle*10)/10)==(round(current_angle*10)/10)) && (p->motorspeed_r <= 0.005) && (p->motorspeed_l <= 0.005)
		// && (fabs(p->motorspeed_r) <= 0.01) && (fabs(p->motorspeed_l) <= 0.01)
		if (fabs(p->angle-current_angle) < (0.1*M_PI/180) && (fabs(p->motorspeed_r) == sm) && (fabs(p->motorspeed_l) == sm)){
			printf("STOP SPINNING PLEASE :(\n");
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
			p->finished = 1;
		}
		break;
	}
}


int dc(double angle, int time){
	if (time == 0){
		mot.cmd = mot_direction_control;
		mot.angle = angle;
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
