/*
 * An example SMR program.
 *
 * FINAL VERSION SMR v2.0
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
void print_cmd(int state);

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
#define ROBOTPORT 24902 //8000//24902 //8000
#define MAXINT 65536
#define TIMETIC 0.01
#define P_GAIN_ANGLE 0.05

// 
#define WHITELINE 100 	// SIM 255, 94 for white paper, white tape was about 84
#define BACKGROUND 128 	//SIM 128, background dependent on shadow and light, was around 80
#define BLACKLINE 0  	// SIM 0, BLACK TAPE / paper IS ABOUT 54

#define KA 16 //10.0 //SIM 16.0
#define KB 76  //77.0 //SIM 76.0

#define KA0 16 //LEFT
#define KB0 76 //LEFT
#define KA1 16.89 //FRONTLEFT
#define KB1 61.49 //FRONTLEFT
#define KA2 16.97 //FRONTMIDDLE
#define KB2 54.93 //FRONTMIDDLE
#define KA3 16.65 //FRONTRIGHT
#define KB3 69.08 //FRONTRIGHT
#define KA4 16 //RIGHT
#define KB4 76 //RIGHT

// Some simulation parameters
#define MAXIRDIST 0.9 //Max distace of IR sensors (around 0.88)
#define ACCELERATION 0 //Should we use acceleration in the code?
#define AVG 1 // Should we average out the ir readings or not?
#define PD 1 // Using PD to hug wall instead of P.

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
	int irsensor[5];
	float avgir[5]; //Average ir-data
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
	double startpos, startangle;
	// Current speed of the system
	double currentspeed;
	// The type of line to follow
	char linetype;
	// The condition to stop a motion
	int condition_type;
	// The IR distance required to maintain a condition
	double ir_dist;
	//Index of laser used
	int laser_index;
	//x dist to gate
	double lenghtToGate; 
	double prevdV; // For the PD controller
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
	mot_drive,
	mot_turn,
	mot_direction_control,
	mot_line_follow,
	mot_wallhug
	
};

enum conditions{
	drivendist = 1,
	irdistfrontmiddle,
	irdistfrontright,
	irdistfrontleft,
	irdistright_less, //<
	irdistright_more, //>
	irdistleft_less,  //<	
	irdistleft_more,   //>
	crossingblack,
	foundBlackLine,
	foundGate,
	middleOfGate,
	l, 			//hugwall left, right
	r,
	dist_lida
};

void update_motcon(motiontype *p, odotype *o);

int fwd(double dist, double speed, int time);
int drive(int condition_type, double condition, double speed, int time);
int turn(double angle, double speed, int time);
int dc(double dist, double speed, double angle, int time);
int follow_line(int condition_type, double condition, char linetype, double speed, int time);
int lida_dist(int index, double length);
int hug_wall(int condition_type, double condition, double speed, double dist, int time);

void linesensor_normalizer(int linedata[8], float line_intensity[8]);
void irsensor_transformer(int irdata[5], float irdistances[5]);
void irsensor_transformer_avg(float irdata[5], float irdistances[5]);
void smooth_ir_data(int irdata[5], float avgir[5]);


float center_of_gravity(float linedata[8], char color);
int lowest_intensity(float linedata[8], char followleft);
int crossingblackline(float line_intensity[8]);
int blackLineFound(int linedata[8], int amountOfBlack);
int checkCondition(motiontype *p, odotype *o);
int gateFound(int index); 
void command(double missions[100][7], int mission, int condition,
double condition_parameter, double speed, int linetype, double distance, double angle);
int update_command_no();
void cmd_followline(double missions[100][7],int linetype, double speed, int condition,
 double condition_parameter);
void cmd_drive(double missions[100][7], int condition,
    double condition_parameter, double speed);
void cmd_fwd(double missions[100][7], double distance, double speed);
void cmd_turnr(double missions[100][7], double speed, double angle);
void cmd_followwall(double missions[100][7], int condition, double distance, double speed);
int lida_dist(int index, double length);

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
	ms_houston,
	ms_fwd,
	ms_drive,
	ms_turn,
	ms_end,
	ms_direction_control,
	ms_followline,
	ms_wallhug
};

enum linetypes {
	br,
	bl,
	bm,
	wm
};


int main(){
	int running, arg, time = 0;
	double dist = 0, angle = 0, speed = 0, condition_param = 0;
	char linetype = 0; // 0 for br, 1 for bl, 2 for cg
	int condition = -1; // -1 means no condition
	// Mission array
	int j = 0;
	int mission_lenght = 0;
	double missions[100][7];
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
	// Linesensor
	memcpy(odo.linesensor, linesensor->data, sizeof(odo.linesensor));
	// IRsensor
	memcpy(odo.irsensor, irsensor->data, sizeof(odo.irsensor));

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

	//double logg[MAXINT][7]; 
	//int log_index = 0;
	//clock_t start_time = clock();

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
		memcpy(odo.irsensor, irsensor->data, sizeof(odo.irsensor));
		smooth_ir_data(odo.irsensor, odo.avgir);
		
		// Execute ODOMETRY
		update_odo(&odo);

		/****************************************
		/ mission statemachine   
		****************************************/
		sm_update(&mission);
		switch (mission.state)
		{
		case ms_init:
			// MISSION ARRAY: ms, cond, condparam, speed, linetype, distance, angle
			mission.state = ms_houston;
			mission_lenght = 0;
			j = 0;

			/*Interesting observations:
				- You cannot use two followline commands right after each other 
				- Turn on or off simulated acceleration using ACCELERATION macro.
			*/

			// Obstacle 1 works 
			cmd_followline(missions,br,0.10,irdistfrontmiddle,0.2);
			cmd_turnr(missions, 0.1, 180);
			//cmd_followline(missions, bm, 0.1, drivendist, 1.2);
			//cmd_followline(missions, bm, 0.1, crossingblack, 0);
			cmd_followline(missions, bl, 0.1, crossingblack,0);
			cmd_turnr(missions, 0.1, 180);
			
			// Obstacle 2 works
			cmd_followline(missions, bm, 0.1, drivendist, 0.53);
			cmd_followline(missions, bl, 0.1, drivendist, 1.2);
			cmd_followline(missions, bm, 0.1, irdistfrontmiddle, 0.15);

			cmd_followline(missions, bm, 0.1, crossingblack, 0);
			cmd_followline(missions, bm, 0.08, drivendist, 0.16);

			cmd_fwd(missions, -1, -0.1);
			cmd_turnr(missions,0.10,-180);
			cmd_followline(missions, bl, 0.15, drivendist, 1.5);

			cmd_followline(missions, bm, 0.15, crossingblack, 0);
			cmd_fwd(missions, 0.25, 0.15);
			cmd_turnr(missions, 0.15,90);
			cmd_followline(missions, bm, 0.15, crossingblack, 0);
			cmd_fwd(missions, 0.225, 0.15);
			
			cmd_followline(missions, bm, 0.15, crossingblack, 0);
		
			// Obstacle 3 works
			cmd_followline(missions,bm,0.1,foundGate,0);
			cmd_drive(missions,drivendist,0.3,0.1);
			cmd_followline(missions,bm,0.1,foundGate,0);
			cmd_fwd(missions, -0.02, -0.15);
			cmd_turnr(missions, 0.1, 75);
			cmd_drive(missions,drivendist, 0.3, 0.1);
			cmd_drive(missions,dist_lida,0.25,0.1);
			
			
			// Obstacle 4
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 90*M_PI/180);
			// Hug the wall
			command(missions, ms_wallhug, irdistright_more, 0.87, 0.1, 0, 0.35, 0);
			cmd_fwd(missions, 0.40, 0.1); // 0.42
			// Go through the first gate
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, -90*M_PI/180);
			command(missions, ms_fwd, 0, 0, 0.1, 0, 0.95, 0);
			command(missions, ms_turn, 0, 0, 0.15, 0, 0, -90*M_PI/180);
			// Find the wall and hug it
			command(missions, ms_fwd, 0, 0, 0.1, 0, 0.2, 0);
			command(missions, ms_wallhug, irdistright_more, 0.8, 0.1, 0, 0.36, 0);
			cmd_fwd(missions, 0.42, 0.1);
			// Go through the second gate
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, -90*M_PI/180);
			command(missions, ms_followline, drivendist, 1.1, 0.10, bm, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, -180*M_PI/180);
			
			command(missions, ms_followline, crossingblack, 0, 0.2, bm, 0, 0);
			
			// Obstacle 5
			//command(missions, ms_turn, 0, 0, 0.10, 0, 0, 90*M_PI/180);
			//cmd_drive(missions, crossingblack, 0, 0.1);
			//cmd_fwd(missions, 0.2, 0.1);
			//command(missions, ms_turn, 0, 0, 0.10, 0, 0, -90*M_PI/180);

			// Actually follow WM
			command(missions, ms_fwd, 0, 0, 0.1, 0, 0.45, 0);
			command(missions, ms_followline, crossingblack, 0, 0.05, wm, 0, 0);
			command(missions, ms_fwd, 0, 0, 0.1, 0, 0.17, 0);
			command(missions, ms_turn, 0, 0, 0.15, 0, 0, -90*M_PI/180);
			

			
			// Obstacle 6
			// Find the garage
			command(missions, ms_followline, irdistfrontmiddle, 0.2, 0.1, bm, 0, 0);
			// Follow the east wall
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, -90*M_PI/180);
			command(missions, ms_drive, irdistleft_more, 0.8, 0.1, 0, 0, 0);
			// Clear the corner
			command(missions, ms_drive, drivendist, 0.5, 0.1, 0, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 90*M_PI/180);
			command(missions, ms_drive, drivendist, 0.25, 0.1, 0, 0, 0);
			// Follow the north wall
			command(missions, ms_wallhug, irdistleft_more, 0.8, 0.1, 0, 0.20, 0);
			// Clear the corner
			command(missions, ms_drive, drivendist, 0.5, 0.1, 0, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 90*M_PI/180);
			command(missions, ms_drive, drivendist, 0.2, 0.1, 0, 0, 0);
			// Follow the west wall
			command(missions, ms_wallhug, irdistleft_more, 0.8, 0.1, 0, 0.22, 0);
			// Clear the corner
			command(missions, ms_drive, drivendist, 0.40, 0.1, 0, 0, 0); //0.35
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 90*M_PI/180);
			// Follow the south wall, get to the gate
			command(missions, ms_drive, irdistfrontleft, 0.2, 0.1, 0, 0, 0);
			command(missions, ms_drive, drivendist, 0.2, 0.1, 0, 0, 0);
			// Open the gate
			command(missions, ms_drive, drivendist, 0.05, 0.1, 0, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 12*M_PI/180);
			command(missions, ms_drive, drivendist, 0.16, 0.1, 0, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 20*M_PI/180);
			command(missions, ms_drive, drivendist, 0.30, 0.1, 0, 0, 0);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 20*M_PI/180);
			command(missions, ms_drive, drivendist, 0.25, 0.1, 0, 0, 0);
			cmd_fwd(missions, -0.05, -0.1);
			command(missions, ms_turn, 0, 0, 0.10, 0, 0, 100*M_PI/180);
			cmd_fwd(missions, 0.05, 0.1);

			// Enter the garage
			command(missions, ms_followline, drivendist, 0.35, 0.1, bm, 0, 0);
			command(missions, ms_followline, irdistfrontmiddle, 0.2, 0.1, bm, 0, 0);

			mission_lenght = update_command_no();
			break;

		case ms_houston:
			if (j==1){
				printf("boxdist= %f\n",(fabs(odo.y_pos)+0.2+0.26-0.045));
			}
			if(j==mission_lenght){
				printf("All missions complete\n");
				 
				mission.state = ms_end;
				break;
			}
			mission.state = missions[j][0]; 
			condition = missions[j][1];
			condition_param = missions[j][2];
			speed = missions[j][3];
			linetype = missions[j][4];
			dist = missions[j][5];
			angle = missions[j][6];
			print_cmd(mission.state); //Current mission: number
			j+=1;
			break;
		case ms_fwd:
			if (fwd(dist, speed, mission.time))
				mission.state = ms_houston;
			break;
		
		case ms_drive:
			if (drive(condition, condition_param, speed, mission.time))
				mission.state = ms_houston;
			break;

		case ms_followline:
			if (follow_line(condition, condition_param, linetype, speed, mission.time))
				mission.state = ms_houston;
			break;
		
		case ms_wallhug:
			if (hug_wall(condition, condition_param, speed, dist, mission.time))
				mission.state = ms_houston;
			break;

		case ms_direction_control:
			//printf("ms_direction_control!\n");
			if (dc(dist, speed, angle, mission.time)){
				mission.state = ms_houston;
			}
			break;

		case ms_turn:
			//printf("ms_turn!\n");
			if (turn(angle, speed, mission.time)){
				mission.state = ms_houston;
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

		//write_laser_log(laserpar);
		
		if (time % 100 == 0){
			/*
			   printf(" laser %f %f %f %f %f %f %f %f %f %f \n", laserpar[0], laserpar[1], laserpar[2], laserpar[3],
			    laserpar[4], laserpar[5], laserpar[6],
	    		laserpar[7], laserpar[8], laserpar[9]);
			*/
			time++;
		}
		/* stop if keyboard is activated*/
		ioctl(0, FIONREAD, &arg);
		if (arg != 0)
			running = 0;


		// Do some logging
		// if (log_index == MAXINT){
		// 	printf("looped around\n");
		// 	log_index = 0;
		// }
		
		// clock_t current_time = clock();
		// double time_spent = (double)(current_time - start_time) / 10000; //TODO: use time() function instead 

		// logg[log_index][0] = mission.time; 		// no. tics for the current mission.
		// logg[log_index][1] = mot.motorspeed_l;	// 
		// logg[log_index][2] = mot.motorspeed_r;	//
		// logg[log_index][3] = time_spent;		// "time spent" - using clock and a no. pulled out of our asses.
		// logg[log_index][4] = odo.x_pos;			// x pos of robot
		// logg[log_index][5] = odo.y_pos;			// y pos of robot
		// logg[log_index][6] = odo.theta;			// absolute theta of robot
		// log_index++;

	} /* end of main control loop */
	
	speedl->data[0] = 0;
	speedl->updated = 1;
	speedr->data[0] = 0;
	speedr->updated = 1;

	// write the log to a .dat file
	//write_log(logg);
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
	double max_acceleration = 0.5;
	double clock_acceleration = max_acceleration * TIMETIC;
	double driven_dist;
	double d;
	static int deaccel_flag = 0;
	double current_angle = 0;
	double dV;
	float irdistances[5];
	float line_intensity[8];
	char go_on;
	float p_gain;
	float d_gain;
	float actual_dist;
	// So the motor voltage is still operational IRL
	double sm = 0.01;


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
		
		case mot_drive:
			p->startpos = (p->left_pos + p->right_pos) / 2;
			p->curcmd = mot_drive;
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

		case mot_wallhug:
			p->startangle = o->theta;
			p->curcmd = mot_wallhug;
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
		if (ACCELERATION){
			d = fabs(p->dist - driven_dist); // remaining distance
			// We need to deaccelerate
			if ((deaccel_flag) || (fabs(p->currentspeed) >= sqrt(2 * max_acceleration * d))){ 
				if (!deaccel_flag){
					deaccel_flag = 1;
				}

				if (p->currentspeed > clock_acceleration){ // Accelerate
					p->currentspeed -= clock_acceleration;
				}
				else if(p->currentspeed < -clock_acceleration){
					p->currentspeed += clock_acceleration;
				
				}else{ // Completely stopped
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;

			}// Keep accelerating or moving forward
			else{ 
				if ((p->speedcmd - p->currentspeed) > clock_acceleration){ // Accelerate
					p->currentspeed += clock_acceleration;
				}
				else if((p->speedcmd - p->currentspeed) < clock_acceleration){
					p->currentspeed -= clock_acceleration;
				}
				else{ // Max speed 
					p->currentspeed = p->speedcmd;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}
		}else{
			if(p->speedcmd>=0){ //Forward
				if (p->dist <= driven_dist){
					p->currentspeed = 0;
					p->finished = 1;
				}else{
					p->currentspeed=p->speedcmd;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}else{ //Reversing
				if (p->dist >= driven_dist){
					p->currentspeed = 0;
					p->finished = 1;
				}else{
					p->currentspeed=p->speedcmd;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}
		}	
		break;

	case mot_drive:
		/* Moving forward, with acceleration, under certain conditions */
		d = 20;
		linesensor_normalizer(odo.linesensor, line_intensity);
		
		if(p->condition_type==drivendist){
			driven_dist = (p->right_pos + p->left_pos) / 2 - p->startpos; 
			d = p->dist - driven_dist; 						// remaining distance

		}else if(p->condition_type==irdistfrontmiddle){
			if (AVG){
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
			}
			d = irdistances[2] - p->ir_dist;

		}else if(p->condition_type==irdistfrontleft){
			if (AVG){
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
			}			
			d = irdistances[1] - p->ir_dist; 

		}else if(p->condition_type==irdistright_more){
			if (AVG){
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
			}
			if (irdistances[4] > p->ir_dist){
				deaccel_flag=1;
			}

		}else if(p->condition_type==irdistleft_more){
			if (AVG){
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
			}
			if (irdistances[0] > p->ir_dist){
				deaccel_flag=1;
			}

		}else if(p->condition_type == dist_lida){
			d = 20; 
			deaccel_flag=(lida_dist(4,p->ir_dist));

		}else if(p->condition_type==crossingblack){
			
			if (crossingblackline(line_intensity)){
				deaccel_flag=1;
			}

		}else{
			printf("You have not set a proper condition. RIP!\n");
			exit(-1);
		}


		if(ACCELERATION){
			// We need to deaccelerate
			if ((deaccel_flag) || (p->currentspeed >= sqrt(2 * max_acceleration * d))){ 
				if (!deaccel_flag){
					deaccel_flag = 1;
				}
				
				if (fabs(0 - p->currentspeed) > clock_acceleration){
					p->currentspeed -= clock_acceleration;
				}else{ // Completely stopped
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;

			}// Keep accelerating or moving forward
			else{ 
				if (fabs(p->speedcmd - p->currentspeed) > clock_acceleration){ // Accelerate
					p->currentspeed += clock_acceleration;
				}else{ // Max speed 
					p->currentspeed = p->speedcmd;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}
		}else{ // NO Acceleration
			if (d<=0 || deaccel_flag){
				p->currentspeed = 0;
				p->finished = 1;
				deaccel_flag = 0;
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}else{
				p->currentspeed = p->speedcmd;
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;
			}
		}
		break;

	case mot_turn:
		// This will make a hard turn / relative to the robot itself.
		if (p->angle > 0){ // left turn
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
		/*Follow a line under some condition*/

		// 1 - normalize linesensor data
		linesensor_normalizer(odo.linesensor, line_intensity);


		// 2 - Do the simple lowest intensity algorithm "fl"
		if (p->linetype==bm){ // center of gravity
			dV = 0.1 * (3.5 - center_of_gravity(line_intensity, 0));
		}else if(p->linetype==wm){
			dV = 0.1 * (3.5 - center_of_gravity(line_intensity, 1));
		}else{
			dV = 0.1 * (3.5 - lowest_intensity(line_intensity, p->linetype));
		}

		// 3 - Calulcate stop condition.
		go_on=0;
		if (p->condition_type==drivendist){
			d = p->dist - o->traveldist; //distance left
			go_on = (d>0);
		
		}else if(p->condition_type==irdistfrontmiddle){
			if (AVG){
				irsensor_transformer_avg(o->avgir, irdistances);
				go_on = (p->ir_dist) < (irdistances[2]); 
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
				go_on = (p->ir_dist) < (irdistances[2]); 
			}
			
			// Solution one: single point
			//printf("IRFM § Single point: %0.2f § ", irdistances[2]);
			// Solution two: Average in front
			//printf("avg three: %0.2f § ",((irdistances[2]+irdistances[3]+irdistances[1])/3));
			// Solution three: average in time
			//printf("avg time: %0.2f\n", irdistances[2]);
			

		}else if(p->condition_type==crossingblack){
			// Fill irdistances with meter data
			go_on=!crossingblackline(line_intensity);
		}else if(p->condition_type==foundBlackLine){
			// Fill irdistances with meter data
			go_on=!blackLineFound(odo.linesensor,1);
		}else if(p->condition_type == foundGate){
			go_on = !(gateFound(p->laser_index));
		}

		// So the motor voltage is still operational IRL
		if (fabs(dV/2) < sm){
			if (dV<0){
				dV = -2 * sm;
			}else{
				dV = 2 * sm;
			}	
		}

		if (ACCELERATION){
			// Deacceleration
			// Go on or stop
			if (!go_on){
				deaccel_flag = 1;
			}
			if ((deaccel_flag) ){ 
				if (fabs(0 - p->currentspeed) > clock_acceleration){
					p->currentspeed -= clock_acceleration;
				}else{ // Completely stopped
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;

			}// Keep accelerating or moving forward
			else{ 
				if (fabs(p->speedcmd - p->currentspeed) > clock_acceleration){ // Accelerate
					p->currentspeed += clock_acceleration;
				}else{ // Max speed 
					p->currentspeed = p->speedcmd;
				}
				p->motorspeed_l =p->currentspeed + dV / 2;
				p->motorspeed_r = p->currentspeed - dV / 2;
			}
		}else{
			// No acceleration
			if (!go_on){
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
					go_on = 1;
					p->motorspeed_l = p->currentspeed;
					p->motorspeed_r = p->currentspeed;
			}else{
				p->currentspeed = p->speedcmd;
				p->motorspeed_l = p->currentspeed + dV / 2;
				p->motorspeed_r = p->currentspeed - dV / 2;
			}
		}
		break;

	case mot_wallhug:
		/* Hug a wall at a given distance */
		// Calulcate stop condition.
		go_on=0;
		p_gain=0.2;
		actual_dist=0;
		d_gain = 0.8; //0.8

		if (p->condition_type==irdistright_more){
			if (AVG){
				irsensor_transformer(odo.irsensor, irdistances);
				go_on = (p->ir_dist) > (irdistances[4]);
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
				go_on = (p->ir_dist) > (irdistances[4]); 
			}

			actual_dist = irdistances[4] * sin(M_PI/2 - (fabs(o->theta - p->startangle)));
			dV = p_gain * (p->dist - actual_dist); //P-controller
			if (PD){
				if (dV>0){ dV -= d_gain*fabs(dV-p->prevdV); }
				else if (dV<0){ dV += d_gain*fabs(dV-p->prevdV); }
				p->prevdV = dV;
			}

		}else if(p->condition_type==irdistleft_more){
			if (AVG){
				irsensor_transformer(odo.irsensor, irdistances);
				go_on = (p->ir_dist) > (irdistances[0]); 
				irsensor_transformer_avg(o->avgir, irdistances);
			}else{
				irsensor_transformer(odo.irsensor, irdistances);
				go_on = (p->ir_dist) > (irdistances[0]); 
			}

			actual_dist = irdistances[0] * sin(M_PI/2 - (fabs(o->theta - p->startangle)));
			dV = -p_gain * (p->dist - actual_dist);
			if (PD){
				if (dV>0){ dV -= d_gain*fabs(dV-p->prevdV); }
				else if (dV<0){ dV += d_gain*fabs(dV-p->prevdV); }
				p->prevdV = dV;
			}

		}

		// So the motor voltage is still operational IRL
		if (fabs(dV/2) < sm){
			if (dV<0){
				dV = -2 * sm;
			}else{
				dV = 2 * sm;
			}	
		}

		// Go on or stop
		if (!go_on){
			deaccel_flag =1;
		}
		if(ACCELERATION){
			// Deacceleration
			if ((deaccel_flag)){ 
				if (fabs(0 - p->currentspeed) > clock_acceleration){
					p->currentspeed -= clock_acceleration;
				}else{ // Completely stopped
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
				}
				p->motorspeed_l = p->currentspeed;
				p->motorspeed_r = p->currentspeed;

			}// Keep hugging wall
			else{ 
				if (fabs(p->speedcmd - p->currentspeed) > clock_acceleration){ // Accelerate
					p->currentspeed += clock_acceleration;
				}else{ // Max speed 
					p->currentspeed = p->speedcmd;
				}
				p->motorspeed_l = p->currentspeed - dV / 2;
				p->motorspeed_r = p->currentspeed + dV / 2;
			}
		}else{
			// No acceleration
			if (!go_on){
					p->currentspeed = 0;
					p->finished = 1;
					deaccel_flag = 0;
					go_on = 1;
					p->motorspeed_l = p->currentspeed;
					p->motorspeed_r = p->currentspeed;
			}else{
				p->currentspeed = p->speedcmd;
				p->motorspeed_l = p->currentspeed - dV / 2;
				p->motorspeed_r = p->currentspeed + dV / 2;
			}
		}

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

void smooth_ir_data(int irdata[5], float avgir[5]){
	static int smoot_ir[50][5]={0};
	static int index=0;

	// First insert new data into array
	for (int i=0; i<5; i++){
		smoot_ir[index][i]=irdata[i];
		avgir[i] = 0; // Reset this array
	}

	// Then get the average data of all points, put into new array
	for (int j=0; j<50; j++){
		for (int i=0; i<5; i++){
			avgir[i] += ((float)smoot_ir[j][i] / 50.0);
		}
	}

	if (index==49){
		index=0;
	}else{
		index++;
	}
}


void irsensor_transformer(int irdata[5], float irdistances[5]){
	/*irdata is raw ir-data. Irdistances are returned.*/		
	irdistances[0] = (float)KA0 / ((float)irdata[0] - (float)KB0);
	irdistances[1] = (float)KA1 / ((float)irdata[1] - (float)KB1);
	irdistances[2] = (float)KA2 / ((float)irdata[2] - (float)KB2);
	irdistances[3] = (float)KA3 / ((float)irdata[3] - (float)KB3);
	irdistances[4] = (float)KA4 / ((float)irdata[4] - (float)KB4);
	
	for (int i = 0; i < 5; i ++){
		if (irdistances[i] < 0){ // Sometimes, instead of giving maxdist, it gave negative values.
			irdistances[i] = MAXIRDIST;
		}
	}
}

void irsensor_transformer_avg(float irdata[5], float irdistances[5]){
	/*irdata is average ir-data. Irdistances are returned.*/	
	irdistances[0] = (float)KA0 / ((float)irdata[0] - (float)KB0);
	irdistances[1] = (float)KA1 / ((float)irdata[1] - (float)KB1);
	irdistances[2] = (float)KA2 / ((float)irdata[2] - (float)KB2);
	irdistances[3] = (float)KA3 / ((float)irdata[3] - (float)KB3);
	irdistances[4] = (float)KA4 / ((float)irdata[4] - (float)KB4);
	
	for (int i = 0; i < 5; i ++){
		if (irdistances[i] < 0){ // Sometimes, instead of giving maxdist, it gave negative values.
			irdistances[i] = MAXIRDIST;
		}
	}
}


void linesensor_normalizer(int linedata[8], float line_intensity[8]){
    float thresholds[8] = {0.59, 0.587, 0.61, 0.585, 0.68, 0.65, 0.620, 0.57}; // SMR11 - Paper
	//float thresholds[8] = {0.57, 0.59, 0.60, 0.62, 0.635, 0.635, 0.62, 0.62}; // SMR7
	//float thresholds[8] = {0.57, 0.59, 0.60, 0.62, 0.635, 0.635, 0.62, 0.62}; // SMR5
	/* SMR 11
		WHITE PAPER
		63.11  61.63  63.75  61.62  75.43  70.99  67.07  59.70  
		WHITE TAPE
		60.72  59.93  62.05  60.31  71.74  68.30  64.57  58.45  
		BLACK PAPER
		44.98  46.31  45.26  45.17  45.83  45.43  45.98  45.88
		BLACK TAPE
		46.46  47.11  45.96  45.54  47.22  46.09  46.62  46.07
		BACKGROUND
		58.46  56.79  58.50  57.08  65.41  62.87  61.53  56.08
		58.48  57.80  58.19  56.51  66.44  63.03  60.74  55.82
		FINAL DAY!
		WHITE TAPE
		60.88  60.01  61.89  60.16  70.87  67.95  63.80  58.07  
		BLACK TAPE
		45.51  46.64  45.19  44.90  46.82  45.68  45.86  45.15
		BACKGROUND
		55.47  55.93  56.92  54.94  62.73  60.24  58.39  53.51  
		55.44  55.94  56.82  55.08  62.82  60.24  58.33  53.63
	
	*/
	/* SMR 7
		BACKGROUND
		54.42  56.25  57.56  58.25  59.04  60.15  59.96  59.34
		BLACK
		47.60  47.52  49.03  48.48  48.71  48.96  48.92  49.60
		WHITE
		59.59  63.22  64.86  66.10  67.19  67.56  65.52  64.74
	*/
	/* SMR 5
		WHITE
		83.41  86.14  77.99  88.61  89.73  87.83  88.28  86.35 
		BLACK
		52.98  53.28  52.25  52.97  53.03  53.21  53.78  56.12
		BACKGROUND
		76.25  78.01  70.41  77.81  79.44  77.96  77.88  77.17
	*/
	for (int i = 0; i < 8; i ++){
        line_intensity[i] = (float)(linedata[i] - BLACKLINE) / (float)(WHITELINE - BLACKLINE); // 0.15
        
		if (line_intensity[i]<0.515){ // SMR11/7: 0.525, SMR5: 0.65
            line_intensity[i]=0;
        }else if (line_intensity[i]>thresholds[i]){ // White line
            line_intensity[i]=1;
        }else{
            line_intensity[i]=0.5; // Floor
        }
		
		//printf("%3d (%0.1f)  ", linedata[i], line_intensity[i]);
    }
	
	//printf("\n");
	
	




}

int lowest_intensity(float linedata[8], char followleft){
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

float center_of_gravity(float linedata[8], char color){
	// Assuming boolean intensity values
	// Finds the average index of zero values.
	int min_index_sum = 0;
	int min_index_count = 0;
	for (int i = 0; i < 8; i ++){
		if (color){
			if (linedata[i] == 1){ // White
				min_index_sum += i;
				min_index_count++;
			}
		}else{
			if (linedata[i] == 0){ // Black
				min_index_sum += i;
				min_index_count++;
			}

		}

	}

	// Avoid dividing by zero.
	if (!min_index_count){
		//printf("No line found!\n");
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

int drive(int condition_type, double condition, double speed, int time){
	if (time == 0){
		mot.cmd = mot_drive;
		mot.speedcmd = speed;
		mot.dist = 0;
		mot.ir_dist=0;
		mot.condition_type = condition_type;

		// TODO: implement more conditions
		if (condition_type==drivendist){
			mot.dist = condition;
		}
		else if(condition_type==irdistfrontmiddle){
			mot.ir_dist = condition;
		}else if(condition_type==irdistfrontleft){
			mot.ir_dist = condition;
		}else if(condition_type==irdistright_more){
			mot.ir_dist = condition;
		}else if(condition_type==irdistleft_more){
			mot.ir_dist = condition;
		}else if(condition_type==dist_lida){
			mot.ir_dist = condition;
			//printf("condition: %f\n",condition);
		}else if(condition_type==crossingblack){
			mot.dist = 0;
		}else{
			printf("Wrong condition type inserted.\n");
		}
		return 0;

	}else{
		return mot.finished;
	}
}

int follow_line(int condition_type, double condition, char linetype, double speed, int time){
	if (time == 0){
		mot.cmd = mot_line_follow;
		mot.speedcmd = speed;
		mot.dist = 0;
		mot.linetype = linetype; // 0 for br, 1 for bl, 2 for cg
		mot.condition_type = condition_type;

		// TODO: implement more conditions
		if (condition_type==drivendist){
			mot.dist = condition;
		}
		else if(condition_type==irdistfrontmiddle){
			mot.ir_dist = condition;
		}else if(condition_type == foundGate){
			mot.laser_index = condition; 
		}else if(!(condition_type == dist_lida) && !(condition_type==crossingblack) && !(condition_type == foundBlackLine)){
			printf("Wrong condition type inserted.\n");
		}
		
		
		return 0;

	}else{
		return mot.finished;
	}
}

int hug_wall(int condition_type, double condition, double speed, double dist, int time){
	if (time == 0){
		mot.cmd = mot_wallhug;
		mot.speedcmd = speed;
		mot.dist = dist; // Will be used as dist from wall
		mot.condition_type = condition_type;

		if(condition_type==irdistleft_more){
			mot.ir_dist = condition;
		}else if(condition_type == irdistright_more){
			mot.ir_dist = condition;
		}else{
			printf("Wrong condition type inserted.\n");
		}
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

int update_command_no(){
    static int command_no = 0;

	static int init_flag = 0;

	if (command_no==0 && init_flag==0){
		init_flag = 1;
		//printf("initialized and returned command_no: %d\n", command_no);
		return command_no;
	}
    command_no++;
	//printf("command_no: %d\n", command_no);
    return command_no;
}

void cmd_followline(double missions[100][7],int linetype, double speed, int condition,
 double condition_parameter){
		int command_no = update_command_no();	
		missions[command_no][0] = ms_followline;
		missions[command_no][1] = condition;
		missions[command_no][2] = condition_parameter;
		missions[command_no][3] = speed;
		missions[command_no][4] = linetype;
        missions[command_no][5] = 0;
		missions[command_no][6] = 0;
}

void cmd_drive(double missions[100][7], int condition,
 double condition_parameter, double speed){
		int command_no = update_command_no();	
		missions[command_no][0] = ms_drive;
		missions[command_no][1] = condition;
		missions[command_no][2] = condition_parameter;
		missions[command_no][3] = speed;
		missions[command_no][4] = 0;
		missions[command_no][5] = 0;
		missions[command_no][6] = 0;
}

void cmd_fwd(double missions[100][7], double distance, double speed){
		int command_no = update_command_no();	
		missions[command_no][0] = ms_fwd;
		missions[command_no][1] = 0;
		missions[command_no][2] = 0;
		missions[command_no][3] = speed,
		missions[command_no][4] = 0;
		missions[command_no][5] = distance;
		missions[command_no][6] = 0;
}

void cmd_turnr(double missions[100][7], double speed, double angle){
		int command_no = update_command_no();	
		missions[command_no][0] = ms_turn;
		missions[command_no][1] = 0;
		missions[command_no][2] = 0;
		missions[command_no][3] = speed,
		missions[command_no][4] = 0;
		missions[command_no][5] = 0;
		missions[command_no][6] = angle*M_PI/180;
}

void cmd_followwall(double missions[100][7], int condition, double distance, double speed){
		if (condition==l){
			condition = irdistleft_more;
		}
		else if (condition == r){
			condition = irdistright_more;
		}

		int command_no = update_command_no();
			
		missions[command_no][0] = ms_wallhug;
		missions[command_no][1] = condition;
		missions[command_no][2] = 0.8;
		missions[command_no][3] = speed,
		missions[command_no][4] = 0;
		missions[command_no][5] = distance;
		missions[command_no][6] = 0;
 
}
void command(double missions[100][7], int mission, int condition,
 double condition_parameter, double speed, int linetype, double distance, double angle){
		int command_no = update_command_no();
		missions[command_no][0] = mission;
		missions[command_no][1] = condition;
		missions[command_no][2] = condition_parameter;
		missions[command_no][3] = speed,
		missions[command_no][4] = linetype;
		missions[command_no][5] = distance;
		missions[command_no][6] = angle;
 
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



int crossingblackline(float line_intensity[7]){
	for (int i=1;i<7;i++){
		if (line_intensity[i]!=0){
			return 0;
		}
	}
	if (line_intensity[0]==0 || line_intensity[7]==0){
		return 1;
	}
	return 0; //returns 1 if blackcrossing is detected else return 0
}  

int blackLineFound(int linedata[8],int amountOfBlack ){ //Takes line data, and how much black is a line should be 1 or 2 
	int sum = 0;
	for (int i=0;i<8;i++){
		if (linedata[i]==0){
			sum++;
		}
	}
	if (sum >= amountOfBlack){
		return 1; //Return 1 if enough black is spotted 
	}
	return 0; 
}  

/*
int checkCondition(motiontype *p, odotype *o){
	int d; 
	char go_on=0;
	if (p->condition_type==drivendist){
		d = p->dist - o->traveldist; //distance left
		go_on = (d>0);
		
	}else if(p->condition_type==irdistfrontmiddle){
		// Fill irdistances with meter data
		//irsensor_transformer(odo.irsensor, irdistances);
		go_on = (p->ir_dist) < (irdistances[2]);
	}else if(p->condition_type==crossingblack){

		go_on=!crossingblackline(odo.linesensor);
	}else if(p->condition_type==blackLineFound){
		go_on=!blackLineFound(odo.linesensor,1);
	}
	return go_on; 
}
*/
/*
int gateFound(int index){ 
	printf("lida 0 data %f\n",laserpar[0]);
	//static double nearestGate[2] = {-1.0,-1.0}; 
	static double closeObject = 1000.0;
	double errorMargin = 0.5; 
	//static double last_x = -1.0; 
	//static double last_y = -1.0;

	int maxDist = 1; 
	

	if((closeObject -laserpar[index] )> errorMargin){
		printf("found close object");
		//last_x = sin(degree * index)*laserpar[index]; 
		//last_y = cos(degree * index) / laserpar[index];
		closeObject = laserpar[index]; 
	
	}else if (laserpar[index]< maxDist && (laserpar[index] - closeObject) > errorMargin){ //gap found
			printf("found gate");
			closeObject = 1000.0; 
			//last_x = -1.0; 
			return 1;

	}
	return 0; 
}
*/
int gateFound(int index){ 
	//printf("lida 0 data %f\n",laserpar[0]);
	static double firstobj=0;
	//static double lenght=-1;

	if((firstobj != 0) && laserpar[index] > (firstobj +0.04)){
		//printf("clear first pole %f\n",laserpar[0]);
		firstobj =0;
		return 1;
	}else if(0.6 > laserpar[index] && laserpar[index]> 0){
		firstobj = laserpar[index];
	
		//printf("lida 0 data %f\n",laserpar[0]);
	} 
	return 0; 
}
int lida_dist(int index, double length){
	if(laserpar[index]<= length){
		//printf("reached %f\n",laserpar[index]);
		return 1;
	}
	//printf("not reached %f\n",laserpar[index]);
	return 0;

}


void print_cmd(int state)
{
	char list_of_states[15][50] = {"ms_init",
	"ms_houston",
	"ms_fwd",
	"ms_drive",
	"ms_turn",
	"ms_end",
	"ms_direction_control",
	"ms_followline",
	"ms_hugwall",
	"add_the_state_name_to_print_cmd_function",
	"add_the_state_name_to_print_cmd_function",
	"add_the_state_name_to_print_cmd_function",
	"add_the_state_name_to_print_cmd_function",
	"add_the_state_name_to_print_cmd_function",
	"add_the_state_name_to_print_cmd_function"};
	printf("Current mission: %s\n", list_of_states[state]);
}
