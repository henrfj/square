
//ADD TO LINE 170
int update_command_no();
void cmd_followline(double missions[100][7],int linetype, double speed, int condition,
 double condition_parameter);
void cmd_drive(double missions[100][7], int condition,
    double condition_parameter, double speed);
void cmd_fwd(double missions[100][7], double distance, double speed);
void cmd_turnr(double missions[100][7], double speed, double angle);

//ADD TO LINE 450
//TODO: optional turn, fwd parameters of speed 
    //and if crossing, then dont param_cond
    //compute rad in cmd_turn not in parameter assignment
    //Delete Command() function, declaration, references

//Available functions: 
    // cmd_followline(missions, linetype, speed, condition, cond_param)
    // cmd_drive(missions, speed, condition, cond_param)
    // cmd_fwd(missions, distance, speed(optional))
    // cmd_turn(missions, angle, speed(optional))
//Available conditions  | condition_parameter:
    //drivendist        | distance in meters
	//irdistfrontmiddle | TODO
	//irdistfrontright  | TODO
	//irdistfrontleft   | TODO
	//irdistright_less  | TODO
	//irdistright_more  | TODO
	//irdistleft_less   | TODO
	//irdistleft_more   | TODO
	//crossingblack     | always 0
	//foundBlackLine    | TODO


/////////////////////    MISSIONS    ///////////////////// 
int main(){
    // followline "bm" @v 0.2 : ($crossingblackline==1)
    cmd_followline(missions, bm, 0.2, crossingblack, 0);

    // fwd 0.1 @v0.2
    cmd_fwd(missions, 0.2, 0.1);

    // followline "wm" @v 0.2 : ($crossingblackline==1)
    cmd_followline(missions, wm, 0.2, crossingblack, 0);

    // fwd 0.1 @v0.2
    cmd_fwd(missions, 0.2, 0.1);

    // turnr 0.10 30
    cmd_turn(missions, 0.2, -90*M_PI/180);

}

/////////////////    END OF MISSIONS    /////////////////
//ADD TO LINE  1120
int update_command_no(){
    static int command_no = 0;
    command_no++;
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
		missions[command_no][6] = angle;
}