#ifndef ROBOT_H
#define ROBOT_H
#define motion_error_std .007

class robot
{
public:

	double pos[3];//x,y,theta position in real world, dont use these in controller, thats cheating!!
	double motor_error;//value of how motors differ from ideal, dont use these, thats cheating!!
					 
	double color[3]; //robot color output, values 0-1

	//robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int motor_command;

	//function initalized varaibles
	void init(int, int, int);

	//robots internal timer
	int timer;

	double gaussrand();



	//different controllers
	void controller_timestep_gradient();
	void controller_orbit();
	void controller_move_straight();

	//flag set to 1 when robot wants to transmitt
	int tx_request;

	//flag set to 1 when new message received
	int incoming_message_flag;
	int id;
	int hop;
	int previous_distance;

	//communication data struct
	struct communcation_data {
		int message;
		int id;
		double distance;
	};

	//received data goes here
	communcation_data data_in;

	//data to transmitt goes here
	communcation_data data_out;

};
#endif