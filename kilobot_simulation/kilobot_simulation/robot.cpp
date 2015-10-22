#include <iostream>
#include "robot.h"

void robot::controller_move_straight()
{

	motor_command = 1;
	color[0] = 1;
}
//example controller that has one robot orbit the other
void robot::controller_orbit()
{
	data_out.id = id;//send out my id

					 //if i received a message
	if (incoming_message_flag == 1)
	{
		//clear the message rx flag
		incoming_message_flag = 0;

		//if i have a higher id than my neighbors
		if ((data_in.id)< id)
		{
			//choose motor command based on distance and past distance value
			color[0] = 1;
			if (data_in.distance > 55)
			{

				if (previous_distance>data_in.distance)
				{
					motor_command = 1;
					color[0] = 1;
					color[1] = 0;
					color[2] = 0;
				}
				else
				{
					color[0] = 0;
					color[1] = 1;
					color[2] = 0;
					motor_command = 2;
				}
			}
			else
			{
				if (previous_distance<data_in.distance)
				{
					color[0] = 1;
					color[1] = 0;
					color[2] = 0;
					motor_command = 1;
				}
				else
				{
					color[0] = 0;
					color[1] = 0;
					color[2] = 1;
					motor_command = 3;
				}
			}

		}
		previous_distance = data_in.distance;
	}

	//timer to controll how frequently i tx
	if ((timer % 10) == 0)
	{
		tx_request = 1;


	}
	timer++;
}

//different controller to do hop count (gradient) display
void robot::controller_timestep_gradient()
{
	//only seed has hop 0
	if (hop != 0)
	{
		//getting a message
		if (incoming_message_flag == 1)
		{
			//clear flag
			incoming_message_flag = 0;

			//update hop if rx'ed message is a lower hop
			if ((data_in.message)< hop)
			{

				hop = data_in.message + 1;


			}
		}
	}

	//start sending out my new hop
	data_out.message = hop;




	//update color based on hop
	if ((hop % 6) == 0)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 0;
	}
	else if ((hop % 6) == 1)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 0;
	}
	else if ((hop % 6) == 2)
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1;
	}
	else if ((hop % 6) == 3)
	{
		color[0] = 0;
		color[1] = 1;
		color[2] = 1;
	}
	else if ((hop % 6) == 4)
	{
		color[0] = 1;
		color[1] = 0;
		color[2] = 1;
	}
	else if ((hop % 6) == 5)
	{
		color[0] = 1;
		color[1] = 1;
		color[2] = 0;
	}




	//transmit every 100 timer ticks
	if ((timer % 100) == 0)
	{
		tx_request = 1;



	}




	timer++;

}

void robot::init(int x, int y, int t)
{


	//initalize robot variables
	pos[0] = x;
	pos[1] = y;
	pos[2] = t;
	motor_command = 0;
	timer = rand() / 100;
	incoming_message_flag = 0;
	tx_request = 0;
	id = rand();
	hop = 255;
	rand();
	motor_error = gaussrand()*motion_error_std;
	
	//set the robot at this position to be the seed of the gradient/hop count
	if ((x == 200) && (y == 200))
		hop = 0;



}

double robot::gaussrand()
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}
