#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include "robot.h"
using namespace std;

#define delay 0 //delay between time steps, use if program is too fast
#define  windowWidth 500 //display window
#define  windowHeight 500 //display window
#define comm_range 100 //communication range between robots
#define num_robots 2 //number of robots running
#define comm_noise_std 5 //standard dev. of sensor noise
#define PI 3.14159265358979324
#define radius 20 //radius of a robot
#define p_control_execute .99 // probability of a controller executing its time step


// Global vars.
double time_sim;  //simulation time
float zoom, view_x, view_y; //var. for zoom and scroll


robot robots[num_robots];//creates an array of robots

						 //generates a gaussian random variable, used for noise
double gaussrand()
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

//check to see if motion causes robots to collide
int find_collisions(int id, double x, double y)
{

	int i;
	for (i = 0;i < num_robots;i++)
	{
		if (i != id)
		{
			double distance = sqrt((x - robots[i].pos[0])*(x - robots[i].pos[0]) + (y - robots[i].pos[1])*(y - robots[i].pos[1]));
			if (distance < 2 * radius)
			{
				return 1;
			}

		}

	}
	return 0;

}

// Drawing routine.
void drawScene(void)
{


	int i, j;

	float forward_motion_step = 1;//motion step size
	double rotation_step = .05;//motion step size

							   //run a step of most or all robot controllers
	for (i = 0;i < num_robots;i++)
	{

		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX))
		{
			//robots[i].controller_timestep_gradient();
			robots[i].controller_orbit();
			//robots[i].controller_move_straight();
		}

	}

	//choose random order to have robots communicate and bump
	int selected[num_robots];
	int order[num_robots];
	for (int i = 0;i < num_robots;i++)
	{
		selected[i] = 0;
		order[i] = 0;
	}

	//randomly choose the order of robots by putting their index into the order array

	for (j = 0;j < num_robots;j++)
	{
		int index_search = rand() % num_robots;
		int index_found = 0;
		while (index_found == 0)
		{
			if (selected[index_search] == 0)
			{
				selected[index_search] = 1;
				index_found = 1;

			}
			index_search++;
			if (index_search == num_robots)
			{
				index_search = 0;
			}

		}
		order[j] = index_search;
	}



	//let robots communicate
	for (i = 0;i < num_robots;i++)
	{

		//if robot wants to communicate, send message to all robots within distance comm_range
		if (robots[order[i]].tx_request == 1)
		{
			robots[order[i]].tx_request = 0;//clear transmission flag
			for (j = 0;j < num_robots;j++)
			{
				if (j != order[i])
				{
					double distance = sqrt((robots[order[i]].pos[0] - robots[j].pos[0])*(robots[order[i]].pos[0] - robots[j].pos[0]) + (robots[order[i]].pos[1] - robots[j].pos[1])*(robots[order[i]].pos[1] - robots[j].pos[1]));
					if (distance < comm_range)//robot within com range, put transmitting robots data in its data_in struct
					{
						robots[j].incoming_message_flag = 1;
						robots[j].data_in = robots[order[i]].data_out;
						robots[j].data_in.distance = distance + gaussrand()*comm_noise_std;

					}
				}
			}
		}
	}

	//move robots
	for (i = 0;i < num_robots;i++)
	{
		if (robots[order[i]].motor_command == 1)//move forward
		{
			if (find_collisions(order[i], robots[order[i]].pos[0] + forward_motion_step*cos(robots[order[i]].pos[2]), robots[order[i]].pos[1] + forward_motion_step*sin(robots[order[i]].pos[2])) == 0)
			{

				robots[order[i]].pos[0] += forward_motion_step*cos(robots[order[i]].pos[2]) ;
				robots[order[i]].pos[1] += forward_motion_step*sin(robots[order[i]].pos[2]) ;
				robots[order[i]].pos[2] += robots[order[i]].motor_error;
			}

		}
		else if (robots[order[i]].motor_command == 2)//turn cw
		{
			double temp_theta = rotation_step + robots[order[i]].pos[2];
			double temp_x = forward_motion_step*cos(temp_theta) + robots[order[i]].pos[0];
			double temp_y = forward_motion_step*sin(temp_theta) + robots[order[i]].pos[1];
			if (find_collisions(order[i], temp_x, temp_y) == 0)
			{
				robots[order[i]].pos[2] = temp_theta;
				robots[order[i]].pos[0] = temp_x;
				robots[order[i]].pos[1] = temp_y;
			}
			else
			{
				robots[order[i]].pos[2] = temp_theta;
			}


		}
		else if (robots[order[i]].motor_command == 3)//turn ccw
		{

			double temp_theta = robots[order[i]].pos[2] - rotation_step;
			double temp_x = forward_motion_step*cos(temp_theta) + robots[order[i]].pos[0];
			double temp_y = forward_motion_step*sin(temp_theta) + robots[order[i]].pos[1];
			if (find_collisions(order[i], temp_x, temp_y) == 0)
			{
				robots[order[i]].pos[2] = temp_theta;
				robots[order[i]].pos[0] = temp_x;
				robots[order[i]].pos[1] = temp_y;
			}
			else
			{
				robots[order[i]].pos[2] = temp_theta;
			}


		}
		else //stop motors
		{


		}

	}

	//draw robots
	int triangleAmount = 300;
	GLfloat twicePi = 2.0f * PI;
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	for (int j = 0;j < num_robots;j++)
	{

		glColor4f(robots[j].color[0], robots[j].color[1], robots[j].color[2], 1.0);
		for (i = 0; i <= triangleAmount; i++)
		{

			glVertex2f(robots[j].pos[0], robots[j].pos[1]);
			glVertex2f(robots[j].pos[0] + (radius * cos(i * twicePi / triangleAmount)), robots[j].pos[1] + (radius * sin(i * twicePi / triangleAmount)));
		}
		glBegin(GL_LINES);
		glColor4f(0, 0, 0, 1.0);
		glVertex2f(robots[j].pos[0], robots[j].pos[1]);
		glVertex2f(robots[j].pos[0] + cos(robots[j].pos[2])*radius, robots[j].pos[1] + sin(robots[j].pos[2])*radius);



	}

	glEnd();
	cout << time_sim << endl;
	time_sim = time_sim + 1;
	glFlush();
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

// Initialization routine.
void setup(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
	glClearColor(1.0, 1.0, 1.0, 0.0);
}

// OpenGL window reshape routine.
void resize(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Keyboard input processing routine.
void keyInput(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'w'://up
		view_y += 100;
		break;
	case 'a'://up
		view_x -= 100;
		break;
	case 's'://up
		view_y -= 100;
		break;
	case 'd'://up
		view_x += 100;
		break;
	case '-':
		zoom = zoom*1.1;
		break;
	case '+':
		zoom = zoom*0.9;

		break;
	default:
		break;
	}
}

void OnIdle(void) {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-zoom + view_x, view_x, -zoom + view_y, view_y, 0.0f, 1.0f);
	//sleep(delay);
	glutPostRedisplay();
}

//setup function for orbit behavior, make sure num_robots is 2
void setup_positions_orbit()
{
	int k = 0;
	for (int i = 0;i < 2;i++)
	{
		robots[k].init(200 + 45 * i, 200 + 45 * i, 10 * rand() / RAND_MAX);
		k++;

	}
}

//setup for gradient fuction, needs num_robots to be 100
void setup_positions_gradient()
{
	int k = 0;
	for (int i = 0;i < 10;i++)
	{
		for (int j = 0;j < 10;j++)
		{
			robots[k].init(200 + 45 * i, 200 + 45 * j, 10 * rand() / RAND_MAX);
			k++;
		}
	}
}

// Main routine.
int main(int argc, char **argv)
{
	//seed random variable for different random behavior every time
	srand(time(NULL));

	//set the simulation time to 0
	time_sim = 0;

	//inital zoom and scroll positions
	zoom = 700;
	view_x = 500;
	view_y = 500;

	//place robots
	//setup_positions_gradient();
	setup_positions_orbit();

	//do some open gl stuff


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(0, 0);

	glutCreateWindow("Kilobot simulator");
	setup();
	glutDisplayFunc(drawScene);
	glutReshapeFunc(resize);
	glutIdleFunc(OnIdle);
	glutKeyboardFunc(keyInput);
	glutMainLoop();

	return 0;
}
