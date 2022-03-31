#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <libplayerc++/playerc++.h>

#include "Vector2D.hh"
#include <map>
#include <string>

using namespace std;
using namespace PlayerCc;

map<string, double> regions;

char state = '0';

void take_action()
{
    double d = 10;

    if (regions["front"] > d && regions["fleft"] > d && regions["fright"] > d)
    {
        // case 0 - nothing
        state = '0';
    }
    else if (regions["front"] < d && regions["fleft"] > d && regions["fright"] > d)
    {
        // case 1 - front
        state = '1';
    }
    else if (regions["front"] > d && regions["fleft"] > d && regions["fright"] < d)
    {
        // case 2 - fright
        state = '2';
    }
    else if (regions["front"] > d && regions["fleft"] < d && regions["fright"] > d)
    {
        // case 3 - fleft
        state = '0';
    }
    else if (regions["front"] < d && regions["fleft"] < d && regions["fright"] > d)
    {
        // case 4 - front and fleft
        state = '1';
    }
    else if (regions["front"] < d && regions["fleft"] > d && regions["fright"] < d)
    {
        // case 5 - front and fright
        state = '1';
    }
    else if (regions["front"] < d && regions["fleft"] < d && regions["fright"] < d)
    {
        // case 7 - front and fleft and fright
        state = '1';
    }
    else if (regions["front"] > d && regions["fleft"] < d && regions["fright"] < d)
    {
        // case 7 - case 8 - fleft and fright
        state = '0';
    }
}

double min_val(RangerProxy& lp, double startingIndex)
{
    double minVal = lp[startingIndex];
	for(int ii = startingIndex + 1; ii < 36; ii++)
	{
		if (lp[ii] < minVal)
		{
            minVal = lp[ii];
		}
	}

    return minVal;
}

void setRegions(RangerProxy& lp)
{
    regions["left"] = min_val(lp, 0);
    regions["fleft"] = min_val(lp, 36);
    regions["front"] = min_val(lp, 72);
    regions["fright"] = min_val(lp, 108);
    regions["right"] = min_val(lp, 144);

    cout << "left: " << regions["left"] << endl;
    cout << "fleft: " << regions["fleft"] << endl;
    cout << "front: " << regions["front"] << endl;
    cout << "fright: " << regions["fright"] << endl;
    cout << "right: " << regions["right"] << endl;

    take_action();
}

void find_wall(double* forwardSpeed, double* turnSpeed)
{
    *forwardSpeed = 0.2;
    *turnSpeed = -0.5;
}

void turn_left(double* forwardSpeed, double* turnSpeed)
{
    *forwardSpeed = 0;
    *turnSpeed = -0.5;
}

void follow_the_wall(double* forwardSpeed, double* turnSpeed)
{
    *forwardSpeed = 0.5;
    *turnSpeed = 0;
}

void Wall(double *forwardSpeed, double *turnSpeed, RangerProxy &lp){

    setRegions(lp);

	switch(state)
	{
	    case '0':
            find_wall(forwardSpeed, turnSpeed);
	        break;
	    case '1':
            turn_left(forwardSpeed, turnSpeed);
	        break;
	    case '2':
            follow_the_wall(forwardSpeed, turnSpeed);
	        break;
	}
	

    /*
	double MIN_DISTANCE = 1.5;
    int maxSpeed = 1;
    double wallLead = M_PI / 2;
    double fourfiveX = lp[45] * cos(45);
    double fourfiveY = lp[45] * sin(45);
    double result = atan2((fourfiveY - MIN_DISTANCE), (fourfiveX + (wallLead - lp[0])));
    *forwardSpeed = 0.5;
    *turnSpeed = result;

    if(abs(lp[90]) > MIN_DISTANCE){
        *forwardSpeed = maxSpeed;
    } else if (abs(lp[0]) == MIN_DISTANCE && abs(lp[90]) > MIN_DISTANCE){
        *forwardSpeed = maxSpeed;
    } else if (abs(lp[0]) == MIN_DISTANCE && abs(lp[90]) == MIN_DISTANCE){
        *forwardSpeed = 0;
        *turnSpeed = (-1)*avoidTurnSpeed;
    } else if (abs(lp[0]) > MIN_DISTANCE && abs(lp[90]) > MIN_DISTANCE){
        *forwardSpeed = 0;
        *turnSpeed = avoidTurnSpeed;
    } else if (abs(lp[0]) > MIN_DISTANCE && abs(lp[90]) == MIN_DISTANCE){
        *forwardSpeed = 0;
        *turnSpeed = (-1)*avoidTurnSpeed;
    }

	*/
}

int main(int argn, char *argv[])
{
    PlayerClient robotClient("localhost");
    Position2dProxy p2dProxy(&robotClient,0);
    RangerProxy laserProxy(&robotClient,1);

    double forwardSpeed, turnSpeed;

    while(true){
        robotClient.Read();
        Wall(&forwardSpeed, &turnSpeed, laserProxy);
        p2dProxy.SetSpeed(forwardSpeed, turnSpeed);
    }
    return 0;
}