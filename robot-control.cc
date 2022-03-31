#include <iostream>

#include <libplayerc++/playerc++.h>

#include "Vector2D.hh"

using namespace std;
using namespace PlayerCc;

double computeWallFollow(const RangerProxy & laser){
  for (int ii = 0; ii < laser.GetIntensityCount(); ii++){
    cout << laser[ii] << endl;
  }

  return 1.0;
}

Vector2D AvoidObstacles(double forwardSpeed, double turnSpeed, 
      RangerProxy &sp)
{
      Vector2D ret(forwardSpeed, turnSpeed);
      double v, w;
      //will avoid obstacles closer than 40cm
      double avoidDistance = 1.5;
      //will turn away at 60 degrees/sec
      int avoidTurnSpeed = 30;

      //left corner is sonar no. 2
      //right corner is sonar no. 3
      if(sp[2] < avoidDistance)
      {
            v = 0;
            //turn right
            w = (-1)*avoidTurnSpeed;
            Vector2D leftCorner(v, w);
            return leftCorner;
      }
      else if(sp[3] < avoidDistance)
      {
            v = 0;
            //turn left
            w = avoidTurnSpeed;
            Vector2D rightCorner(v, w);
            return rightCorner;
      }
      else if( (sp[0] < avoidDistance) && \
               (sp[1] < avoidDistance))
      {
            //back off a little bit
            v = -0.2;
            w = avoidTurnSpeed;
            Vector2D stuck(v,w);  
            return stuck;
      }


      return ret;
}

double computeV(const Vector2D & target, const Vector2D & robot)
{
  double v = 0;
  Vector2D d;
  double dMax = 4; 
  double vMax = 0.4;
  
  d = target - robot;
  double dist = d.Length();
  if (dist > dMax)
    v = vMax;
  else
    v = vMax * dist / dMax;
  
  return v;
}

double computeW(const Vector2D & target, const Vector2D & robot, 
		double robotHeading)
{
  double w = 0;
  
  Vector2D d;
  
  d =  target - robot;
  
  double dAngle = d.Angle() - robotHeading;
  if (dAngle > M_PI)
    dAngle -= (2 * M_PI);
  if (dAngle < -M_PI)
    dAngle += (2 * M_PI);
  
  w = 0.8 * dAngle;
  
  return w;
}

bool targetReached(const Vector2D & target, const Vector2D & robot)
{
  bool gotThere = false;

  Vector2D d;
  
  d = target - robot;
  if (d.Length() < 0.4)
    gotThere = true;
  
  return gotThere;
}

int
main(int argn, char *argv[])
{
  PlayerClient robotClient("localhost");

  Position2dProxy base(&robotClient, 0);
  RangerProxy laser(&robotClient, 1);
  // Set the target position here
  Vector2D target(0, 0);
  Vector2D wall;

  // Robot pose, position and heading.
  Vector2D robot;
  double robotHeading;

  // Wait until we get the pose of the robot from the server.
  do {
    robotClient.Read();
  }
  while (!base.IsFresh());
  robot.X( base.GetXPos() );
  robot.Y( base.GetYPos() );
  robotHeading = base.GetYaw();
  
  // Enable robot motors (else the robot will not move).
  base.SetMotorEnable(true);
  while (!targetReached(target, robot))
    {
      robotClient.Read();
      if (base.IsFresh())
      {
        robot.X( base.GetXPos() );
        robot.Y( base.GetYPos() );
        robotHeading = base.GetYaw();

        double x = computeWallFollow(laser);
        
        double v = computeV(target, robot);
        double w = computeW(target, robot, robotHeading);
        wall = AvoidObstacles(v, w, laser);
        base.SetSpeed(wall.Length(), wall.Angle());
      }
    }
  // Disable robot motors.
  base.SetMotorEnable(false);

  return 0;
}
