/**
 * @file    MyRobot.h
 * @brief   A simple example for computing the odometry while the robot moves straight
 *
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @author  Jesús García Martínez <jesusgar@ing.uc3m.es>
 * @date    2023-12
 */

#include "MyRobot.h"

 /**
  * @brief Main program.
  */
int main(int argc, char** argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}

