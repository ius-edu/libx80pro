#include <stdio.h>
#include <unistd.h>
#include "robots/X80Pro.h"

int main()
{
	X80Pro robot;
	
	X80Pro_init(&robot);
	robot.connect(&robot, "192.168.0.203");

	printf("reset head\n");
	robot.resetHead();

	/* move robot forward */
	printf("move robot forward\n");
	robot.resumeBothDCMotors(&robot);
	
	/* move robot */
	printf("move robot\n");
	robot.setBothDCMotorVelocities(&robot, 3000, 3000);
	
	/* hold move for 3 seconds */
	printf("hold the move for 3 seconds\n");
	sleep(3);
	
	/* halt robot */
	printf("halt robot\n");
	robot.suspendBothDCMotors(&robot);
	
	return 0;
}
