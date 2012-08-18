package edu.ius.robotics;

public class X80Test
{
	public static final int NO_CTRL = -32768;
	public static final int SERVO0_INI = 3650;
	public static final int SERVO1_INI = 3300;
	
	public static void main(String[] args)
	{
		X80 robot = new X80("192.168.0.203", 10001);
		
		robot.servoNonTimeCtrlAll(SERVO0_INI, SERVO1_INI, NO_CTRL, NO_CTRL, NO_CTRL, NO_CTRL);
		//APLite aplite = new APLite(robot);
		
		//aplite.resetHead();
	}
}
