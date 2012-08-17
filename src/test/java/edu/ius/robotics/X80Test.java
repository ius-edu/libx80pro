package edu.ius.robotics;

public class X80Test
{
	public static void main(String[] args)
	{
		X80 robot = new X80("192.168.0.203", 10001);
		APLite aplite = new APLite(robot);
		
		aplite.resetHead();
	}
}
