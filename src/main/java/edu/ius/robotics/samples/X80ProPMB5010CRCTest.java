package edu.ius.robotics.samples;

import edu.ius.robotics.robots.boards.PMB5010;

public class X80ProPMB5010CRCTest
{
	public static void main(String[] args)
	{
		byte[] msg = PMB5010.takePhoto();
		System.out.printf("%x\n", (0xFF & msg[6]));
	}
}
