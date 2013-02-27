package edu.ius.robotics.robots.interfaces;

public interface IRobotVideo
{
	public void videoEvent(String robotIP, int robotPort, byte[] videoData);
}
