package edu.ius.robotics.robots.interfaces;

public interface IRobot
{
	public void sensorEvent(String robotIP, int robotPort, byte[] sensorData, int dataLength);
}
