package edu.ius.robotics.robots.interfaces;

public interface IRobot
{
	public void socketEvent(String robotIP, int robotPort, byte[] sensorData, int length);
	
	public void startProgress();
	public boolean isBusy();
	public int reportProgress();
	public void progressCompletedEvent(String robotIP, int robotPort);
}
