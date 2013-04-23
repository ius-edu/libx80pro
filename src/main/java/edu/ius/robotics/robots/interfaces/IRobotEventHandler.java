package edu.ius.robotics.robots.interfaces;

public interface IRobotEventHandler
{
//	public void standardSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void customSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void motorSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void powerSensorReceivedEvent(String robotIP, int robotPort);
	
	public void startupEvent(IRobot sender);
	public void shutdownEvent(IRobot sender);
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType);
	public void audioDataReceivedEvent(IRobot sender, short[] audioData);
	public void imageDataReceivedEvent(IRobot sender, byte[] imageData);
}
