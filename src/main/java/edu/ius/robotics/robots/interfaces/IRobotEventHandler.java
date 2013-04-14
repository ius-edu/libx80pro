package edu.ius.robotics.robots.interfaces;

import java.io.ByteArrayOutputStream;

public interface IRobotEventHandler
{
//	public void standardSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void customSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void motorSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void powerSensorReceivedEvent(String robotIP, int robotPort);
	
	public void sensorDataReceivedEvent(String robotIP, int robotPort, int sensorDataType);
	public void audioDataReceivedEvent(String robotIP, int robotPort, short[] audioBuffer);
	public void imageDataReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream imageBuffer);
}
