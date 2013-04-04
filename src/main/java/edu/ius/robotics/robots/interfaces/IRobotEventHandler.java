package edu.ius.robotics.robots.interfaces;

import java.io.ByteArrayOutputStream;

public interface IRobotEventHandler
{
//	public void standardSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void customSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void motorSensorDataReceivedEvent(String robotIP, int robotPort);
//	public void powerSensorReceivedEvent(String robotIP, int robotPort);
	
	public void sensorDataReceivedEvent(String robotIP, int robotPort, int sensorDataType);
	public void audioSegmentReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream audioBuffer);
	public void audioCodecResetRequestReceivedEvent(String robotIP, int robotPort);
	public void imageReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream imageBuffer);
}
