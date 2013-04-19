//package edu.ius.robotics.samples;
//
//import java.io.ByteArrayOutputStream;
//import java.io.IOException;
//
//import edu.ius.robotics.robots.interfaces.IRobotEventHandler;
//import edu.ius.robotics.robots.x80pro.X80Pro;
//
//public class X80ProManeuverCompletedTest implements IRobotEventHandler
//{
//	public static void main(String[] args) {
//		X80ProManeuverCompletedTest robotTest = new X80ProManeuverCompletedTest();
//		X80Pro robot = null;
//		try {
//			robot = new X80Pro("192.168.0.202", robotTest);
//		}
//		catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		if (null == robot) {
//			return;
//		}
//		// old way
//		robot.runStepPWM(1.0, 1000);
//		try {
//			Thread.sleep(1000);
//		}
//		catch (InterruptedException ex) {
//			ex.printStackTrace();
//		}
//		// new way
//		// SI units, 1.0 meters in 1.0 seconds (with this hardware configuration? yeah! right.)
//		// Course correction could be applied to runStepToCompletion.
//		//robot.runStepWithProgressReport(1.0, 1.0);
//		while (robot.isBusy()) {
//			try {
//				Thread.sleep(100);
//				robot.reportProgress();
//			}
//			catch (InterruptedException ex) {
//				ex.printStackTrace();
//			}
//		}
//	}
//	
//	@Override
//	public void sensorDataReceivedEvent(String robotIP, int robotPort, int sensorDataType)
//	{
//		// TODO Auto-generated method stub
//		
//	}
//
//	@Override
//	public void imageDataReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream imageBuffer)
//	{
//		// TODO Auto-generated method stub
//		
//	}
//
//	@Override
//	public void audioDataReceivedEvent(String robotIP, int robotPort, short[] audioData)
//	{
//		// TODO Auto-generated method stub
//		
//	}
//}
