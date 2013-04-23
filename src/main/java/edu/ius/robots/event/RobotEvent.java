package edu.ius.robots.event;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

import edu.ius.robotics.robots.boards.PMB5010;
import edu.ius.robotics.robots.codecs.ADPCM;
import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;

public class RobotEvent implements Runnable
{
	private boolean isReady;
	private IRobot iRobot;
	private IRobotEventHandler iRobotEventHandler;
	private Object data;
	private RobotEventType robotEventType;
	//private BlockingQueue<RobotEvent> eventQueue;
	//private ExecutorService consumerService;
	private ADPCM adpcm;
	
	private void initRobotEventDispatcher(IRobot iRobot, IRobotEventHandler iRobotEventHandler, RobotEventType robotEventType, Object data)
	{
		this.iRobot = iRobot;
		this.iRobotEventHandler = iRobotEventHandler;
		this.robotEventType = robotEventType;
		this.data = data;
		adpcm = new ADPCM();
		isReady = true;
	}
	
	public RobotEvent(IRobot iRobot, IRobotEventHandler iRobotEventHandler)
	{
		initRobotEventDispatcher(iRobot, iRobotEventHandler, null, null);
	}
	
	public RobotEvent(IRobot iRobot, IRobotEventHandler iRobotEventHandler, RobotEventType robotEventType)
	{
		initRobotEventDispatcher(iRobot, iRobotEventHandler, robotEventType, null);
	}
	
	public RobotEvent(IRobot iRobot, IRobotEventHandler iRobotEventHandler, RobotEventType robotEventType, Object data)
	{
		initRobotEventDispatcher(iRobot, iRobotEventHandler, robotEventType, data);
	}
	
	public void setRobot(IRobot robot)
	{
		this.iRobot = robot;
	}
	
	public IRobot getRobot()
	{
		return iRobot;
	}
	
	public void setRobotEventHandler(IRobotEventHandler iRobotEventHandler)
	{
		this.iRobotEventHandler = iRobotEventHandler;
	}
	
	public IRobotEventHandler getRobotEventHandler()
	{
		return iRobotEventHandler;
	}
	
	public void setData(Object data)
	{
		this.data = data;
	}
	
	public Object getData()
	{
		return data;
	}
	
	public void setRobotEventType(RobotEventType robotEventType)
	{
		this.robotEventType = robotEventType;
	}
	
	public RobotEventType getRobotEventType()
	{
		return robotEventType;
	}
	
	public boolean isReady()
	{
		return isReady;
	}
	
	@Override
	public void run()
	{
		// dequeue and call
		//RobotEvent event = eventQueue.poll();
		//RobotEvent event = null;
		
		isReady = false;
		//RobotEventType eventType = event.getType();
		
		if (RobotEventType.SENSOR_DATA_RECEIVED_EVENT == robotEventType)
		{
			iRobotEventHandler.sensorDataReceivedEvent(iRobot, (Integer) data);
		}
		else if (RobotEventType.IMAGE_DATA_RECEIVED_EVENT == robotEventType)
		{
			// Step 3: Decode complete data from buffer if we have finished receiving.
			////System.err.println("Finished jpeg image assembly");
			byte[] decodedImageData = null;
			//decodedImageData = jpegCodec.decode((ByteArrayOutputStream) data);
			//BufferedImage bufferedImage = ;
			iRobotEventHandler.imageDataReceivedEvent(iRobot, decodedImageData);
		}
		else if (RobotEventType.AUDIO_DATA_RECEIVED_EVENT == robotEventType)
		{
			int audioDataLength = ((ByteArrayOutputStream)data).size();
			ByteBuffer audioByteBuffer = ByteBuffer.wrap(((ByteArrayOutputStream)data).toByteArray());
			//audioByteBuffer.order(PMB5010.BYTE_ORDER);
			short[] decodedAudioData = adpcm.decode(audioByteBuffer.array(), audioDataLength);
			iRobotEventHandler.audioDataReceivedEvent(iRobot, decodedAudioData);
		}
		else if (RobotEventType.AUDIO_CODEC_RESET_EVENT == robotEventType)
		{
			adpcm.init();
		}
		else if (RobotEventType.SHUTDOWN_EVENT == robotEventType)
		{
			iRobotEventHandler.shutdownEvent(iRobot);
		}
		
		isReady = true;
	}
}
