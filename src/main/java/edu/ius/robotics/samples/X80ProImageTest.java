package edu.ius.robotics.samples;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.util.Iterator;

import javax.imageio.ImageIO;
import javax.imageio.ImageWriter;
import javax.imageio.stream.ImageOutputStream;

import edu.ius.robotics.robots.X80Pro;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;

public class X80ProImageTest implements IRobotEventHandler
{
	public static void main(String[] args) throws IOException
	{
		ImageIO.setUseCache(false);
		
		X80ProImageTest imageTest = new X80ProImageTest();
		X80Pro robot = new X80Pro("192.168.0.204", imageTest);
		robot.resetHead();
		
		try
		{
			System.out.println("Sleeping for 1 sec");
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println("Suspending all sensors");
		robot.suspendAllSensors();
		System.out.println("All sensors suspended");
		
		try
		{
			System.out.println("Sleeping for 1 sec");
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Take a photo");
		robot.takePhoto();
		
		try
		{
			System.err.println("Sleeping for 10 sec");
			Thread.sleep(10000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		robot.lowerHead();
	}
	
	@Override
	public void imageReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream imageBuffer)
	{
		System.err.println("*** IMAGE RECEIVED EVENT ***");
		
        BufferedImage bi = new BufferedImage(176, 144, BufferedImage.TYPE_INT_ARGB);
        byte[] rawBytes = imageBuffer.toByteArray();
		int count = 0; 
        for (int h = 0; h < 144; h++)
        {
            for (int w = 0; w < 176; w++)
            {
                bi.setRGB(w, h, rawBytes[count++]);
            }
        }
        try
        {
            File outputFile = new File("/tmp/outputFile.jpeg");
            ImageOutputStream ios = ImageIO.createImageOutputStream(outputFile);
            Iterator<ImageWriter> imageWriters = ImageIO.getImageWritersByFormatName("jpeg");
            ImageWriter imageWriter = (ImageWriter) imageWriters.next();
            imageWriter.setOutput(ios);
            imageWriter.write(bi);
            ios.close();
        }
        catch (IOException ex)
        {
        	ex.printStackTrace();
        }
//		Iterator<?> readers = ImageIO.getImageReadersByFormatName("jpeg");
//		ImageReader reader = (ImageReader) readers.next();
//		ImageInputStream iis = null;
//		try
//		{
//			iis = ImageIO.createImageInputStream(new ByteArrayInputStream(imageBuffer.toByteArray()));
//		}
//		catch (IOException e)
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		
//		if (null == iis)
//		{
//			System.err.println("ImageInputStream (iis) is null!");
//			return;
//		}
//		
//		reader.setInput(iis, true);
//		ImageReadParam param = reader.getDefaultReadParam();
//        Image image = null;
//		try
//		{
//			image = reader.read(0, param);
//		}
//		catch (IOException e)
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		
//		if (null == image)
//		{
//			System.err.println("Image (image) is null!");
//			return;
//		}
//		
//        //got an image file
//        BufferedImage bufferedImage = new BufferedImage(image.getWidth(null), image.getHeight(null), BufferedImage.TYPE_INT_RGB);
		//iRobotImage.imageEvent(robotIP, robotPort, bufferedImage);


        //File imageFile = new File("/tmp/outputImage.bmp");
        //try
		//{
			//ImageIO.write(bufferedImage, "bmp", imageFile);
			//System.err.println("Wrote bmp (outputImage.bmp) to disk");
		//}
		//catch (IOException e)
		//{
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
	}

	@Override
	public void sensorDataReceivedEvent(String robotIP, int robotPort, int sensorDataType)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void audioSegmentReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream audioBuffer)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void audioCodecResetRequestReceivedEvent(String robotIP, int robotPort)
	{
		// TODO Auto-generated method stub
		
	}

}
