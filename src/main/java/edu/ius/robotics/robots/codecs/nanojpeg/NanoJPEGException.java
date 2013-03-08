package edu.ius.robotics.robots.codecs.nanojpeg;

import edu.ius.robotics.robots.codecs.nanojpeg.NanoJPEGDecodeResult;

public class NanoJPEGException extends Exception
{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	NanoJPEGDecodeResult decodeResult;
	
	public NanoJPEGException(NanoJPEGDecodeResult decodeResult)
	{
		this.decodeResult = decodeResult;
	}
}
