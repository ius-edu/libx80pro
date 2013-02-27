package edu.ius.robotics.robots.codecs;

import edu.ius.robotics.robots.codecs.DecodeResult;

public class JPEGDecoderException extends Exception
{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	DecodeResult decodeResult;
	
	public JPEGDecoderException(DecodeResult decodeResult)
	{
		this.decodeResult = decodeResult;
	}
}
