package edu.ius.robotics.robots.boards;

/* The following is taken pretty much directly from the 
 * PMB5010 serial protocol documentation
 */

public class PMB5010
{
	/* Packet Info */
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int DID_OFFSET = 4;
	public static final int PAYLOAD_OFFSET = 6;
	public static final int LENGTH_OFFSET = 5;
	public static final int SEQ_OFFSET = 3;
	public static final int RESERVED_OFFSET = 3;
	public static final int VIDEO_SEQ_OFFSET = 0;
	public static final int VIDEO_LENGTH_OFFSET = 1;
	public static final int METADATA_SIZE = 9;
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 94; // 0x5e
	public static final byte STX1 = 2; // 0x02
	public static final byte ETX0 = 94; // 0x5e
	public static final byte ETX1 = 13; // 0x0d
	
	/* RID */
	public static final byte RID_HOST = 0x00;
	public static final byte RID_PMB5010 = 0x08;
	
	/* RESERVED */
	public static final byte RESERVED = 0x00;
	
	/* FLAGS */
	public static final byte FLAG_VALUE = 0x06;
	public static final byte SEQ_BEGIN = 0x00;
	public static final byte SEQ_TERMINATE = (byte) (0xff & 0xff);
	
	/* DID Listing */
	public static final byte RX_PING = 0x00;
	public static final byte TX_ACK = 0x01;
	public static final byte RX_ACK = 0x01;
	public static final byte VIDEO_PACKAGE = 0x09;
	public static final byte AUDIO_PACKAGE = 0x0A;
	public static final byte START_AUDIO_RECORDING = 0x0B;
	public static final byte STOP_AUDIO_RECORDING = 0x0C;
	public static final byte START_AUDIO_PLAYBACK = 0x0D;
	public static final byte STOP_AUDIO_PLAYBACK = 0x0E;
	public static final byte AUDIO_SLOW_DOWN = 0x10;
	public static final byte TAKE_PHOTO = 0x20;
	public static final byte ADPCM_RESET = 0x33;
	public static final byte TX_PING = (byte) (0xFF & 0xff);
	
    /*
     * calcCRC method comes directly from PMB5010 Protocol documentation.
     */
    public static byte calcCRC(byte[] buf) 
    {
		byte shift_reg, sr_lsb, data_bit, v;
		byte fb_bit;
		int z;
		shift_reg = 0; // initialize the shift register
		z = buf.length - 5;
		
		for (int i = 0; i < z; ++i) 
		{
		    v = (byte) (buf[2 + i]); // start from RID
		    
		    // for each bit
		    for (int j = 0; j < 8; ++j) 
		    {
				// isolate least sign bit
				data_bit = (byte) ((v & 0x01) & 0xFF);
				sr_lsb = (byte) ((shift_reg & 0x01) & 0xFF);
				// calculate the feed back bit
				fb_bit = (byte) (((data_bit ^ sr_lsb) & 0x01) & 0xFF);
				shift_reg = (byte) ((shift_reg & 0xFF) >>> 1);
				
				if (fb_bit == 1)
				{
				    shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xFF);
				}
				
				v = (byte) ((v & 0xFF) >>> 1);
		    }
		}
		
		return shift_reg;
    }
    
    public static byte[] ping(byte seq)
    {
    	byte[] msg = new byte[10];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = (byte) (seq & 0xff);
    	msg[4] = (byte) (TX_PING & 0xff);
    	msg[5] = 1;
    	msg[6] = 0;
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] ack(byte seq)
    {
    	byte[] msg = new byte[10];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = (byte) (seq & 0xff);
    	msg[4] = TX_ACK;
    	msg[5] = 1;
    	msg[6] = 1;
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] startAudioRecording(short voiceSegmentLength)
    {
    	byte[] msg = new byte[10];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = START_AUDIO_RECORDING;
    	msg[5] = 1; // len
    	msg[6] = (byte) (voiceSegmentLength & 0xff);
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] stopAudioRecording()
    {
    	byte[] msg = new byte[9];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = STOP_AUDIO_RECORDING;
    	msg[5] = 0;
    	msg[6] = calcCRC(msg);
    	msg[7] = ETX0;
    	msg[8] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] startAudioPlayback(short sampleLength, byte seq)
    {
    	byte[] msg = new byte[10];
    	
    	int length = 4;
    	
    	if (sampleLength < 16000)
    	{
    		length = 1;
    	}
    	else if (sampleLength < 16000*20)
    	{
    		length = 2;
    	}
    	else if (sampleLength < 16000*32)
    	{
    		length = 3;
    	}
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = (byte) (seq & 0xff);
    	msg[4] = START_AUDIO_PLAYBACK;
    	msg[5] = 1;
    	msg[6] = (byte) (length & 0xff);
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] stopAudioPlayback()
    {
    	byte[] msg = new byte[9];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = STOP_AUDIO_PLAYBACK;
    	msg[5] = 0;
    	msg[6] = calcCRC(msg);
    	msg[7] = ETX0;
    	msg[8] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] continueAudioPlayback(byte[] encodedAudioSample)
    {
    	int z = encodedAudioSample.length;
    	
    	byte[] msg = new byte[8+z];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED; // Not SEQ?
    	msg[4] = AUDIO_PACKAGE;
    	msg[5] = (byte) (z & 0xff);
    	
    	for (int i = 0; i < z; ++i)
    	{
    		msg[6+i] = encodedAudioSample[i];
    	}
    	
    	msg[6+z] = calcCRC(msg);
    	msg[7+z] = ETX0;
    	msg[8+z] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] takePhoto()
    {
    	byte[] msg = new byte[9];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = TAKE_PHOTO;
    	msg[5] = 0; // len
    	msg[6] = calcCRC(msg);
    	msg[7] = ETX0;
    	msg[8] = ETX1;
    	
    	return msg;
    }
}
