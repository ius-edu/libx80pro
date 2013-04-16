package edu.ius.robotics.robots.boards;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/* The following is taken pretty much directly from the 
 * PMB5010 serial protocol documentation
 */

public class PMB5010
{
	public static final ByteOrder BYTE_ORDER = ByteOrder.LITTLE_ENDIAN; 
	
	/* Packet Info */
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int METADATA_SIZE = 9;
	public static final int DID_OFFSET = 4;
	public static final int PAYLOAD_OFFSET = 6;
	public static final int LENGTH_OFFSET = 5;
	public static final int SEQ_OFFSET = 3;
	public static final int RESERVED_OFFSET = 3;
	public static final int VIDEO_SEQ_OFFSET = 0;
	public static final int VIDEO_LENGTH_OFFSET = 1;
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 0x5E; // 94
	public static final byte STX1 = 0x02; // 2
	public static final byte ETX0 = 0x5E; // 94
	public static final byte ETX1 = 0x13; // 13
	
	/* RID */
	public static final byte RID_HOST = 0x00;
	public static final byte RID_PMB5010 = 0x08;
	
	/* RESERVED */
	public static final byte RESERVED = 0x00;
	
	/* FLAGS */
	public static final byte FLAG_VALUE = 0x06;
	public static final int SEQ_BEGIN = 0x00;
	public static final int SEQ_TERMINATE = 0xFF;
	
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
	public static final byte TX_PING = (byte) (0xFF & 0xFF);
	
    /*
     * calcCRC method comes directly from PMB5010 Protocol documentation.
     */
    public static byte checksum(byte[] buf) 
    {
		int shift_reg, sr_lsb, data_bit, v;
		int fb_bit;
		int z;
		shift_reg = 0; // initialize the shift register
		z = buf.length - 5;
		for (int i = 0; i < z; ++i) 
		{
		    v = buf[2 + i]; // start from RID
		    // for each bit
		    for (int j = 0; j < 8; ++j) 
		    {
				// isolate least sign bit
				data_bit = ((v & 0x01) & 0xFF);
				sr_lsb = ((shift_reg & 0x01) & 0xFF);
				// calculate the feed back bit
				fb_bit = (((data_bit ^ sr_lsb) & 0x01) & 0xFF);
				shift_reg = ((shift_reg & 0xFF) >> 1);
				if (fb_bit == 1)
				{
				    shift_reg = ((shift_reg ^ 0x8C) & 0xFF);
				}
				v = ((v & 0xFF) >> 1);
		    }
		}
		return (byte) shift_reg;
    }
    
    public static int ping(byte[] buffer, int seq)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = (Byte.SIZE >> 3);
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put((byte) seq).put(TX_PING).put((byte) dataLength);
    	msg.put((byte) 0); // ping
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int ack(byte[] buffer, int seq)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = (Byte.SIZE >> 3);
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put((byte) seq).put(TX_ACK).put((byte) dataLength);
    	msg.put((byte) 1); // ack
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int startAudioRecording(byte[] buffer, byte voiceSegmentLength)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = (Byte.SIZE >> 3);
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put(RESERVED).put(START_AUDIO_RECORDING).put((byte) dataLength);
    	msg.put(voiceSegmentLength); // voiceSegmentLength * 255
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int stopAudioRecording(byte[] buffer)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = 0;
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put(RESERVED).put(STOP_AUDIO_RECORDING).put((byte) dataLength);
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int startAudioPlayback(byte[] buffer, short sampleLength, byte seq)
    {
    	byte abbreviatedSampleLength = 4;
    	
    	if (sampleLength < 16000)
    	{
    		abbreviatedSampleLength = 1;
    	}
    	else if (sampleLength < 16000*20)
    	{
    		abbreviatedSampleLength = 2;
    	}
    	else if (sampleLength < 16000*32)
    	{
    		abbreviatedSampleLength = 3;
    	}
    	
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	int dataLength = (Byte.SIZE >> 3);
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put((byte) seq).put(START_AUDIO_PLAYBACK).put((byte) dataLength);
    	msg.put(abbreviatedSampleLength);
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int stopAudioPlayback(byte[] buffer)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = 0;
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put(RESERVED).put(STOP_AUDIO_PLAYBACK).put((byte) dataLength);
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int continueAudioPlayback(byte[] buffer, byte[] encodedAudioSample)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	msg.order(PMB5010.BYTE_ORDER);
    	int dataLength = encodedAudioSample.length;
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put(RESERVED).put(AUDIO_PACKAGE).put((byte) dataLength);
    	for (int i = 0, z = encodedAudioSample.length; i < z; ++i) msg.put(encodedAudioSample[i]);
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
    
    public static int takePhoto(byte[] buffer)
    {
    	ByteBuffer msg = ByteBuffer.wrap(buffer);
    	int dataLength = 0;
    	msg.put(STX0).put(STX1).put(RID_PMB5010).put(RESERVED).put(TAKE_PHOTO).put((byte) dataLength);
    	msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
    	return dataLength + METADATA_SIZE;
    }
}
