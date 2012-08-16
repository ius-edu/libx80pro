package edu.ius.robotics;

public class PMB5010
{
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
		
		for (int i = 0 ; i < z; ++i) 
		{
		    v = (byte) (buf[2 + i]); // start from RID
		    
		    // for each bit
		    for (int j = 0; j < 8; ++j) 
		    {
				// isolate least sign bit
				data_bit = (byte) ((v & 0x01) & 0xff);
				sr_lsb = (byte) ((shift_reg & 0x01) & 0xff);
				// calculate the feed back bit
				fb_bit = (byte) (((data_bit ^ sr_lsb) & 0x01) & 0xff);
				shift_reg = (byte) ((shift_reg & 0xff) >>> 1);
				
				if (fb_bit == 1)
				{
				    shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xff);
				}
				
				v = (byte) ((v & 0xff) >>> 1);
		    }
		}
		
		return shift_reg;
    }    
}
