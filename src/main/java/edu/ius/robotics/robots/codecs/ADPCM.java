/*
 * This is a Java port of the original Intel/DVI ADPCM coder/decoder.
 */

/***********************************************************
Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
Netherlands.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Stichting Mathematisch
Centrum or CWI not be used in advertising or publicity pertaining to
distribution of the software without specific, written prior permission.

STICHTING MATHEMATISCH CENTRUM DISCLAIMS ALL WARRANTIES WITH REGARD TO
THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS, IN NO EVENT SHALL STICHTING MATHEMATISCH CENTRUM BE LIABLE
FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

******************************************************************/

/*
** Intel/DVI ADPCM coder/decoder.
**
** The algorithm for this coder was taken from the IMA Compatability Project
** proceedings, Vol 2, Number 2; May 1992.
**
** Version 1.2, 18-Dec-92.
**
** Change log:
** - Fixed a stupid bug, where the delta was computed as
**   stepsize*code/4 in stead of stepsize*(code+0.5)/4.
** - There was an off-by-one error causing it to pick
**   an incorrect delta once in a blue moon.
** - The NODIVMUL define has been removed. Computations are now always done
**   using shifts, adds and subtracts. It turned out that, because the standard
**   is defined using shift/add/subtract, you needed bits of fixup code
**   (because the div/mul simulation using shift/add/sub made some rounding
**   errors that real div/mul don't make) and all together the resultant code
**   ran slower than just using the shifts all the time.
** - Changed some of the variable names to be more meaningful.
*/

package edu.ius.robotics.robots.codecs;

import java.nio.ByteBuffer;

public class ADPCM
{
	public static int[] indexTable = 
	{
		-1, -1, -1, -1, 2, 4, 6, 8, 
		-1, -1, -1, -1, 2, 4, 6, 8
	};
	
	public static int[] stepsizeTable = 
	{
		7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 
		19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 
		50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 
		130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 
		337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 
		876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 
		2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 
		5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 
		15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
	};
	
	public static class ADPCMState 
	{
		/* ignoring the convention to wrap these with getters and setters */
		public short previousOutput; /* previous output value */
		public char index; /* index into step size table */
	}
	
	ADPCMState adpcmState;
	
	public void init()
	{
		adpcmState.previousOutput = 0;
		adpcmState.index = 0;
	}
	
	public ADPCM()
	{
		adpcmState = new ADPCMState();
		adpcmState.previousOutput = 0;
		adpcmState.index = 0;
	}
	
	public short[] decode(ByteBuffer input, int length)
	{
		int outputLength = (length & 0xFF)*(Short.SIZE >> 3);
		short[] output = new short[outputLength];
		short predictedOutput = adpcmState.previousOutput;	/* Predicted output value */
		char index = adpcmState.index;						/* Current step change index */
		int step = stepsizeTable[index];					/* Stepsize */
		int predictedOutputDelta;							/* Current change to valpred */
		int sign;											/* Current adpcm sign bit */
		int delta;											/* Current adpcm output value */
		int outputIndex = 0;								/* Current index in output buffer */
		int inputIndex = 0;									/* Current index in input buffer */
		boolean bufferStep = false;							/* toggle between outputbuffer/output */
		
		for (int i = output.length - 1; 0 <= i; i--)
		{
			//System.err.println("i: " + i);
			/* Step 1: Get the data value */
			//System.err.println("step 1: get the data value");
			if (bufferStep) delta = (input.getInt(inputIndex) & 0x0F);
			else delta = (((input.getInt(++inputIndex) & 0x0F) >> 4) & 0xFF);
			bufferStep = !bufferStep;
			
			/* Step 2: Find new index value (for later) */
			//System.err.println("step 2: find new index value (for later)");
			index += indexTable[delta];
			if (index < 0) index = 0;
			else if (88 < index) index = 88;
			
			/* Step 3: Separate sign and magnitude */
			//System.err.println("step 3: separate sign and magnitude");
			sign = delta & 0x08;
			delta = delta & 0x07;
			
			/* Step 4: Combine difference and new predicted value */
			/*
			 * Computes predictedOutputDelta = (delta + 0.5)*step/4, but see comment
			 * in encodeADPCM();
			 */
			//System.err.println("step 4: combine difference and new predicted value");
			predictedOutputDelta = ((step & 0xFF) >> 3);
			if (0 < (delta & 0x04)) predictedOutputDelta += step;
			if (0 < (delta & 0x02)) predictedOutputDelta += step >> 1;
			if (0 < (delta & 0x01))	predictedOutputDelta += step >> 2;
			
			if (0 < sign) predictedOutput -= predictedOutputDelta;
			else predictedOutput += predictedOutputDelta;
			
			/* Step 5: Clamp output value */
			//System.err.println("step 5: clamp output value");
			if (32767 < predictedOutput) predictedOutput = 32767;
			else if (predictedOutput < -32768) predictedOutput = -32768;
			
			/* Step 6: Update step value */
			//System.err.println("step 6: update step value");
			step = stepsizeTable[index];
			
			/* Step 7: Output value */
			//System.err.println("step 7: output value");
			output[outputIndex] = predictedOutput;
			
			//System.err.println("*** increment outputIndex ***");
			++outputIndex;
			
			//System.err.println();
		}
		adpcmState.previousOutput = predictedOutput;
		adpcmState.index = index;
		return output;
	}
	
	public byte[] encode(short[] input, int length)
	{
		int outputLength = (length & 0xFFFF)*(Short.SIZE >> 3);
		byte[] output = new byte[outputLength];
		short predictedOutput = adpcmState.previousOutput;
		char index = adpcmState.index;
		int step = stepsizeTable[index];					
		int outputHolder = 0;								/* place to keep previous 4-bit value */
		
		boolean bufferStep = true;
		int inputIndex = 0;
		int outputIndex = 0;
		for (int i = outputLength; 0 < i; --i)
		{
			short inputValue = input[inputIndex++];
			
			/* Step 1: Compute difference with previous value */
			int diff = inputValue - predictedOutput;
			int sign = (diff < 0) ? 8 : 0;
			if (0 < sign) diff = -diff;
			
			/* Step 2: Divide and clamp */
			/* Note
			 * This code *approximately* computes:
			 * 	delta = diff*4/step;
			 * 	predictedOutputDelta = (0.5*delta)*step/4
			 * but in shift step bits are dropped. The net results of this is
			 * that even if you have fast mul/div hardware you cannot put it to
			 * good use since the fixup would be too expensive.
			 */
			int delta = 0;
			int predictedOutputDelta = step >> 3;
			
			if (step <= diff)
			{
				delta = 4;
				diff -= step;
				predictedOutputDelta += step;
			}
			
			step >>= 1;
			if (step <= diff)
			{
				delta |= 2;
				diff -= step;
				predictedOutputDelta += step;
			}
			
			step >>= 1;
			if (step <= diff)
			{
				delta |= 1;
				predictedOutputDelta += step;
			}
			
			/* Step 3: Update previous value */
			if (0 < sign) predictedOutput -= predictedOutputDelta;
			else predictedOutput += predictedOutputDelta;
			
			/* Step 4: Clamp previous value to 16 bits */
			if (32767 < predictedOutput) predictedOutput = 32767;
			else if (predictedOutput < -32768) predictedOutput = -32768;
			
			/* Step 5: Assemble value, update index and step values */
			delta |= sign;
			
			index += indexTable[delta];
			if (index < 0) index = 0;
			else if (88 < index) index = 88;
			
			/* Step 6: Output value */
			if (bufferStep) outputHolder = (byte) ((delta << 4) & 0xF0);
			else output[outputIndex] = (byte) (((delta & 0x0F) | outputHolder) & 0xFF);
			
			bufferStep = !bufferStep;
			outputIndex++;
		}
		
		/* Output last step, if needed */
		if (!bufferStep) output[outputIndex] = (byte) (outputHolder & 0xFF);
		
		adpcmState.previousOutput = predictedOutput;
		adpcmState.index = index;
		return output;
	}
}
