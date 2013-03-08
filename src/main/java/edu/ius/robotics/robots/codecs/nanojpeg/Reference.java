/** 
 * http://stackoverflow.com/questions/7884581/how-can-i-simulate-pass-by-reference-in-java
 * This was posted by Mark Peters
 */

package edu.ius.robotics.robots.codecs.nanojpeg;

public class Reference<T> 
{
    private T referent;

    public Reference()
    {
    	referent = null;
    }
    
    public Reference(T initialValue) 
    {
       referent = initialValue;
    }

    public void set(T newVal) 
    {
       referent = newVal;
    }

    public T get() 
    {
       return referent;
    }
}
