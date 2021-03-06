/*
 * OdometryDisplay.java
 */

package Navigation;

import lejos.hardware.lcd.TextLCD;

public class OdometryDisplay extends Thread {
	private static final long DISPLAY_PERIOD = 50;
	private Odometer odometer;
	private TextLCD t;
	private double thetaDisplay;

	public OdometryDisplay(Odometer odometer, TextLCD t) {
		this.odometer = odometer;
		this.t = t;
	}

	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		t.clear();

		while (true) {
			displayStart = System.currentTimeMillis();

			t.drawString("X:              ", 0, 0);
			t.drawString("Y:              ", 0, 1);
			t.drawString("T:              ", 0, 2);

			odometer.getPosition(position, new boolean[] { true, true, true });

			for (int i = 0; i < 3; i++) {
				if (i == 2) //just displays theta in terms of degrees fromm 0 to 360
				{
					if(position[i] > 2*Math.PI)
					{
						thetaDisplay = position[i] - 2*Math.PI;
					}
					else if(position[i] < 0)
					{
						thetaDisplay = position[i] + 2*Math.PI;
					}
					else
					{
						thetaDisplay = position[i];
					}
					t.drawString(formattedDoubleToString(thetaDisplay*57.2598, 2), 3, i); //draw theta in degrees
				}
				else //draws x and y
				{
					t.drawString(formattedDoubleToString(position[i], 2), 3, i);
				}
			}

			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) { //if run takes > display_period to complete, do another cycle, if not (perferred), try..
				try 
				{
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart)); //..to sleep till the next cycle (display_period from display end)
				} catch (InterruptedException e) 
				{
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}
	
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0) //if negative, take the base
				t = -t; 
			
			while (t > 0) { //take the base of number and get its decimal?
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}
			
			result += stack;
		}
		
		// put the decimal, if needed
		if (places > 0) {
			result += ".";
		
			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}

}
