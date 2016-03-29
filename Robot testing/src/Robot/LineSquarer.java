package Robot;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LineSquarer 
{
	/**
	 * The Odometer object that we use to access x, y, and theta values.
	 */
	private Odometer odo;

	private SampleProvider leftColorSensor = Main.leftColorValue;
	private float[] leftColorData = Main.leftColorData;
	
	private SampleProvider rightColorSensor = Main.rightColorValue;
	private float[] rightColorData = Main.rightColorData;
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor leftMotor = Main.leftMotor;	
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	
	private static final double LSDist = Main.LS_DIST;
	private int low = 20;
	private int high = 100;
	
	public LineSquarer()
	{
		this.odo = Main.odometer;
	}
	
	public void squareWithLines()
	{
		Navigator nav = Main.nav;//thus i can use method in navigation.java class
		
		driveToVerticalLine();
		
		nav.turnTo(0.5*Math.PI);
		
		driveToHorizontalLine();
		
		try {
			Thread.sleep(50);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		nav.travelTo(-LSDist,0);
		while(nav.isNavigating())
		{
			try {Thread.sleep(500);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
		nav.turnTo(0);
		findMiddleOfLine();
		nav.travelTo(0, 0);
		while(nav.isNavigating())
		{
			try {Thread.sleep(500);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
	}

	private void driveToVerticalLine() 
	{
		leftMotor.setSpeed(high);
		rightMotor.setSpeed(high);
		leftMotor.forward();
		rightMotor.forward();
		int leftReading = 0;
		int rightReading = 0;
		leftReading = getLeftColorData();
		rightReading = getRightColorData();
		
		int bufferLeft = 0;
		int bufferRight = 0;
		boolean lineLeft = false;
		boolean lineRight = false;
		
		//both color must find the black line
		while(!(lineLeft && lineRight))
		{
			leftReading = getLeftColorData();
			rightReading = getRightColorData();
			
			if(leftReading < 30 && bufferLeft < 1)
			{
				bufferLeft++;
			}
			else if(leftReading < 30)
			{
				lineLeft = true;
				if(rightReading < 30)
				{
					lineRight = true;
				}
				System.out.println("Found vertical line on left.");
			}
			else
			{
				bufferLeft = 0;
			}
			
			if(rightReading < 30 && bufferRight < 1)
			{
				bufferRight++;
			}
			else if(rightReading < 30)
			{
				lineRight = true;
				if(leftReading < 30)
				{
					lineLeft = true;
				}
				System.out.println("Found vertical line on right.");
			}
			else
			{
				bufferRight = 0;
			}
			
			if(lineLeft && !lineRight)
			{
				leftMotor.stop(true);
				
				rightMotor.setSpeed(low);
				rightMotor.forward();
			}
			else if(lineRight && !lineLeft)
			{
				rightMotor.stop(true);
				
				leftMotor.setSpeed(low);
				leftMotor.forward();
			}
			else if(lineRight&&lineLeft)
			{
				rightMotor.stop(true);
				leftMotor.stop(false);
			}
			
			try {Thread.sleep(10);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
		System.out.println("Found vertical line on both.");
		
		//findMiddleOfLine();
		//System.out.println("Found middle of line. ");
		//this is assuming lower left field
		odo.setTheta(0);
		odo.setX(-LSDist);
	}
	
	private void driveToHorizontalLine() 
	{
		leftMotor.setSpeed(high);
		rightMotor.setSpeed(high);
		leftMotor.forward();
		rightMotor.forward();
		int leftReading = 0;
		int rightReading = 0;
		leftReading = getLeftColorData();
		rightReading = getRightColorData();
		
		int bufferLeft = 0;
		int bufferRight = 0;
		boolean lineLeft = false;
		boolean lineRight = false;
		
		//both color must find the black line
		while(!(lineLeft && lineRight))
		{
			leftReading = getLeftColorData();
			rightReading = getRightColorData();
			
			if(leftReading < 30 && bufferLeft < 1)
			{
				bufferLeft++;
			}
			else if(leftReading < 30)
			{
				lineLeft = true;
				if(rightReading < 30)
				{
					lineRight = true;
				}
				System.out.println("Found vertical line on left.");
			}
			else
			{
				bufferLeft = 0;
			}
			
			if(rightReading < 30 && bufferRight < 1)
			{
				bufferRight++;
			}
			else if(rightReading < 30)
			{
				lineRight = true;
				if(leftReading < 30)
				{
					lineLeft = true;
				}
				System.out.println("Found vertical line on right.");
			}
			else
			{
				bufferRight = 0;
			}
			
			if(lineLeft && !lineRight)
			{
				leftMotor.stop(true);
				
				rightMotor.setSpeed(low);
				rightMotor.forward();
			}
			else if(lineRight && !lineLeft)
			{
				rightMotor.stop(true);
				
				leftMotor.setSpeed(low);
				leftMotor.forward();
			}
			else if(lineRight&&lineLeft)
			{
				rightMotor.stop(true);
				leftMotor.stop(false);
			}
			
			try {Thread.sleep(10);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
		System.out.println("Found horizontal line on both.");
		//findMiddleOfLine();
		//System.out.println("Found middle of line.");
		//this is assuming lower left field
		odo.setTheta(0.5*Math.PI);
		odo.setY(-LSDist);
	}

	private int getLeftColorData() 
	{
		leftColorSensor.fetchSample(leftColorData,0);      		//get light sensor Red value 
		int LSvalue=  (int)((leftColorData[0])*100);		// times 100 into 0~100 scale,easier to test 
		return LSvalue;
	}
	
	private int getRightColorData()
	{
		rightColorSensor.fetchSample(rightColorData,0);
		int LSvalue = (int)((rightColorData[0])*100);
		return LSvalue;
	}
	
	private void findMiddleOfLine()
	{
		leftMotor.setSpeed(low);
		rightMotor.setSpeed(low);
		
		int leftTacho = leftMotor.getTachoCount();
		int leftReading = getLeftColorData();
		while(leftReading < 30)
		{
			leftMotor.forward();
			try {Thread.sleep(50);} 
			catch (InterruptedException e) {e.printStackTrace();}
			leftReading = getLeftColorData();
		}
		leftMotor.stop();
		leftTacho = leftMotor.getTachoCount() - leftTacho;
		leftMotor.rotate(-leftTacho/2);
		
		int rightTacho = rightMotor.getTachoCount();
		int rightReading = getRightColorData();
		while(rightReading < 30)
		{
			rightMotor.forward();
			try {Thread.sleep(50);} 
			catch (InterruptedException e) {e.printStackTrace();}
			rightReading = getRightColorData();
		}
		rightMotor.stop();
		rightTacho = rightMotor.getTachoCount() - rightTacho;
		rightMotor.rotate(-rightTacho/2);
		odo.setTheta(0);
	}
	
	
}
