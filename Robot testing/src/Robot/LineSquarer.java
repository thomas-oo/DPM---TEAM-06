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
	
	private static final int LSDist = 5;
	private int low = 20;
	private int high = 50;
	
	public LineSquarer()
	{
		this.odo = Main.odometer;
	}
	
	public void squareWithLines()
	{
		Navigator navi=new Navigator();//thus i can use method in navigation.java class
		navi.start();
		
		driveToVerticalLine();
		
		navi.turnTo(0.5*Math.PI);
		
		driveToHorizontalLine();
		
		navi.travelTo(0,0);
		while(navi.isNavigating())
		{
			try {Thread.sleep(500);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
		navi.turnTo(0);
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
		
		//both color must find the black line
		while(!(leftReading < 50 && rightReading < 50))
		{
			if(leftReading < 50)
			{
				System.out.println("Found vertical line on left.");
				leftMotor.stop();
				
				rightMotor.setSpeed(low);
				rightMotor.forward();
			}
			else if(rightReading < 50)
			{
				System.out.println("Found vertical line on right.");
				rightMotor.stop();
				
				leftMotor.setSpeed(low);
				leftMotor.forward();
			}
			else
			{
				System.out.println("No vertical line found.");
				leftMotor.setSpeed(high);
				rightMotor.setSpeed(high);
				leftMotor.forward();
				rightMotor.forward();
			}
			leftReading = getLeftColorData();
			rightReading = getRightColorData();
		}
		System.out.println("Found vertical line on both.");
		
		findMiddleOfLine();
		System.out.println("Found middle of line. ");
		//this is assuming lower left field
		odo.setTheta(0);
		odo.setX(0-LSDist);
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
		
		//both color must find the black line
		while(!(leftReading < 50 && rightReading < 50))
		{
			if(leftReading < 50)
			{
				System.out.println("Found horizontal line on left.");
				leftMotor.stop();
				
				rightMotor.setSpeed(low);
				rightMotor.forward();
			}
			else if(rightReading < 50)
			{
				System.out.println("Found horizontal line on right.");
				rightMotor.stop();
				
				leftMotor.setSpeed(low);
				leftMotor.forward();
			}
			else
			{
				System.out.println("No horizontal line found.");
				leftMotor.setSpeed(high);
				rightMotor.setSpeed(high);
				leftMotor.forward();
				rightMotor.forward();
			}
			leftReading = getLeftColorData();
			rightReading = getRightColorData();
		}
		System.out.println("Found horizontal line on both.");
		findMiddleOfLine();
		System.out.println("Found middle of line.");
		//this is assuming lower left field
		odo.setTheta(0.5*Math.PI);
		odo.setY(0-LSDist);
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
		while(leftReading < 50)
		{
			leftMotor.forward();
			leftReading = getLeftColorData();
		}
		leftMotor.stop();
		leftTacho = leftMotor.getTachoCount() - leftTacho;
		leftMotor.rotate(-leftTacho/2);
		
		int rightTacho = rightMotor.getTachoCount();
		int rightReading = getRightColorData();
		while(rightReading < 50)
		{
			rightMotor.forward();
			rightReading = getRightColorData();
		}
		rightMotor.stop();
		rightTacho = rightMotor.getTachoCount() - rightTacho;
		rightMotor.rotate(-rightTacho/2);
	}
}
