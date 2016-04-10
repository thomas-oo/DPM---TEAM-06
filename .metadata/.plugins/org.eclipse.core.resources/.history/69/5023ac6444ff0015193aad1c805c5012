package Robot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**This class is responsible for accurate localization of the robot via color sensors. This is achieved by finding horizontal
 * and vertical lines in the starting corner using two color sensors. Once found, the robot will square up with the horizontal
 * line and move to what it thinks is 0,0. It will then set the odometer's x and y to 0, and theta to 0. If you are starting the robot in
 * a different starting corner than the lower left one, you will have to externally set x and y and theta values accordingly as
 * this class will not do so. This class uses the same classes from Main (odometer, and navigator classes) as well as many other variables from Main.
 * 
 * @author Thomas
 *
 */
public class LineSquarer 
{
	/**
	 * The Odometer object that we use to access x, y, and theta values.
	 */
	private Odometer odo;
	
	/**
	 * The Navigator object that we use to go to move to coordinates and turn.
	 */
	private Navigator nav;
	/**
	 * SampleProvider for the left color sensor.
	 */
	private SampleProvider leftColorSensor = Main.leftColorValue;
	/**
	 * Stores the left color sensor's reading.
	 */
	private float[] leftColorData = Main.leftColorData;
	
	/**
	 * SampleProvider for the right color sensor.
	 */
	private SampleProvider rightColorSensor = Main.rightColorValue;
	/**
	 * Stores the right color sensor's reading.
	 */
	private float[] rightColorData = Main.rightColorData;
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor leftMotor = Main.leftMotor;	
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	
	/**
	 * Distance from color sensors to middle of track
	 */
	private static final double LSDist = Main.LS_DIST;
	/**
	 * Low speed that is used to slowly turn the robot a certain way so that it can detect the line.
	 */
	private int low = 20;
	/**
	 * High speed that is used to move the robot to the lines.
	 */
	private int high = 100;
	
	/**
	 * This class takes no constructors but uses the Main's odometer and navigation classes in here.
	 */
	public LineSquarer()
	{
		this.odo = Main.odometer;
		this.nav = Main.nav;
	}
	
	/**
	 * Method that does the localization. It is working on the assumption that the robot is approximately heading towards a vertical line as
	 * viewed from the starting corner. This method will set x and y and theta to 0,0,0 respectively once done. It is up to further code to determine
	 * where it truly is and heading depending on its starting corner.
	 */
	public void squareWithLines()
	{
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

	/**
	 * Method that drives the robot to a vertical line. Once one of the color sensors detect a black line, that side's wheel motor stops and
	 * the other motor goes slowly so that the robot rotates slowly. Then once the other color sensor also detects a black line, we set X
	 * (accounting for the distance between the light sensors and the middle of the track of the robot). We also set
	 * theta to be 0.
	 */
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
	
	/**
	 * Method that drives the robot to a horizontal line. Once one of the color sensors detect a black line, that side's wheel motor stops and
	 * the other motor goes slowly so that the robot rotates slowly. Then once the other color sensor also detects a black line, we set Y
	 * (accounting for the distance between the light sensors and the middle of the track of the robot). We also set
	 * theta to be half pi. 
	 */
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

	/**Method that fetches a current reading of the left color sensor. The reading is from 0 to 100 where a smaller value
	 * represents a darker light value.
	 * @return LSvalue : The color sensor value read on the left color sensor.
	 */
	private int getLeftColorData() 
	{
		leftColorSensor.fetchSample(leftColorData,0);      		//get light sensor Red value 
		int LSvalue=  (int)((leftColorData[0])*100);		// times 100 into 0~100 scale,easier to test 
		return LSvalue;
	}
	/**Method that fetches a current reading of the right color sensor. The reading is from 0 to 100 where a smaller value
	 * represents a darker light value.
	 * @return LSvalue : The color sensor value read on the right color sensor.
	 */
	private int getRightColorData()
	{
		rightColorSensor.fetchSample(rightColorData,0);
		int LSvalue = (int)((rightColorData[0])*100);
		return LSvalue;
	}
	
	/**
	 * Method to find the middle of the detected line. This method will rotate the left motor until the left light sensor
	 * no longer detects the black line and rotates backwards accordingly. The right motor also rotates until the right
	 * light sensor no longer detects the black line and rotates backwards accordingly. The end result is the robot is 
	 * squarely on the vertical line and facing 0 degrees. Then, theta can be set to 0.
	 */
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
