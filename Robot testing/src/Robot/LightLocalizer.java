package Robot;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**This class accomplishes the task of Localization using the lightsensor and gridlines.
 * </p>It is responsible for using lightsensor readings to detect grid lines and then using the odometer, determine at what heading these lines get detected.
 * </p>Most of the variables are private. It uses many public variables from Main.
 * The methods are private and non-static.
 * @author Thomas
 *
 */
public class LightLocalizer {
	/**
	 * The Odometer object that we use to access x, y, and theta values.
	 */
	private Odometer odo;
	/**
	 * Buffer sampleProvider that can fetch samples from the light sensor.
	 */
	private SampleProvider colorSensor;
	/**
	 * Buffer array to store data from the light sensor.
	 */
	private float[] colorData;
	/**
	 * Stores the amount of lines detected.
	 */
	private int numberOfLines;
	/**
	 * An array to store position data from odometer
	 */
	double [] pos = new double [3];
	/**
	 * Stores the absolute x coordinate relative to the board.
	 */
	private double x; //values to compute
	/**
	 * Stores the absolute y coordinate relative to the board.
	 */
	private double y;
	/**
	 * Stores the absolute heading relative to the board.
	 */
	private double theta;
	/**
	 * Radius of wheels. This is set in Main.
	 */
	public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;		//its actually between 2.15 and 2.10 but by trials it seems like 2.1 works perfectly 
	/**
	 * Track or width of the robot. This is set in Main.
	 */
	public static final double TRACK = Main.TRACK;			//by measurement, the distance between two wheels
	/**
	 * Distance of the lightsensor from the middle of the track of the robot. This is set in Main.
	 */
	private double LS_DIST = Main.LS_DIST;	//by measurement, the distance between ls and the center of track
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor leftMotor = Main.leftMotor;	
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	/**
	 * Stores the rotation speed of the motors when we rotate the robot (in deg/s).
	 */
	public static double ROTATION_SPEED = 50;	//rotation speed
	/**
	 * Stores the rotation speed of the motors when we move the robot forwards (in deg/s).
	 */
	public static double forwardspeed=100;	
	/**
	 * Stores the amount we sleep the LightLocalizer thread each cycle.
	 */
	private static final int sleepperiod=0;	//i first have this value like 10 or something
	//later by testing i found its better leave it 0 thus no sleeping time 
	/** 
	 * The starting corner at which the robot is placed
	 */
	private int startingCorner;
	
	/**
	 * Constructor for LightLocalizer class. Takes no inputs but inherits many variables from Main.
	 */
	public LightLocalizer() 
	{
		/*this.odo = Main.odometer;
		this.colorSensor = Main.colorValue;
		this.colorData = Main.colorData;
		this.startingCorner = Main.startingCorner;*/
	}
	/**
	 * The method does light localization. The angles (as reported by the odometer) at which we detect the black lines are used to figure out an absolute heading. 0 is in the positive x axis.
	 * </p>As well, light localization can determine absolute x and y relative to the board.
	 */
	public void doLocalization() 
	{
		Navigator navi=new Navigator();//thus i can use method in navigation.java class
		navi.start();
		moveToLocalizingSpot(navi);
		
		double angle[]=new double[4];	//declare an array to store the angles 
		numberOfLines=0;	//counter
		int LSvalue = 0;
		int bufferCount = 0;
		boolean blackLineDetected = false;
		while (numberOfLines<=3) //this loop detects 4 gridlines 
		{	
			LSvalue = getFilteredData();
			
			if(LSvalue < 50 && bufferCount < 5)
			{
				bufferCount++;
				blackLineDetected = false;
			}
			else if(LSvalue < 50)
			{
				Sound.beep();
				angle[numberOfLines] = odo.getTheta();
				blackLineDetected = true;
			}
			if(LSvalue >= 50)
			{
				if(blackLineDetected)
				{
					  angle[numberOfLines] = (angle[numberOfLines] + odo.getTheta())/2;
					  numberOfLines++;
				}
				bufferCount = 0;
				blackLineDetected = false;
			}
			rotateCounterClockwise();
			try{Thread.sleep(sleepperiod);}catch(Exception e){}
		}
		// start rotating and clock all 4 gridlines
		
		leftMotor.stop();
		rightMotor.stop();
		
		double temp=0;
		
		temp=(angle[1]-angle[3])/2;
		y=-LS_DIST*Math.cos(temp);
		
		temp=(angle[0]-angle[2])/2;
		x=-LS_DIST*Math.cos(temp);
		
		theta = angle[2] + (angle[0]-angle[2])/2 - Math.PI;
		System.out.println("Adustment theta: " + theta);
		theta = angle[3] + theta;
		odo.setX(x);
		odo.setY(y);
		odo.setTheta(theta);
		for(int i = 0 ; i < angle.length; i++)
		{
			System.out.println("Angle " + i + " : " + angle[i]);
		}
		System.out.println("Set x to: " + x);
		System.out.println("Set y to: " + y);
		System.out.println("Set theta to: " + theta);
		
		navi.turnTo(0);
		
		navi.travelTo(0,0);
		while(navi.isNavigating())
		{
			try {Thread.sleep(500);} 
			catch (InterruptedException e) {e.printStackTrace();}
		}
		navi.turnTo(0);
		leftMotor.stop();
		rightMotor.stop();
		switch (startingCorner) {
		case 1: odo.setX(0); odo.setY(0); break;
		case 2: odo.setX(300); odo.setY(0); break;
		case 3: odo.setX(300); odo.setY(300); break;
		case 4: odo.setX(0); odo.setY(300); break;
		}
		// when done travel to the nearest intersection
	}
	/**Gets the most recent color sensor value. Scales it by 100 and returns that value
	 * @return Color sensor value from 0-100. Higher values mean more light (lighter).
	 */
	private int getFilteredData() 
	{
		colorSensor.fetchSample(colorData,0);      		//get light sensor Red value 
		int LSvalue=  (int)((colorData[0])*100);		// times 100 into 0~100 scale,easier to test 
		return LSvalue;
	}
	/**Move to the localizing spot using navigator. The localizing spot is 11cm along the 45 degree line.
	 * @param navi The navigator object that will be used to turn the robot.
	 */
	private void moveToLocalizingSpot(Navigator navi) 
	{
		double heading = 0;
		switch (this.startingCorner) {
		case 1: heading = 45; break;
		case 2: heading = 135; break;
		case 3: heading = 225; break;
		case 4: heading = 315; break;
		}
		navi.turnTo(heading * Math.PI/180.0);	//turn to heading
		odo.setTheta(heading * Math.PI/180.0);
		//set current position as 0,0 and 315degree
		leftMotor.setSpeed((int)forwardspeed);
		rightMotor.setSpeed((int)forwardspeed);
		//moves 11 cm
		leftMotor.rotate(convertDistance(WHEEL_RADIUS,11),true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS,11),false);
		//by trial, this distance is enough for the robot to do ls localization
		leftMotor.stop();
		rightMotor.stop();
	}

	/** Takes an angle (in radians) and returns the same angle but in degrees.
	 * @param angle Angle in radians to be converted
	 * @return The same angle in degrees
	 */
	private double toDegrees(double angle) 
	{
		return 57.2958 * angle;
	}
	/**
	 * Rotates the robot counter-clockwise. The rotation speed of the wheels are ROTATION_SPEED (in deg/s).
	 */
	private void rotateCounterClockwise() {
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.backward();
		rightMotor.forward();
	}
	/**Converts linear distance that you want a wheel to rotate into a value that stores the rotations (in degrees) that the wheel needs to rotate.
	 * @param radius The radius of the wheel.
	 * @param distance The distance you want a wheel to rotate.
	 * @return An int that stores the rotation (in degrees) that a wheel needs to rotate.
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}
