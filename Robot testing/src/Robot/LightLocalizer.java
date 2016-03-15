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
	 * Constructor for LightLocalizer class. Takes no inputs but inherits many variables from Main.
	 */
	public LightLocalizer() 
	{
		this.odo = Main.odometer;
		this.colorSensor = Main.usValue;
		this.colorData = Main.colorData;
	}
	/**
	 * The method does light localization. The angles (as reported by the odometer) at which we detect the black lines are used to figure out an absolute heading. 0 is in the positive x axis.
	 * </p>As well, light localization can determine absolute x and y relative to the board.
	 */
	public void doLocalization() 
	{
		Navigator navi=new Navigator();//thus i can use method in navigation.java class
		moveToLocalizingSpot(navi);
		
		double angle[]=new double[4];	//declare an array to store the angles 
		numberOfLines=0;	//counter
		while (numberOfLines<=3) //this loop detects 4 gridlines 
		{	
			int LSvalue = getFilteredData();
			odo.getPosition(pos);	//get current position from odometer
			if (LSvalue<=50)	//the floor is something above 70, when it first sees a black line, is 60~50, so i set 50 to make the robot stops quicker. Note that when the ls is exactly above the black line, the lsvalue is less then 15
			{
				Sound.twoBeeps(); 
				angle[numberOfLines]=toDegrees(pos[2]);	//store current angle
				numberOfLines++;	//counter increments
				if (numberOfLines==4)	//if count to 4,then all 4 gridlines are detected, break the loop
				{
					leftMotor.stop();
					rightMotor.stop();
					Sound.beep();
					break; 
				}
			}
			rotateCounterClockwise(); //rotate the robot counter-clockwise
			try { Thread.sleep(sleepperiod); } catch(Exception e){}	
		}
		// start rotating and clock all 4 gridlines
		
		
		double temp=0;
		temp=360-angle[1]+angle[3];
		y=-LS_DIST*Math.cos(Math.PI*temp/360);
		temp=Math.abs(angle[0]-angle[2]);
		x=-LS_DIST*Math.cos(Math.PI*temp/360);
		theta=temp/2+90;
		odo.getPosition(pos);
		theta=theta+pos[2];
		if (theta>=360)
		{
			theta=theta % 360;
		}
		if (theta<0)
		{	
			theta=360+theta;
		}
		odo.setPosition(new double [] {x, y, theta});	
		Sound.buzz();
		
		navi.travelTo(0,0);
		navi.turnTo(0);
		leftMotor.stop();
		rightMotor.stop();
		odo.setPosition(new double [] {0, 0, 0});
		// when done travel to (0,0) and turn to 0 degrees
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
		navi.turnTo(0.25 * Math.PI);	//turn to 45 degree
		odo.setTheta(0.25*Math.PI);
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
