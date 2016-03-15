package Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class accomplishes the task of Localization using an ultrasonic sensor.
 * </p>It is responsible for using ultrasonic readings to determine where absolute 0 degrees is relative to the board.
 * </p>Most of the variables are private. It uses many public variables from Main.
 * The methods are private and non-static.
 * @author Thomas
 */
public class USLocalizer
{
	/**
	 * The rotation speed of the motors (in deg/s) for localization.
	 */
	public static double ROTATION_SPEED = 60;
	/**
	 * The Odometer object that we use to access x, y, and theta values.
	 */
	private Odometer odo;
	/**
	 * Buffer sampleProvider that can fetch samples from the ultrasonic sensor.
	 */
	private SampleProvider usSensor;
	/**
	 * Buffer array to store data from the ultrasonic sensor (in m).
	 */
	private float[] usData;
	/**
	 * Radius of wheels. This is set in Main.
	 */
	public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
	/**
	 * Track or width of the robot. This is set in Main.
	 */
	public static final double TRACK = Main.TRACK;
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private static final EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	/**
	 * Stores the ultrasonic reading which we will say is the distance to the nearest wall (in cm).
	 */
	private double distanceToWall;
	/**
	 * Stores a distance where we say that we see a wall (in cm). This is a final (constant) variable.
	 */
	private static final int WALL_DISTANCE=50;
	/**
	 * Sleep period of the localization loop. This is a final (constant) variable.
	 */
	private static final int sleepperiod=0;

	/**
	 * An array to store position data from odometer
	 */
	private double [] pos = new double [3];	//declare an array to store the odometer value
	/**
	 * The angle at which we find the first wall when we do not see a wall and then rotate counterclockwise (in radians).
	 */
	private double angleA;
	/**
	 * The angle at which we find the second wall (in radians).
	 */
	private double angleB;

	/**
	 * Constructor for USLocalizer class. Takes no inputs but inherits many variables from Main.
	 */
	public USLocalizer() 
	{
		this.odo = Main.odometer;
		this.usSensor = Main.usValue;
		this.usData = Main.usData;
	}

	/**
	 * The method does falling edge localization. The angles (as reported by the odometer) at which we detect the walls are used to figure out an absolute heading. 0 is in the positive x axis.
	 */
	public void doLocalization()
	{	
		Navigator navi = new Navigator();
		while (true) 	//this loop makes the robot keeps rotating counter-clockwise until it sees no wall
		{
			distanceToWall=getFilteredData();	//get the us data
			if (distanceToWall>=WALL_DISTANCE)	//if detects no wall 
			{
				Sound.beep();					
				break;					//break the while loop
			} 
			else
			{
				rotateCounterClockwise();		
			}
			try { Thread.sleep(sleepperiod); } catch(Exception e){}		// run and sleep for a while
		}
		// rotate the robot until it sees no wall

		turn(70);
		//sometimes the robot imediately detects the wall,so i turn the robot counter-clockwise an amount 
		//of degrees so it will face to no wall and then start the below method, which will detects wall 

		while (true) 	//this loop makes the robot keeps rotating counter-clockwise until it sees a wall
		{
			distanceToWall=getFilteredData();//get the us data
			if (distanceToWall<WALL_DISTANCE)//if detects a wall
			{
				odo.getPosition(pos);	//get odometer data
				angleA=pos[2];	//store the current angle as angleA
				Sound.beep();	
				leftMotor.stop();	//stop the motor
				rightMotor.stop();	//stop the motor
				break;			//exit the while loop
			} else
			{
				rotateCounterClockwise();
			}
			try { Thread.sleep(sleepperiod); } catch(Exception e){}		
		}
		// keep rotating until the robot sees a wall, then latch the angle
		while (true) //this loop makes the robot keeps rotating clockwise until it sees no wall
		{
			distanceToWall=getFilteredData();//get the us data
			if (distanceToWall>=WALL_DISTANCE)	//if detects no wall
			{
				Sound.beep();
				break;
			} else
			{
				rotateClockwise();		
				//rotate the robot clockwise
			}
			try { Thread.sleep(sleepperiod); } catch(Exception e){}		// Poor man's timed sampling
		}
		// switch direction and wait until it sees no wall
		while (true)//this loop makes the robot keeps rotating clockwise until it sees a wall
		{
			distanceToWall=getFilteredData();	//get us data
			if (distanceToWall<WALL_DISTANCE)	//if detects a wall
			{
				odo.getPosition(pos);	//get odometer data
				angleB=pos[2];	//store current angle as angleB
				Sound.beep();
				leftMotor.stop(); //stop both motors
				rightMotor.stop();
				break; //break the while loop
			} else
			{
				rotateClockwise();
			}
			try { Thread.sleep(sleepperiod); } catch(Exception e){}		// Poor man's timed sampling
		}
		// keep rotating until the robot sees a wall, then latch the angle
		odo.setPosition(new double [] {0.0, 0.0, (225 + ((angleB-angleA)/2))});
		navi.turnTo(0);
	}
	/**
	 * Rotates the robot clockwise. The rotation speed of the wheels are ROTATION_SPEED (in deg/s).
	 */
	private void rotateClockwise() {
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.forward();
		rightMotor.backward();
	}
	/**Turns the robot a certain degrees counterclockwise.
	 * @param angle Counter-clockwise angle that the robot will turn.
	 */
	private void turn(double angle)
	{
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.rotate(convertAngle(-WHEEL_RADIUS, TRACK, angle), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, angle), false);
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
	/**Converts a desired change in robot's heading (in radians) into how much linear distance a wheel needs to travel (in cm).
	 * @param radius The radius of the wheel (in cm).
	 * @param width The track or width of the robot (in cm).
	 * @param angle The angle you want the robot to turn (in degrees).
	 * @return The linear distance that a wheel would have to rotate.
	 */
	private static int convertAngle(double radius, double width, double angle) 
	{
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**Converts linear distance that you want a wheel to rotate into a value that stores the rotations (in degrees) that the wheel needs to rotate.
	 * @param radius The radius of the wheel.
	 * @param distance The distance you want a wheel to rotate.
	 * @return An int that stores the rotation (in degrees) that a wheel needs to rotate.
	 */
	private static int convertDistance(double radius, double distance)
	{
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**Gets the most recent ultrasonic sensor value.
	 * @return A float that stores the ultrasonic reading (in cm). If the value is more than WALL_DISTANCE, set it to WALL_DISTANCE (ceiling filter).
	 */
	private float getFilteredData() 	
	{
		usSensor.fetchSample(usData, 0);
		float distance = usData[0];
		distance=(float) (distance *100.0);
		if(distance > WALL_DISTANCE)
		{
			distance = WALL_DISTANCE;	
		}
		return distance;
	}



}


