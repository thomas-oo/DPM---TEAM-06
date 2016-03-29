package Robot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**This class handles accomplishes the task of Odometry, that is, it is responsible for updating the x, y, and heading values of the robot relative to the board.
 * The methods in this class are all private and non static. The variables used in this navigator are private but this class uses a few public variables from the main method.
 * 0 degrees is in the positive x-axis and heading increases counterclockwise.
 * @author Thomas
 *
 */
public class Odometer extends Thread {
	/**
	 * Stores the x coordinate of the robot (in cm).
	 */
	private double x;
	/**
	 * Stores the y coordinate of the robot (in cm).
	 */
	private double y;
	/**
	 * Stores the heading of the robot (in radians). The heading is 0-2pi.
	 */
	private double theta;
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private EV3LargeRegulatedMotor rightMotor;
	/**
	 * Stores the tachoreading of the left motor from the previous cycle (in degrees).
	 */
	private int oldTachoLeft;
	/**
	 * Stores the tachoreading of the right motor from the previous cycle (in degrees).
	 */
	private int oldTachoRight;
	/**
	 * Stores the most recent tachoreading of the left motor (in degrees).
	 */
	private int nowTachoLeft;
	/**
	 * Stores the most recent tachoreading of the right motor (in degrees).
	 */
	private int nowTachoRight;
	
	
	/**
	 * Stores the change in displacement of the left wheel between the current cycle and the previous cycle (in cm).
	 */
	private double displacementLeft;
	/**
	 * Stores the change in displacement of the right wheel between the current cycle and the previous cycle (in cm).
	 */
	private double displacementRight;
	/**
	 * Stores the change in displacement of the robot between the current cycle and the previous cycle (in cm). 
	 * </p>The robot's position is defined to be measured from the middle of the track of the robot.
	 */
	private double displacementRobot;
	/**
	 * Stores the change in displacement of the right wheel RELATIVE to the left wheel between the current cycle and the previous cycle (in cm).
	 * </p> We approximate this value as the arclength the right wheel traces out relative to the left wheel between the current and previous cycles.
	 */
	private double displacementArc;
	/**
	 * Radius of wheels. This is set in Main.
	 */
	private double WHEEL_RADIUS = Main.WHEEL_RADIUS;
	/**
	 * Track or width of the robot. This is set in Main.
	 */
	private double TRACK = Main.TRACK;

	/**
	 * How long we sleep the odometer thread for (in ms).
	 */
	private static final long ODOMETER_PERIOD = 25;

	/**
	 * An object that is used for synchronizing purposes.
	 */
	private Object lock;

	/**
	 * Constructor for Odometer class. Takes no inputs but inherits the motors from Main. Initializes many values to 0 (including the x,y,theta values).
	 */
	public Odometer() //positive x-axis is 0, increases counter clockwise
	{
		x = 0.0;
		y = 0.0;
		theta = 0;
		this.leftMotor = Main.leftMotor;
		this.rightMotor = Main.rightMotor;
		oldTachoLeft = 0;
		oldTachoRight = 0;
		nowTachoLeft = 0;
		nowTachoRight = 0;
		lock = new Object();
	}

	public void run() {
		long updateStart, updateEnd;

		while (true) { //use tachocounts of left and right motors to determine linear movement as well as how its heading changed
			updateStart = System.currentTimeMillis();
			nowTachoLeft = leftMotor.getTachoCount();
			nowTachoRight = rightMotor.getTachoCount();
			synchronized (lock) //locks the variables so that they cannot be accessed 
			{
				
				//calculates linear displacement. use this linear displacement and the heading to increment x and y
				displacementLeft = Math.PI*WHEEL_RADIUS*(nowTachoLeft - oldTachoLeft)/180;
				displacementRight = Math.PI*WHEEL_RADIUS*(nowTachoRight - oldTachoRight)/180;
				oldTachoLeft = nowTachoLeft;
				oldTachoRight = nowTachoRight;
				displacementRobot = 0.5*(displacementLeft + displacementRight);
				displacementArc = displacementRight - displacementLeft;
				theta += displacementArc/TRACK;
				if(theta < 0)
				{
					theta += 2 * Math.PI;
				}
				if(theta > 2*Math.PI)
				{
					theta -= 2 * Math.PI;

				}
				x += displacementRobot * Math.cos(theta);
				y += displacementRobot * Math.sin(theta);
			}
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try 
				{
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} 
				catch (InterruptedException e) {
				}
			}
		}
	}
	/**A getter that stores the x, y, heading values into an array of 3 doubles respectively. This method locks all objects so that x,y, theta cannot change from another thread while this method runs.
	 * @param position The array that will store the x, y, heading values. 
	 */
	public void getPosition(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) 
		{
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**A getter that returns the x value. This method also locks all objects so that x cannot change from another thread while this method runs
	 * @return A double that has the x value.
	 */
	public double getX() {
		double result;

		synchronized (lock) 
		{
			result = x;
		}

		return result;
	}
	/**A getter that returns the y value. This method also locks all objects so that y cannot change from another thread while this method runs
	 * @return A double that has the y value.
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}
	/**A getter that returns the heading value. This method also locks all objects so that theta cannot change from another thread while this method runs
	 * @return A double that has the heading value.
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**A setter for the x, y, and heading values. This methods locks all objects so that x, y, and theta cannot change from another thread while this method runs.
	 * @param position The double array that has the new x, y, theta values that we want to set the x, y, theta values to.
	 */
	public void setPosition(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) 
		{
			x = position[0];
			y = position[1];
			theta = position[2];
		}
	}

	/**A setter for the x value. This method also locks all objects so that x cannot change from another thread while this method runs.
	 * @param x The new x value we want to set x to.
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}
	/**A setter for the y value. This method also locks all objects so that y cannot change from another thread while this method runs.
	 * @param y The new y value we want to setyx to.
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}
	/**A setter for the theta value. This method also locks all objects so that theta cannot change from another thread while this method runs. Also makes sure theta is bounded between (0,2pi).
	 * @param theta The new theta value we want to set theta to.
	 */
	public void setTheta(double theta) {
		synchronized (lock) 
		{
			if(theta < 0)
			{
				theta += 2 * Math.PI;
			}
			if(theta > 2*Math.PI)
			{
				theta -= 2 * Math.PI;

			}
			this.theta = theta;
		}
	}
}
