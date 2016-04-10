package Robot;

import Robot.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class accomplishes the task of Navigation, that is, it is responsible for sending commands to the motors to go to a coordinate.
 * In addition to movement, this class can also rotate the robot to a certain heading using an Odometer.
 * The methods in this class are all private and non static. The variables used in this navigator are private but this class uses a few public variables from the main method.
 * 
 * @author Thomas, Aagnik
 */
public class Navigator extends Thread
{
	/**
	 * The angular velocity the wheels will go forward at (in deg/sec).
	 */
	private int forwardSpeed = 150;

	//destination variables
	/**
	 * The array that stores the x and y coordinates for the destination (in cm). destDistance[0] will store x, destDistance[1] will store y.
	 */
	private double[] destDistance = new double[2];
	/**
	 * The heading the robot will need to head, if it wants to go straight and arrive at the destination (in radians).
	 */
	private double destTheta;

	//current coordinates
	/**
	 * An array that stores the current x, y, and heading values of the robot as passed from odometer.
	 */
	private double[] nowDistance = new double[3];
	/**
	 * Stores the current x coordinate (in cm).
	 */
	private double nowX;
	/**
	 * Stores the current y coordinate (in cm).
	 */
	private double nowY;
	/**
	 * Stores the current heading (in radians).
	 */
	private double nowTheta;

	//thresholds to allow control to within this window.
	/**
	 * The allowable threshold for angles to be within this value of the optimal value (in radians). It is a final (constant) variable.
	 */
	private final double thetaThreshold = 0.00873; //2 degrees in rads
	/**
	 * The allowable threshold for coordinates (x and y) to be within this value of the optimal value (in cm). It is a final (constant) variable.
	 */
	private final double destThreshold = 0.4;

	//state variables
	/**
	 * Flag to state whether or not the robot is navigating to a coordinate. 
	 *</p>
	 * True if it is in the process of navigating (turning, moving, obstacle avoiding). False if 1) it has no coordinate to navigate to or 2) arrived at the destination.
	 */
	private boolean isNavigating;
	/**
	 * Flag to state whether or not the robot is moving towards a coordinate.
	 *</p>
	 * True if it is in the process of moving. False if it is not moving.
	 */
	private boolean isCorrecting;

	//motors that need to be used
	/**
	 * The EV3LargeRegulatedMotor object for the left motor. This is set in Main.
	 */
	private EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	/**
	 * The EV3LargeRegulatedMotor object for the right motor. This is set in Main.
	 */
	private EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	/**
	 * The EV3LargeRegulatedMotor object for the motor that rotates the ultrasonic sensor. This is set in Main.
	 */
	private EV3MediumRegulatedMotor headMotor; //WE NEED A MOTOR FOR US SENSOR!!!! (or find an alternative design)

	//classes that navigator depends on
	/**
	 * The Odometer object that we use to access x, y, and theta values.
	 */
	private Odometer odometer;
	/**
	 * The ObstacleAvoidance object that gets created when we detect a wall. This is set in Main.
	 */
	private ObstacleAvoidance avoidance;

	//buffers for accessing ultrasonic data
	/**
	 * Buffer array to store data from the ultrasonic sensor (in m).
	 */
	public float[] usData;
	/**
	 * Stores the most recent distance reading from the ultrasonic sensor (in cm).
	 */
	public double usDistance;
	/**
	 * Buffer sampleProvider that can fetch samples from the ultrasonic sensor. 
	 */
	public SampleProvider usSampleProvider;

	//variables that are set in main, passed here
	private final int wallDist;
	/**
	 * Distance to keep away from a wall when the robot encounter an obstacle (in cm). This is set in Main. This is a final (constant) variable.
	 */
	private final int bandCenter;
	/**
	 * Allowable deviation from the bandCenter that the ultrasonic sensor reading can be before it 1) goes closer or 2) goes further from the wall (in cm). This is set in Main. This is a final (constant) variable.
	 */
	private final int bandWidth;
	/**
	 * Stores the low motor speed (in deg/s). This is set in Main. This is a final (constant) variable.
	 */
	private final int motorLow;
	/**
	 * Stores the high motor speed (in deg/s). This is set in Main. This is a final (constant) variable.
	 */
	private final int motorHigh;
	/** 
	 * Status of whether the robot is playing. True if it's playing. False if it's not (i.e. it's localizing or travelling to home zone)
	 */
	private boolean isPlaying;
	/**
	 * This constructor takes NO inputs. The constructor will its class variables to the ones set in Main. We do not except these variables to change.
	 */

	/**
	 * This stores the possible states that Navigator can exist in. These states flow sequentially except for WALL.
	 * @author Thomas
	 *
	 */
	enum State {
		/**
		 * The initial state that Navigator goes into when it is called to do something.
		 */
		INIT,
		/**
		 * The state that Navigator goes into when the robot detects an obstacle.
		 */
		WALL,
		/**
		 * The state that Navigator goes into when the robot needs to turn to a desired heading.
		 */
		TURNING, 
		/**
		 * The state that Navigator goes into when the robot is traveling (linear movement).
		 */
		TRAVELING};
		
	/** 
	 * This stores the possible roles that the player can take on. 
	 * @author Aagnik
	 */
	enum Role {Attacker, Defender}
	/**
	 * Private variable to store the role of the player
	 */
	private Role role;

		/**
		 * Constructor for Navigator class. Takes no inputs but inherits many variables from Main.
		 */
		public Navigator()
		{
			this.odometer = Main.odometer;
			this.leftMotor = Main.leftMotor;
			this.rightMotor = Main.rightMotor;
			this.headMotor = Main.headMotor;

			this.bandCenter = Main.bandCenter;
			this.bandWidth = Main.bandWidth;
			this.motorLow = Main.motorLow; 
			this.motorHigh = Main.motorHigh;
			this.wallDist = Main.wallDist;

			this.usSampleProvider = Main.usValue;
			this.usData = new float[usSampleProvider.sampleSize()];
			
			if (Main.role == 0) this.role = Role.Attacker;
			else if (Main.role == 1) this.role = Role.Defender;
		}

		public void run()
		{
			//4 states
			//state for initializing
			//if navigator has a destination, go to state turning
			//state for turning
			//turn to the destination theta if it isn't already facing that direction
			//go to state traveling
			//state for traveling
			//before moving forward, check if there is an emergency (wall), if so, go to state wall
			//if no emergency, check if you are already at the waypoint, 
			//if not, move with forward speed
			//for the next cycle, recalc a the latest destination theta with latest coordinates from odometer
			//on next cycle, go to state init to make sure that you are heading in the new theta direction
			//if so, stop, and set the flag isNavigating to false, lets the main method know that the waypoint is reached.
			//state for wall
			//if a wall is detected, turn 90 deg clockwise and position the camera at a 45 degree angle towards the wall.
			//start avoidance (navigator thread is blocked UNTIL avoidance is done.
			//avoidance continues until your coordinate is within the vector connecting where you first detected the wall, to the waypoint
			//once that is done, you should be safe so turn back 90 deg and go to state init to continue with your path.
			State state = State.INIT;
			while (true)
			{
				odometer.getPosition(nowDistance);
				nowX = nowDistance[0];
				nowY = nowDistance[1];
				nowTheta = nowDistance[2];

				usSampleProvider.fetchSample(usData, 0);
				usDistance = (double)(usData[0]*100.0);

				isCorrecting = false;

				switch(state)
				{
				case INIT:
					if(isNavigating)
					{
						state = State.TURNING;
					}
					break;
				case TURNING:
					isCorrecting = false;
					if (!facingDest(destTheta))
					{
						leftMotor.stop(true);
						rightMotor.stop(false);
						turnTo(destTheta);
					}
					else if(facingDest(destTheta))
					{
						state = State.TRAVELING;
					}
					break;
				case TRAVELING:
					isCorrecting = true;
					if(checkEmergency())
					{
						state = State.WALL;
						break;
					}
					else if(!checkIfDone(nowDistance))
					{
						setSpeeds(forwardSpeed, forwardSpeed);
						leftMotor.forward();
						rightMotor.forward();
						updateTravel();
						System.out.println("destTheta: " + destTheta*57.3);
						state = State.INIT;
					}
					else if(checkIfDone(nowDistance))
					{
						leftMotor.stop(true);
						rightMotor.stop(false);
						isNavigating = false;
						System.out.println("Arrived at x: " + destDistance[0] + " y: " + destDistance[1]);
						state = State.INIT;
					}
					break;
				case WALL:
					isCorrecting = false;
					setSpeeds(forwardSpeed, forwardSpeed);
					leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, (Math.PI)/2), true);
					rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, (Math.PI)/2));

					headMotor.rotate(45);

					avoidance = new ObstacleAvoidance(this, nowX, nowY, nowTheta, odometer,leftMotor, rightMotor,bandCenter, bandWidth,
							motorLow, motorHigh, usSampleProvider);
					avoidance.start(); 
					try 
					{
						avoidance.join();
					} catch (InterruptedException e1) 
					{
						e1.printStackTrace(); 
					}
					System.out.println("WALL DONE: Arrived at x: " + odometer.getX() + " y: " + odometer.getY());
					leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, (Math.PI)/2), true);
					rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, (Math.PI)/2));
					headMotor.rotate(-45);
					state = State.INIT;
					break;
				}
				try
				{
					Thread.sleep(30);
				}
				catch(InterruptedException e)
				{
					e.printStackTrace();
				} 	
			}
		}


		/**Method that checks if usDistance is <= 20. If so, we will say that there is an obstacle.
		 * @return 
		 * True if we see an obstacle.
		 * </p>
		 * False if we do not see an obstacle.
		 */
		private boolean checkEmergency() { //checking if it's too close to the wall
			if(usDistance <= wallDist || enteringForbiddenZone())
				return true;
			else
				return false;
		}
		/**
		 * Updates the destTheta to a more current one.
		 */
		private void updateTravel() //update the destination angle of the next "cycle" of travel
		{
			destTheta = getDestAngle();
		}
		/**Checks if we are facing in the heading towards the destination within a threshold.
		 * @param destTheta The heading we need to face so that we are headed towards the destination
		 * @return True if we are facing in the desired heading.
		 * </p> False if we are not facing in the desired heading.
		 */
		private boolean facingDest(double destTheta) //checks if nowTheta is facing destTheta within a leeway of thetaThreshold 
		{
			if(nowTheta > (destTheta - thetaThreshold) && nowTheta < (destTheta + thetaThreshold))
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		/**Checks if we are at the destination within a threshold in both x and y.
		 * @param nowDistance The array that stores the x and y coordinates of the destination.
		 * @return True if we are at the destination.
		 * </p> False if we are not at the destination
		 */
		private boolean checkIfDone(double[] nowDistance) //checks if nowX, Y are within destThreshold of destDistnace[0], [1]
		{
			if(nowX > (destDistance[0]-destThreshold) && (nowX < (destDistance[0] + destThreshold)))
			{
				if(nowY > (destDistance[1]-destThreshold) && (nowY < (destDistance[1]+ destThreshold)))
				{
					return true;
				}
				return false;
			}
			return false;
		}
		/**Simply sets left and right motors to a certain speed (in deg/s).
		 * @param leftSpeed Left motor speed
		 * @param rightSpeed Right motor speed
		 */
		private void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
		{
			leftMotor.setSpeed(leftSpeed);
			rightMotor.setSpeed(rightSpeed);
			isNavigating();
		}
		/**Public method that is called outside of navigator to give navigator the command to move to a certain coordinate. </p>
		 * Stores the destination x and y coordinates inside destDistance[]. Calls getDestAngle to calculate the desired heading. Raises the isNavigating flag.
		 * @param destX
		 * @param destY
		 */
		public void travelTo(double destX, double destY) //called from main, (never called inside navigator). Is basically a setter for destination and flags isNavigating to true
		{
			destDistance[0] = destX;
			destDistance[1] = destY;
			destTheta = getDestAngle();
			isNavigating = true;
		}
		/**Uses the destination x and y coordinates and the current x and y coordinates to calculate the desired heading (destTheta)
		 * @return A desired heading (in radians).
		 */
		private double getDestAngle() //uses destDistance[0],[1] to calculate destTheta. destTheta is the heading with respect to coordinate system. 
		//(where the min is 0 (on positive x-axis), and max is 2pi. positive theta is counterclockwise angle from the positive x-axis.
		{
			double errorX = destDistance[0] - nowX;
			double errorY = destDistance[1] - nowY;

			if(Math.abs(errorX) < destThreshold)
			{
				if(errorY > 0)
				{
					return 0.5 * Math.PI; //90
				}
				else
				{
					return 1.5 * Math.PI; //270
				}
			}
			else if(Math.abs(errorY) < destThreshold)
			{
				if(errorX > 0)
				{
					return 0.0; //0
				}
				else
				{
					return Math.PI; //180
				}
			}


			else if(errorX > 0) 
			{
				if(errorY > 0) //positive theta
				{
					return Math.atan(errorY/errorX);
				}
				else //converts quadrant 4 into a positive theta
				{
					return 2*Math.PI + Math.atan(errorY/errorX);
				}
			}
			else if(errorX < 0)
			{
				if(errorY > 0) //quad 2, positive theta
				{
					return (Math.atan(errorY/errorX) + Math.PI);
				}
				else if(errorY < 0) //quad 3, positive theta
				{
					return (Math.atan(errorY/errorX) + Math.PI);
				}
			}
			return 0; //IF SOMETHING GOES HORRIBLY WRONG, RETURN 0.
		}
		public double getDestAngle(int x, int y) //uses destDistance[0],[1] to calculate destTheta. destTheta is the heading with respect to coordinate system. 
		//(where the min is 0 (on positive x-axis), and max is 2pi. positive theta is counterclockwise angle from the positive x-axis.
		{
			double errorX = x - nowX;
			double errorY = y - nowY;

			if(Math.abs(errorX) < destThreshold)
			{
				if(errorY > 0)
				{
					return 0.5 * Math.PI; //90
				}
				else
				{
					return 1.5 * Math.PI; //270
				}
			}
			else if(Math.abs(errorY) < destThreshold)
			{
				if(errorX > 0)
				{
					return 0.0; //0
				}
				else
				{
					return Math.PI; //180
				}
			}


			else if(errorX > 0) 
			{
				if(errorY > 0) //positive theta
				{
					return Math.atan(errorY/errorX);
				}
				else //converts quadrant 4 into a positive theta
				{
					return 2*Math.PI + Math.atan(errorY/errorX);
				}
			}
			else if(errorX < 0)
			{
				if(errorY > 0) //quad 2, positive theta
				{
					return (Math.atan(errorY/errorX) + Math.PI);
				}
				else if(errorY < 0) //quad 3, positive theta
				{
					return (Math.atan(errorY/errorX) + Math.PI);
				}
			}
			return 0; //IF SOMETHING GOES HORRIBLY WRONG, RETURN 0.
		}

		/**Sends command to left and right motors to turn a certain about of degrees so that the robot will turn to a desired heading. The turn itself will be a minimal turn ie. A turn CW/CCW of >180 degrees will be a turn CCW/CW of (360 - the desired >180 degrees). All angles in this method are in radians.
		 * </p> Relies on WHEEL_RADIUS and TRACK to calculate each motor's rotation (in radians).
		 * @param destTheta The desired heading that you want to robot to turn to.
		 */
		public void turnTo(double destTheta) //uses destTheta and nowTheta to calculate AND TURN required minimal angle.
		{
			setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.
			//nowTheta is recalc'd here.
			nowTheta = odometer.getTheta();
			double turnTheta = destTheta - nowTheta; //dest and nowTheta both are from [0,2pi]
			//CALCULATES MINIMAL TURN and EXECUTES
			//ROTATES UNTIL TURN IS COMPLETE.
			if(turnTheta >= -Math.PI && turnTheta <= Math.PI)
			{
			}
			else if(turnTheta < -Math.PI && turnTheta > -2*Math.PI)
			{
				turnTheta = turnTheta + 2*Math.PI;
			}
			else if(turnTheta>Math.PI && turnTheta < 2*Math.PI)
			{
				turnTheta = turnTheta - 2*Math.PI;
			}
			else
			{
			}
			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, turnTheta), true);
			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, turnTheta));
		}
		/**A public getter for isNavigating
		 * @return isNavigating
		 */
		public boolean isNavigating() //returns true if another thread has called travelTo or turnTo
		{
			return isNavigating;
		}
		/**A public getter for isCorrecting
		 * @return isCorrecting
		 */
		public boolean isCorrecting()
		{
			return isCorrecting;
		}

		/**Converts linear distance that you want a wheel to rotate into a value that stores the rotations (in degrees) that the wheel needs to rotate.
		 * @param radius The radius of the wheel.
		 * @param distance The distance you want a wheel to rotate.
		 * @return An int that stores the rotation (in degrees) that a wheel needs to rotate.
		 */
		private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
		{
			return (int) ((90.0 * distance) / (Math.PI * radius));
		}
		/** Converts a desired change in robot's heading (in radians) into how much linear distance a wheel needs to travel (in cm). 
		 * @param radius The radius of the wheel (in cm).
		 * @param width The track or width of the robot (in cm).
		 * @param angle The angle you want the robot to turn (in radians).
		 * @return The linear distance that a wheel would have to rotate.
		 */
		private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
		{ //hopefully works
			return convertDistance(radius, angle*width);
		}
		/**A public setter for isNavigating
		 * @param isNavigating Flag that tells if the robot is navigating or not.
		 */
		public void setNavigating(boolean isNavigating) //sets isNavigating if ever needed (unused here)
		{
			this.isNavigating = isNavigating;
		}
		/**
		 * A public setter for setting the playing variable to true or false
		 * @param isPlaying Flag that tells if the robot is playing
		 */
		public void setPlaying(boolean isPlaying)
		{
			this.isPlaying = isPlaying;
		}
		/**
		 * Determines if the player is navigating into the forbidden zone
		 * @ return True if entering forbidden zone, false if not
		 */
		private boolean enteringForbiddenZone()
		{
			// get values of the odometer
			double x = odometer.getX();
			double y = odometer.getY();
			double theta = odometer.getTheta();

			// if the robot is playing and the robot is approaching the forbidden zones
			// For attacker, forbidden zones are: RED REGION
			// For defender, forbidden zones are: BLUE REGION, GREEN REGION
			// return FALSE
			if (isPlaying)
			{
				if (role == Role.Attacker)
				{
					if (x >= 30 || x <= 9*30)
					{
						if (Math.abs(y - 8*30) <= 5 && Math.abs(theta - Math.PI/2) <= 10) return true;
						else if (Math.abs(y - 10*30) <= 5 && Math.abs(theta - 3*Math.PI/2) <= 10) return true;
					} 
					else if (Math.abs(x - 30) <= 5 && Math.abs(theta) <= 10) return true; 
					else if (Math.abs(x - 9*30) <= 5 && Math.abs(theta - Math.PI) <= 10) return true;
				}
				else if (role == Role.Defender)
				{
					if (x >= 30 || x <= 9*30)
					{
						if (Math.abs(y - 8*30) <= 5 && Math.abs(theta - 3 * Math.PI/2) <= 10) return true;
						else if (Math.abs(y - 10*30) <= 5 && Math.abs(theta - Math.PI/2) <= 10) return true;
					} 
					else if (Math.abs(x - 30) <= 5 && Math.abs(theta - Math.PI) <= 10) return true; 
					else if (Math.abs(x - 9*30) <= 5 && Math.abs(theta) <= 10) return true;
				}
			}
			// if the robot is travelling to home zone and the robot is approaching the forbidden zones
			// For attacker, forbidden zones are: RED REGION, BLUE REGION
			// For defender, forbidden zones are: BLUE REGION, GREEN REGION
			// return TRUE
			else if (!isPlaying) 
			{
				if (role == Role.Attacker)
				{
					if (y >= 2*30 && y <= 10*30)
					{
						if (Math.abs(x - 30) <= 5 && Math.abs(theta) <= 10) return true; 
						else if (Math.abs(x - 9*30) <= 5 && Math.abs(theta - Math.PI) <= 10) return true;
					}
				}
				else if (role == Role.Defender)
				{
					if (y >= 30 && y <= 8*30)
					{
						if (Math.abs(x - 30) <= 5 && Math.abs(theta) <= 10) return true; 
						else if (Math.abs(x - 9*30) <= 5 && Math.abs(theta - Math.PI) <= 10) return true;
					}
				}
			}
			// if none of the conditions above were satisfied then the robot is safe
			return false;
		}
}
