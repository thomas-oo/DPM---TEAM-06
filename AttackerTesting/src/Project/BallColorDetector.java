package Project;


import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Ball color detection.
 *
 */
public class BallColorDetector {

	/**
	 * EV3 Color sensor
	 */
	private EV3ColorSensor colorSensor;
	/**
	 * data collected
	 */
	private float[] colorData;
	/**
	 * red, green, blue values data
	 */
	public float redValue,greenValue,blueValue;
	/**
	 * Sensor Mode
	 */
	private SensorMode colorValue;

	/**
	 * number of times red was detected
	 */
	int redCount = 0;
	/**
	 * number of times blue was detected
	 */
	int blueCount = 0;

	/**
	 * if a red is the color of the ball
	 */
	boolean redColorDetected = false;
	/**
	 * if blue is the color of the ball
	 */
	boolean blueColorDetected = false;
	
	
	/**
	 * 
	 * @param colorSensor 
	 * @param colorData
	 * @param colorValue
	 */
	public BallColorDetector(EV3ColorSensor colorSensor, float[] colorData, SensorMode colorValue)
	{
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.colorValue = colorValue;
	}

	/**
	 * starting color detection
	 */
	public void start()

	{
		try{
			while(true){

				getRGBValue();

				//optional (printing out the normalized values of red, green and blue)
				System.out.println("redValue: " + redValue);
				System.out.println("greenValue" + greenValue);
				System.out.println("blueValue" + blueValue);

				//in the case that the detected color of the ball is red
				if (redValue>greenValue && redValue>blueValue)
				{
					redCount ++;
				}

				// in the case that the detected color of the ball is blue
				//if (blueValue>redValue && blueValue>greenValue)
				if (!(redValue>greenValue && redValue>blueValue)) //if not red, then it's blue
				{
					blueCount++;
				}
				
				//if this code detects the color red 5 times, will report as detected red and get out of the loop
				if (redCount == 5)
				{
					redColorDetected = true;
					break;
				}
				
				//if this code detects the color blue 5 times, will report as detected blue and get out of the loop
				if (blueCount ==5)
				{
					blueColorDetected = true;
					break;
				}

				//this code runs every 200ms
				Thread.sleep(200);
			}
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
	}
	
	public int getColor()
	{
		if (redColorDetected)
		{
			return 1;
		}
		if (!redColorDetected)
		{
			return 2;
		}
		else
			return 2;
		
	}
	
	/**
	 * fetching the data and breaking it down into its color components
	 */
	public void getRGBValue()
	{
		//fetch the data obtained by the ColorSensor
		colorValue.fetchSample(colorData, 0); 

		redValue = colorData[0];
		greenValue = colorData[1];
		blueValue = colorData[2];	
	}

	
	
}
