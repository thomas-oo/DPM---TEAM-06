package Lab5ballistics;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab5 {
	static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	//static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	//define motors
	private static final Port touchsensorport = LocalEV3.get().getPort("S1");		
	//get touchsensor from port s1
	final static int forwardspeed=600;
	final static int backwardspeed=100;
	final static int ACCELERATION = 6000;
	//set motor speed and acceleration
	private static final int sleepperiod=2000;
	//control the period of sleep
	private static TextLCD LCD = LocalEV3.get().getTextLCD();
	//define the screen display
	public static void main(String[] args) 
	{
		SensorModes sensor = new EV3TouchSensor(touchsensorport);
		// get an instance of this sensor in measurement mode
		SampleProvider touch= sensor.getMode("Touch");
		float[] sample = new float[touch.sampleSize()];
		// define sample to fetch data
		int count=0; 
		//count #of shots
		while (true)
		{
			if (Button.ESCAPE.isDown()) 
			{
				break;
			}
			//if bck button pressed, break the loop
			LCD.clear();
			LCD.drawString("started: ", 0, 0);
			LCD.drawString("shots left: ", 0, 1);
			LCD.drawInt((5-count ), 11, 1);
			//draw on the screen
			touch.fetchSample(sample, 0);
			//fetch touchsensor data
			if (sample[0]==1)	//if touchsensor pressed, shot the ball
			{
				count++;
				Sound.beep(); //just for testing purpose
				leftMotor.setAcceleration(ACCELERATION);
			//	rightMotor.setAcceleration(ACCELERATION);
				leftMotor.setSpeed((int) forwardspeed);
			//	rightMotor.setSpeed((int) forwardspeed);
				leftMotor.rotate(-150,true);
				//rightMotor.rotate(80,false);
				//shot the ping-pang ball, all data tested by trial
				try { Thread.sleep(sleepperiod); } catch(Exception e){}		// Poor man's timed sampling
				leftMotor.setSpeed((int) backwardspeed);
				//rightMotor.setSpeed((int) backwardspeed);
				leftMotor.forward();
				leftMotor.flt();
				//move the arm back 
			}
		}
		
	}
}
