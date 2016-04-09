package Robot;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
public class Defense 
{
	public static final EV3MediumRegulatedMotor defensemotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	final static int holdSpeed = 0;
	final static int openSpeed = 100;
	final static int holdacc = 2000;
	final static int openacc = 100;
	public void mode(int status) 
	{
		if (status==0)
		{
			defensemotor.setAcceleration(0);
			defensemotor.setSpeed(0);
			defensemotor.flt();
			System.out.println("Mode0");
			return;
		}
		if (status==1)
		{
			defensemotor.setAcceleration(holdacc);
			defensemotor.setSpeed(holdSpeed);	
			defensemotor.lock(100);
			System.out.println("Mode1");
			return;
		}
		if (status==2)
		{
			System.out.println("Mode2");
			defensemotor.setAcceleration(openSpeed);
			defensemotor.setSpeed(openacc);	
			defensemotor.rotateTo(90);
			defensemotor.lock(50);
			//defensemotor.setAcceleration(holdacc);
			//defensemotor.setSpeed(holdSpeed);	
			
			return;
		}
		if (status==3)
		{
			System.out.println("Mode3");
			defensemotor.setAcceleration(openSpeed);
			defensemotor.setSpeed(openacc);	
			defensemotor.rotateTo(0);
			defensemotor.lock(50);
			//defensemotor.setAcceleration(holdacc);
			//defensemotor.setSpeed(holdSpeed);	
			
			return;
		}
		if (status==4)
		{
			System.out.println("Mode4");
			defensemotor.lock(50);
			return;
		}
		return;
	}
	
}
