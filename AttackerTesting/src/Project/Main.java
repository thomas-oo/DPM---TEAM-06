package Project;


public class Main {
	
	public static BallGrab ballGrabber;
	
	public static void main (String[]args)
	{
		System.out.println("I work");
		ballGrabber = new BallGrab();
		ballGrabber.grabBall();
	}
	
	public static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}

}
