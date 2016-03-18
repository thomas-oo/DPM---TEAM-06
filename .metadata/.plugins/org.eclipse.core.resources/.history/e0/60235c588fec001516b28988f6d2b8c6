package logging_data;


import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

//
//  Control of the wall follower is applied periodically by the 
//  UltrasonicPoller thread.  The while loop at the bottom executes
//  in a loop.  Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  50 mS at the end of each loop, then one cycle through the loop
//  is approximately 70 mS.  This corresponds to a sampling rate
//  of 1/70mS or about 14 Hz.
//

public class UltrasonicPoller extends Thread {
	private SampleProvider median;
	private float[] sample;
	int distance;

	public UltrasonicPoller(SampleProvider median) {
		this.median = median;
		sample=new float[median.sampleSize()];
		
	}

	// Sensors now return floats using a uniform protocol.
	// Need to convert US result to an integer [0,255]

	public void run() {
		while (true) {
			median.fetchSample(sample, 0); // acquire data
			distance = (int) (sample[0] * 100.0); // extract from buffer, cast										
		}
	}

	public int getDistance() {
		return distance;
	}

}