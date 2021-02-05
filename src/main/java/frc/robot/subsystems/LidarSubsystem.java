package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// NOTE: the lidar sensor measurement is from the back of lidar sensor

public class LidarSubsystem extends SubsystemBase {
	
	public double LIDAR_TOLERANCE = 0; // tune
	private Counter counter;
	
	public LidarSubsystem () {
		//super();

		System.out.println("Initializing Lidar");

		DigitalInput source = new DigitalInput(Constants.LIDAR_DIO);
		System.out.println("Source: " + source);
		counter = new Counter(source);

	    counter.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
	    counter.setSemiPeriodMode(true);
	    counter.reset();
		System.out.println("Counter: " + counter);

	}

	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in cm
	 */
	public double getDistance() {
		double cm;

		SmartDashboard.putNumber("Period", counter.getPeriod());

		// if (counter.get() < 1)
			// return 0;

		// while (counter.get() < 1) {
		// 	System.out.println("Lidar: waiting for distance measurement");
		// }
		/* getPeriod returns time in seconds. The hardware resolution is microseconds.
		 * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		 */
		cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return cm;
	}

	/**
	 * Converts getDistance() of cm to inches
	 * @return Distance in inches
	 */
	public double getInches() {
		return getDistance() / 2.54;
	}

	public double getFeet() {
		// SmartDashboard.putNumber("Dist in feet", getInches() / 12);
		return getInches() / 12;
	}
}