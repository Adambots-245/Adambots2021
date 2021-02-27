package frc.robot.sharkmacro.motionprofiles;

import java.util.ArrayList;

import frc.robot.sharkmacro.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Intermediary class that represents a raw recording of a motion profile.
 * 
 * @author Alec Minchington
 *
 */
public class Recording {

	/**
	 * A list containing the lists of recorded positions and velocities.
	 */
	private ArrayList<ArrayList<Double>> recordings;

	/**
	 * Left Talon to pass to the {@link Profile} generated in {@link #toProfile()}.
	 */
	private WPI_TalonFX leftTalon;

	/**
	 * Right Talon to pass to the {@link Profile} generated in {@link #toProfile()}.
	 */
	private WPI_TalonFX rightTalon;

	/**
	 * Constructs a new {@link Recording} object.
	 * 
	 * @param recordings
	 *            a list containing the lists of recorded positions and velocities
	 * @param leftTalon
	 *            Talon used to record left position and velocity, passed from
	 *            {@link ProfileRecorder#stop()}
	 * @param rightTalon
	 *            Talon used to record right position and velocity, passed from
	 *            {@link ProfileRecorder#stop()}
	 */
	public Recording(ArrayList<ArrayList<Double>> recordings, WPI_TalonFX leftTalon, WPI_TalonFX rightTalon) {
		this.recordings = recordings;
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
	}

	/**
	 * Transforms the raw recorded positions and velocities into a Talon-formatted
	 * motion profile. Each point in the motion profile is formatted as follows:
	 * <p>
	 * <center>
	 * {@code [ <position in raw units>, <velocity in raw units per 100ms>, <time for the Talon to hold this point> ]}
	 * </center>
	 * </p>
	 * 
	 * @return a new {@link Profile} containing the new motion profiles
	 */
	public Profile toProfile() {

		// Remove differential in list size
		int minSize = Integer.MAX_VALUE;
		for (int i = 0; i < recordings.size(); i++) {
			if (recordings.get(i).size() < minSize) {
				minSize = recordings.get(i).size();
			}
		}
		for (int i = 0; i < recordings.size(); i++) {
			recordings.get(i).subList(minSize, recordings.get(i).size()).clear();
		}

		// Remove leading zero rows
		for (int i = 0; i < minSize; i++) {
			if (!areEqual(i, 0)) {
				for (int j = 0; j < recordings.size(); j++) {
					recordings.get(j).subList(0, i).clear();
				}
				minSize -= i;
				break;
			}
		}

		ArrayList<Double> leftPosition = recordings.get(0);
		ArrayList<Double> leftFeedforwardValues = recordings.get(1);
		ArrayList<Double> rightPosition = recordings.get(2);
		ArrayList<Double> rightFeedforwardValues = recordings.get(3);

		double[][] leftProfile = new double[minSize][3];
		double[][] rightProfile = new double[minSize][3];

		for (int i = 0; i < minSize; i++) {
			leftProfile[i][0] = leftPosition.get(i);
			leftProfile[i][1] = leftFeedforwardValues.get(i);
			leftProfile[i][2] = Constants.DT_MS;

			rightProfile[i][0] = rightPosition.get(i);
			rightProfile[i][1] = rightFeedforwardValues.get(i);
			rightProfile[i][2] = Constants.DT_MS;
		}
		return new Profile(leftProfile, rightProfile);
	}
	
	/**
	 * Converts an {@code ArrayList} of type Integer to an {@code ArrayList} of type
	 * Double.
	 * 
	 * @param list
	 *            list to convert
	 * @return converted list
	 */
	private ArrayList<Double> toDoubleList(ArrayList<Integer> list) {
		ArrayList<Double> d = new ArrayList<Double>(list.size());
		for (Integer i : list) {
			d.add((double) i);
		}
		return d;
	}

	/**
	 * Compares the positions and velocities at a given instant in time.
	 * 
	 * @param idx
	 *            list index to get comparable values from
	 * @param comparator
	 *            value to compare the positions and velocities to
	 * @return {@code true} if all four positions and velocities are equal to the
	 *         comparator, {@code false} otherwise
	 */
	private boolean areEqual(int idx, double comparator) {
		return (recordings.get(0).get(idx) == comparator && recordings.get(1).get(idx) == comparator
				&& recordings.get(2).get(idx) == comparator && recordings.get(3).get(idx) == comparator);
	}

	/**
	 * Converts raw units into wheel rotations.
	 * 
	 * @param rawUnits
	 *            raw sensor units
	 * @return raw units as wheel rotations
	 */
	private double toRotations(double rawUnits) {
		return rawUnits * (1.0 / Constants.ENCODER_COUNTS_PER_REV);
	}

	/**
	 * Converts raw units per 100ms into RPM.
	 * 
	 * @param rawUnitsPer100ms
	 *            raw sensor units
	 * @return raw units per 100ms as RPM
	 */
	private double toRPM(double rawUnitsPer100ms) {
		return rawUnitsPer100ms * (600.0 / Constants.ENCODER_COUNTS_PER_REV);
	}
}