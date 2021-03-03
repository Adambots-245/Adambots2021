package frc.robot.sharkmacro.motionprofiles;

import java.util.ArrayList;
import java.util.List;

import frc.robot.sharkmacro.Constants;
import frc.robot.sharkmacro.Parser;
import frc.robot.utils.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Handles the reading and writing of {@link Profile}s.
 * 
 * @author Alec Minchington
 *
 */
public final class ProfileParser extends Parser {

	/**
	 * Constructs a new {@link ProfileParser} object.
	 * 
	 * @param filename
	 *            name of the file to read or write a new file with
	 */
	public ProfileParser(String filename) {
		super(Constants.PROFILE_STORAGE_DIRECTORY, Constants.PROFILE_DEFAULT_PREFIX, filename);
	}

	/**
	 * This method writes an {@link Profile} to a file. The given {@code Profile} is
	 * transformed into a writable list and then passed to
	 * {@link org.hammerhead226.sharkmacro.Parser Parser} to be written.
	 * 
	 * @param al
	 *            the {@code Profile} instance to write to file
	 * @return {@code true} if the file was written successfully, {@code false} if
	 *         not
	 */
	public boolean writeToFile(Profile profile) {
		ArrayList<String[]> profileToWrite = new ArrayList<String[]>(profile.length);
		String[][] left = profile.getLeftProfile_String();
		String[][] right = profile.getRightProfile_String();

		for (int i = 0; i < profile.length; i++) {
			String[] line = new String[] { left[i][0], left[i][1], right[i][0], right[i][1],
					Integer.toString(profile.dt, 10) };
			profileToWrite.add(line);
		}

		return super.writeToFile(profileToWrite);
	}

	/**
	 * This method gets the raw action list from
	 * {@link org.hammerhead226.sharkmacro.Parser Parser} and transforms it into an
	 * {@link Profile} instance.
	 * <p>
	 * {@link org.hammerhead226.sharkmacro.Parser#cache Parser.cache} is checked
	 * first to see if the file has already been parsed, and returns a clone of the
	 * {@code Profile} from the cache if the file exists in the cache. If the file
	 * does not exist in the cache, then the file is parsed and added to the cache.
	 * 
	 * @return a new {@code Profile} instance
	 */
	public Profile toObject(WPI_TalonFX leftTalon, WPI_TalonFX rightTalon, int leftPidSlotIdx, int rightPidSlotIdx) {
		List<String[]> profileRaw = readFromFile();

		if (profileRaw == null) {
			DriverStation.getInstance();
			DriverStation.reportError("Tried to load nonexistant Profile from name: " + super.filename, false);
			return new Profile(new double[0][0], new double[0][0], leftTalon, rightTalon, leftPidSlotIdx,
					rightPidSlotIdx);
		}

		// Process read values into Profile
		String[][] left = new String[profileRaw.size()][3];
		String[][] right = new String[profileRaw.size()][3];

		for (int i = 0; i < profileRaw.size(); i++) {
			left[i][0] = profileRaw.get(i)[0];
			left[i][1] = profileRaw.get(i)[1];
			left[i][2] = profileRaw.get(i)[4];

			System.out.print("toObject() left values:\tpos? " + left[i][0] + "\tvel? " + left[i][1] + "\theading? " + left[i][2] + "\n");

			right[i][0] = profileRaw.get(i)[2];
			right[i][1] = profileRaw.get(i)[3];
			right[i][2] = profileRaw.get(i)[4];
		}

		Profile p = new Profile(left, right, leftTalon, rightTalon, leftPidSlotIdx, rightPidSlotIdx);

		return p;
	}
	
	/**
	 * Cache a saved profile.
	 * 
	 * @param filename
	 *            the name of the profile to cache
	 */
	public static void cache(String filename) {
		Parser.cache(Constants.PROFILE_STORAGE_DIRECTORY, filename);
	}
	
	/**
	 * Cache all profiles in the profile storage directory.
	 */
	public static void cacheAll() {
		Parser.cacheAll(Constants.PROFILE_STORAGE_DIRECTORY);
	}

	/**
	 * This method generates a new filename to be used for saving a new file. For
	 * example, if the newest file in the storage directory is
	 * {@code prefix0003.csv}, the method will return {@code prefix0004.csv}.
	 * 
	 * @return a new complete filename in the prefix + number naming convention
	 */
	public static String getNewFilename() {
		return getNewFilename(Constants.PROFILE_STORAGE_DIRECTORY, Constants.PROFILE_DEFAULT_PREFIX);
	}

	/**
	 * This method finds the newest file named with prefix + number naming
	 * convention in the storage directory.
	 * 
	 * @return the complete filename of the latest (highest numbered) file in the
	 *         storage directory
	 */
	public static String getNewestFilename() {
		return getNewestFilename(Constants.PROFILE_STORAGE_DIRECTORY, Constants.PROFILE_DEFAULT_PREFIX);
	}
}
