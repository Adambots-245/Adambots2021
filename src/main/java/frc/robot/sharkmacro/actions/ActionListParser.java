package frc.robot.sharkmacro.actions;

import java.util.ArrayList;
import java.util.List;

import frc.robot.sharkmacro.Constants;
import frc.robot.sharkmacro.Parser;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Handles the reading and writing of {@link ActionList}s.
 * 
 * @author Alec Minchington
 *
 */
public class ActionListParser extends Parser {

	/**
	 * Constructs a new {@link ActionListParser} object.
	 * 
	 * @param filename
	 *            name of the file to read or write a new file with
	 */
	public ActionListParser(String filename) {
		super(Constants.ACTIONLIST_STORAGE_DIRECTORY, Constants.ACTIONLIST_DEFAULT_PREFIX, filename);
	}

	/**
	 * This method writes an {@link ActionList} to a file. The given
	 * {@code ActionList} is transformed into a writable list and then passed to
	 * {@link org.hammerhead226.sharkmacro.Parser Parser} to be written.
	 * 
	 * @param al
	 *            the {@code ActionList} instance to write to file
	 * @return {@code true} if the file was written successfully, {@code false} if
	 *         not
	 */
	public boolean writeToFile(ActionList al) {
		if (al.getSize() == 0) {
			DriverStation.getInstance();
			DriverStation.reportWarning("Tried to write empty ActionList!", false);
			return false;
		}

		ArrayList<String[]> actionListToWrite = new ArrayList<String[]>(al.getSize());
		for (Action a : al) {
			actionListToWrite.add(a.toStringArray());
		}

		return super.writeToFile(actionListToWrite);
	}

	/**
	 * This method gets the raw action list from
	 * {@link org.hammerhead226.sharkmacro.Parser Parser} and transforms it into an
	 * {@link ActionList} instance.
	 * <p>
	 * {@link org.hammerhead226.sharkmacro.Parser#cache Parser.cache} is checked
	 * first to see if the file has already been parsed, and returns a clone of the
	 * {@code ActionList} from the cache if the file exists in the cache. If the
	 * file does not exist in the cache, then the file is parsed and added to the
	 * cache.
	 * 
	 * @return a new {@code ActionList} instance
	 */
	public ActionList toObject() {

		List<String[]> actionListRaw = readFromFile();

		if (actionListRaw == null) {
			DriverStation.getInstance();
			DriverStation.reportError("Tried to load nonexistant ActionList from name: " + super.filename, false);
			return new ActionList(null);
		}

		ArrayList<Action> list = new ArrayList<Action>(Constants.ACTIONRECORDER_LIST_DEFAULT_LENGTH);
		for (String[] s : actionListRaw) {
			list.add(new Action(s[0], Double.parseDouble(s[1]), Double.parseDouble(s[2])));
		}

		ActionList al = new ActionList(list);

		return al;
	}

	/**
	 * Cache a saved actionlist.
	 * 
	 * @param filename
	 *            the name of the actionlist to cache
	 */
	public static void cache(String filename) {
		Parser.cache(Constants.ACTIONLIST_STORAGE_DIRECTORY, filename);
	}

	/**
	 * Cache all action lists in the action list storage directory.
	 */
	public static void cacheAll() {
		Parser.cacheAll(Constants.ACTIONLIST_STORAGE_DIRECTORY);
	}

	/**
	 * This method generates a new filename to be used for saving a new file. For
	 * example, if the newest file in the storage directory is
	 * {@code prefix0003.csv}, the method will return {@code prefix0004.csv}.
	 * 
	 * @return a new complete filename in the prefix + number naming convention
	 */
	public static String getNewFilename() {
		return getNewFilename(Constants.ACTIONLIST_STORAGE_DIRECTORY, Constants.ACTIONLIST_DEFAULT_PREFIX);
	}

	/**
	 * This method finds the newest file named with prefix + number naming
	 * convention in the storage directory.
	 * 
	 * @return the complete filename of the latest (highest numbered) file in the
	 *         storage directory
	 */
	public static String getNewestFilename() {
		return getNewestFilename(Constants.ACTIONLIST_STORAGE_DIRECTORY, Constants.ACTIONLIST_DEFAULT_PREFIX);
	}
}