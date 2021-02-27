package frc.robot.sharkmacro.actions;

import java.util.ArrayList;

import frc.robot.sharkmacro.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This class's function is to record a robot's actions and export them into an
 * {@link ActionList}.
 * 
 * @author Alec Minchington
 *
 */
public class ActionRecorder {

	/**
	 * Represents whether this {@link ActionRecorder} has started recording
	 * {@link Actions}.
	 */
	private static boolean isRecording = false;

	/**
	 * {@link edu.wpi.first.wpilibj.Timer Timer} used to record the start and end
	 * time of {@link Action}s being recorded.
	 */
	private static Timer timer = new Timer();

	/**
	 * This list serves as a buffer for recorded {@link Action}s to be stored in
	 * before they're exported to an {@link ActionList}.
	 */
	private static ArrayList<Action> buffer = new ArrayList<Action>(Constants.ACTIONRECORDER_LIST_DEFAULT_LENGTH);

	/**
	 * This method starts listening for calls to {@link #addAction(Action)} and
	 * starts the {@link #timer}.
	 */
	public static void start() {
		if (!isRecording) {
			buffer.clear();
			timer.start();
			isRecording = true;
		}
	}

	/**
	 * This method stops listening for calls to {@link #addAction(Action)} and
	 * returns an {@link ActionList} containing the {@link Action}s in
	 * {@link #buffer}.
	 * 
	 * @return a new {@link ActionList} of the recorded {@link Action}s
	 */
	public static ActionList stop() {
		if (isRecording) {
			timer.stop();
			timer.reset();
			isRecording = false;
			return new ActionList(buffer);
		} else {
			DriverStation.getInstance();
			DriverStation.reportWarning("Tried to stop recording but not started!", false);
			return new ActionList(new ArrayList<Action>());
		}
	}

	/**
	 * Adds an {@link Action} to the {@link #buffer}, to be made into an
	 * {@link ActionList} after recording.
	 * 
	 * @param a
	 *            the {@code Action} to be added
	 */
	public static void addAction(Action a) {
		if (isRecording) {
			buffer.add(a);
		} else {
			DriverStation.getInstance();
			DriverStation.reportWarning("Tried to add action while not recording! Call start() first.", false);
		}
	}

	/**
	 * This method gets the time since listening for calls to
	 * {@link #addAction(Action)} started.
	 * 
	 * @return the time elapsed since recording was started
	 */
	public static double getTime() {
		return timer.get();
	}
	
	/**
	 * @return {@code true} if the {@code ActionRecorder} is currently recording, {@code false} if not
	 */
	public static boolean isRecording() {
		return isRecording;
	}

}