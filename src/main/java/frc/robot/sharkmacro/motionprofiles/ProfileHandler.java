package frc.robot.sharkmacro.motionprofiles;

import frc.robot.sharkmacro.Constants;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Class to easily manage motion profile execution on a number of Talon SRXs. Some logic
 * taken from <a href=
 * "https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionProfile/src/org/usfirst/frc/team217/robot/MotionProfileExample.java">here</a>
 * 
 * @author Alec Minchington
 *
 */
public class ProfileHandler {

	/**
	 * List of the motion profiles to be executed.
	 */
	private double[][][] profiles;

	/**
	 * Represents the current point being streamed from the left profile to the left
	 * talon.
	 */
	private int profileIndex = 0;

	/**
	 * List of PID gains slots to use on respective talons for motion profile
	 * execution.
	 */
	private int[] pidSlotIdxs;

	/**
	 * Talons to be used for motion profile execution.
	 */
	private WPI_TalonFX[] talons;

	/**
	 * Object that takes a runnable class and starts a new thread to call its
	 * {@link java.lang.Runnable#run() run()} method periodically. This instance
	 * will handle {@link PeriodicExecutor}.
	 */
	private Notifier executorThread;

	/**
	 * The current state of the motion profile execution manager.
	 * 
	 * @see ExecutionState
	 */
	private ExecutionState executionState;

	/**
	 * The current state of the talons.
	 * 
	 * @see SetValueMotionProfile
	 */
	private SetValueMotionProfile currentMode;

	/**
	 * Whether the motion profile execution has finished.
	 */
	private boolean finished = false;

	/**
	 * Whether the motion profile execution has started.
	 */
	private boolean started = false;

	/**
	 * List of {@link com.ctre.phoenix.motion.MotionProfileStatus
	 * MotionProfileStatus} objects for each talon.
	 */
	private MotionProfileStatus[] statuses;

	/**
	 * List of trajectory points to be pushed to each talon.
	 */
	private TrajectoryPoint[] trajPoints;

	/**
	 * Constructs a new {@link MotionProfileHandler} object that will handle the
	 * execution of the given motion profiles on their respective talons.
	 * 
	 * @param profiles
	 *            the motion profiles to be executed on their respective talon
	 * @param talons
	 *            the talons to execute the motion profiles on
	 * @param pidSlotIdxs
	 *            the pid profile slots to execute the motion profiles with
	 */
	public ProfileHandler(final double[][][] profiles, WPI_TalonFX[] talons, int[] pidSlotIdxs) {
		this.profiles = profiles;
		this.talons = talons;
		this.pidSlotIdxs = pidSlotIdxs;
		this.executionState = ExecutionState.WAITING;
		this.statuses = new MotionProfileStatus[talons.length];
		for (int i = 0; i < statuses.length; i++) {
			statuses[i] = new MotionProfileStatus();
		}
		this.trajPoints = new TrajectoryPoint[talons.length];
		for (int i = 0; i < trajPoints.length; i++) {
			trajPoints[i] = new TrajectoryPoint();
		}

		bufferThread = new Notifier(new PeriodicBufferProcessor());
		bufferThread.startPeriodic(Constants.DT_SECONDS / 2.0);

		for (int i = 0; i < talons.length; i++) {
			this.talons[i].changeMotionControlFramePeriod(Constants.MOTIONCONTROL_FRAME_PERIOD);
		}

		executorThread = new Notifier(new PeriodicExecutor());
		
		fillTalonsWithMotionProfile();
		DriverStation.getInstance();
		DriverStation.reportWarning("PROFILE LOADED", false);
	}

	/**
	 * Called to start the execution of the motion profile.
	 */
	public void execute() {
		executorThread.startPeriodic(0.005);
		started = true;
	}

	/**
	 * Called if the motion profile execution needs to be prematurely stopped.
	 */
	public void onInterrupt() {
		bufferThread.stop();
		executorThread.stop();
		setMode(SetValueMotionProfile.Disable);
	}

	/**
	 * Called after motion profile execution has finished.
	 */
	private void onFinish() {
		finished = true;
		bufferThread.stop();
		executorThread.stop();
		setMode(SetValueMotionProfile.Disable);
		for (int i = 0; i < talons.length; i++) {
			talons[i].clearMotionProfileTrajectories();
		}
	}

	/**
	 * Called periodically while the motion profile is being executed. Manages the
	 * state of the Talons executing the motion profiles.
	 */
	public void manage() {
		fillTalonsWithMotionProfile();
		updateMotionProfilesStatuses();

		boolean readyToProgress = true;

		switch (executionState) {
		case WAITING:
			if (started) {
				started = false;
				setMode(SetValueMotionProfile.Disable);
				executionState = ExecutionState.STARTED;
			}
			break;
		case STARTED:
			for (int i = 0; i < statuses.length; i++) {
				if (statuses[i].btmBufferCnt <= Constants.MINIMUM_POINTS_IN_TALON) {
					readyToProgress = false;
				}
			}
			if (readyToProgress) {
				setMode(SetValueMotionProfile.Enable);
				executionState = ExecutionState.EXECUTING;
			}
			break;
		case EXECUTING:
			readyToProgress = true;
			for (int i = 0; i < statuses.length; i++) {
				if (!statuses[i].activePointValid || !statuses[i].isLast) {
					readyToProgress = false;
				}
			}
			if (readyToProgress) {
				onFinish();
			}
			break;
		}
	}

	/**
	 * Sets the state of the talons.
	 * 
	 * @param t
	 *            the motion profile mode to set the Talon to
	 */
	private void setMode(SetValueMotionProfile mode) {
		this.currentMode = mode;
		for (int i = 0; i < talons.length; i++) {
			talons[i].set(ControlMode.MotionProfile, mode.value);
		}
	}

	/**
	 * Fill the talons' top-level buffer with a given motion profile.
	 * 
	 */
	private void fillTalonsWithMotionProfile() {

		if (profileIndex == 0) {
			for (int i = 0; i < talons.length; i++) {
				talons[i].clearMotionProfileTrajectories();
				talons[i].configMotionProfileTrajectoryPeriod(0);
				talons[i].clearMotionProfileHasUnderrun(0);
			}
		}

		updateMotionProfilesStatuses();

		int maxFilled = statuses[0].topBufferCnt;
		for (int i = 0; i < statuses.length; i++) {
			if (statuses[i].topBufferCnt > maxFilled) {
				maxFilled = statuses[i].topBufferCnt;
			}
		}

		int numPointsToFill = Constants.TALON_TOP_BUFFER_MAX_COUNT - maxFilled;

		boolean finished = false;

		while (!finished && numPointsToFill > 0) {
			for (int i = 0; i < trajPoints.length; i++) {

				if (profileIndex >= profiles[i].length) {
					finished = true;
					break;
				}

				trajPoints[i].position = profiles[i][profileIndex][0];
				trajPoints[i].velocity = profiles[i][profileIndex][1];
				trajPoints[i].headingDeg = 0;
				trajPoints[i].timeDur = (int) profiles[i][profileIndex][2];
				trajPoints[i].profileSlotSelect0 = pidSlotIdxs[i];
				trajPoints[i].profileSlotSelect1 = 0;

				trajPoints[i].zeroPos = false;
				if (profileIndex == 0) {
					trajPoints[i].zeroPos = true;
				}

				trajPoints[i].isLastPoint = false;
				if ((profileIndex + 1) == profiles[i].length) {
					trajPoints[i].isLastPoint = true;
				}
			}

			for (int i = 0; i < trajPoints.length; i++) {
				talons[i].pushMotionProfileTrajectory(trajPoints[i]);
			}

			profileIndex++;
			numPointsToFill--;
		}

		updateMotionProfilesStatuses();
	}

	/**
	 * Updates the {@link com.ctre.phoenix.motion.MotionProfileStatus
	 * MotionProfileStatus} objects of each talon.
	 */
	public void updateMotionProfilesStatuses() {
		for (int i = 0; i < talons.length; i++) {
			talons[i].getMotionProfileStatus(statuses[i]);
		}
	}

	/**
	 * Converts an integer time value into a
	 * {@link com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration
	 * TrajectoryDuration}.
	 * 
	 * @param durationMs
	 *            time duration of the trajectory
	 * @return {@code TrajectoryDuration} with the value of the passed duration
	 */
	// private TrajectoryDuration toTrajectoryDuration(int durationMs) {
	// 	TrajectoryDuration dur = TrajectoryDuration.Trajectory_Duration_0ms;
	// 	dur = TrajectoryDuration.valueOf(durationMs);
	// 	if (dur.value != durationMs) {
	// 		DriverStation.getInstance();
	// 		DriverStation.reportError(
	// 				"Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);
	// 	}
	// 	return dur;
	// }

	/**
	 * @return the {@link com.ctre.CANTalon.MotionProfileStatus
	 *         CANTalon.MotionProfileStatus} objects of each of the talons.
	 */
	public MotionProfileStatus[] getStatus() {
		return statuses;
	}

	/**
	 * @return the {@link TalonState} representing the talons' current state
	 */
	public SetValueMotionProfile getMode() {
		return this.currentMode;
	}

	/**
	 * @return {@code true} when the motion profile is finished executing,
	 *         {@code false} otherwise
	 */
	public boolean isFinished() {
		return finished;
	}

	/**
	 * Class to periodically call
	 * {@link com.ctre.CANTalon#processMotionProfileBuffer()
	 * processMotionProfileBufffer()} for {@link ProfileHandler#leftTalon} and
	 * {@link ProfileHandler#rightTalon}.
	 */
	class PeriodicBufferProcessor implements java.lang.Runnable {
		public void run() {
			for (int i = 0; i < talons.length; i++) {
				if (statuses[i].btmBufferCnt < Constants.TALON_BTM_BUFFER_MAX_COUNT) {
					talons[i].processMotionProfileBuffer();
				}
			}
		}
	}

	/**
	 * Object that takes a runnable class and starts a new thread to call its
	 * {@link java.lang.Runnable#run() run()} method periodically. This instance
	 * will handle {@link PeriodicBufferProcessor}.
	 */
	Notifier bufferThread;

	/**
	 * Class to periodically call {@link ProfileHandler#manage()}.
	 */
	class PeriodicExecutor implements java.lang.Runnable {
		public void run() {
			manage();
		}
	}

}

/**
 * Enum to help manage the current state of the motion profile execution.
 * 
 * @author Alec Minchington
 *
 */
enum ExecutionState {

	/**
	 * The possible states of motion profile execution.
	 * <p>
	 * {@link #WAITING}: The manager is waiting for motion profile execution to
	 * start
	 * <p>
	 * {@link #STARTED}: The manager starts the execution when the Talon's buffer is
	 * sufficiently filled
	 * <p>
	 * {@link #EXECUTING}: The manager waits for execution to finish and then safely
	 * exits the execution process
	 */
	WAITING, STARTED, EXECUTING;
}
