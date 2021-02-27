package frc.robot.sharkmacro.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Serves as the base {@link edu.wpi.first.wpilibj.command.Command Command}
 * class for SharkMacro. All robot commands to be recorded should extend this
 * instead of {@code Command}.
 * 
 * @author Alec Minchington
 *
 */
public class RecordableCommand extends Command {

	/**
	 * Time this command is started.
	 */
	private double startTime;

	/**
	 * {@code true} if this command is being played back from an {@link ActionList},
	 * {@code false} otherwise.
	 */
	protected boolean isPlayback = false;

	/*
	 * Constructs a new RecordableCommand object.
	 */
	public RecordableCommand() {
	}

	/**
	 * Calls {@link #setTimeout(double)}.
	 * 
	 * @param sec
	 *            the timeout
	 */
	public void setTimeoutSeconds(double sec) {
		setTimeout(sec);
	}

	/**
	 * Called once when this command is started. If this command is being recorded
	 * ({@link #isPlayback} {@code = false}), {@link #startTime} is set to the
	 * current time.
	 */
	@Override
	protected void initialize() {
		if (!isPlayback) {
			startTime = ActionRecorder.getTime();
		}
		System.out.println(this.getClass().getName() + " started at " + Timer.getFPGATimestamp());
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	/**
	 * Called once when {@link #isFinished()} returns {@code true}. If this command
	 * is being recorded ({@link #isPlayback} {@code = false}), the end time is
	 * recorded and a new {@link Action} representing the command is passed to
	 * {@link ActionRecorder}.
	 */
	@Override
	protected void end() {
		System.out.println(this.getClass().getName() + " ended at " + Timer.getFPGATimestamp());
		if (!isPlayback) {
			ActionRecorder.addAction(new Action(this.getClass().getName(), startTime, ActionRecorder.getTime()));
		}
	}

	@Override
	protected void interrupted() {
		end();
	}
}