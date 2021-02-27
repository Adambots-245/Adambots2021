package frc.robot.sharkmacro.actions;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Models an action of a robot. This class's {@link #start()} method can be
 * called to add the {@link edu.wpi.first.wpilibj.command.Command Command}
 * associated with it to the command scheduler.
 * 
 * A list of these actions makes up an {@link ActionList}, which forms the basis
 * for playing back {@code Command}s autonomously.
 * 
 * @author Alec Minchington
 *
 */
public final class Action {

	/**
	 * The absolute class name of the {@link edu.wpi.first.wpilibj.command.Command
	 * Command} this {@link Action} represents.
	 */
	private final String commandName;

	/**
	 * The time this {@link Action} is scheduled to start.
	 */
	private final double startTime;

	/**
	 * The time this {@link Action} is scheduled to end.
	 */
	private final double endTime;

	/**
	 * Constructs a new {@link Action}.
	 * 
	 * @param commandName
	 *            the absolute class name of a
	 *            {@link edu.wpi.first.wpilibj.command.Command Command}
	 * @param startTime
	 *            time this {@link Action} will be started
	 * @param endTime
	 *            time this {@code Action} will end
	 */
	public Action(String commandName, double startTime, double endTime) {
		this.commandName = commandName;
		this.startTime = Math.round(startTime * 1000.0) / 1000.0;
		this.endTime = Math.round(endTime * 1000.0) / 1000.0;
	}

	/**
	 * Execute this {@link Action}. This is done by using reflection to create a new
	 * instance of {@link edu.wpi.first.wpilibj.command.Command Command}'s
	 * superclass, {@link RecordableCommand} and calling its
	 * {@link edu.wpi.first.wpilibj.command.Command#start() start()} method.
	 * 
	 */
	public void start() {
		try {
			RecordableCommand rc = (RecordableCommand) Class.forName(commandName).getConstructor().newInstance();
			rc.setTimeoutSeconds(endTime - startTime);
			rc.isPlayback = true;
			rc.start();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * This method returns the {@link java.lang.String String} array representation
	 * of this {@link Action}. The representation is structured as follows:
	 * <p>
	 * <center> {@code [} {@link #commandName} {@code ,} {@link #startTime}
	 * {@code ,} {@link #endTime} {@code ]} </center>
	 * </p>
	 * 
	 * @return the {@code String[]} representation of this {@code Action}.
	 */
	public String[] toStringArray() {
		@SuppressWarnings("serial")
		ArrayList<String> list = new ArrayList<String>() {
			{
				add(commandName);
				add(Double.toString(startTime));
				add(Double.toString(endTime));
			}
		};
		return Arrays.toString(list.toArray()).replace("[", "").replace("]", "").replace(" ", "").split(",");
	}

	/**
	 * Gets the {@link #commandName} property of this {@link #Action}.
	 * 
	 * @return the absolute class name of the
	 *         {@link edu.wpi.first.wpilibj.command.Command Command}
	 */
	public String getCommandName() {
		return this.commandName;
	}

	/**
	 * Gets the {@link #startTime} property of this {@link #Action}.
	 * 
	 * @return the time this {@code Action} is scheduled to start
	 */
	public double getStartTime() {
		return this.startTime;
	}

	/**
	 * Gets the {@link #endTime} property of this {@link #Action}.
	 * 
	 * @return the time this {@code Action} is scheduled to end
	 */
	public double getEndTime() {
		return this.endTime;
	}

	/**
	 * Generates a string representation of this {@link Action}.
	 * 
	 * @return the string representation
	 */
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(commandName);
		sb.append("\n");
		sb.append(startTime);
		sb.append("\n");
		sb.append(endTime);
		return sb.toString();
	}
}