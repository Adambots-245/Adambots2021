package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.Log;

public class ToggleRecordingCommand extends Command {
    
    private DriveTrainSubsystem driveTrainSubsystem;

    public ToggleRecordingCommand(DriveTrainSubsystem driveTrainSubsystem) {

        super();
		// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.driveTrainSubsystem = driveTrainSubsystem;
    }

    // Called once when the command executes
	protected void initialize() {
        super.initialize();
		driveTrainSubsystem.toggleProfileRecording();
    }
    
    public boolean isFinished() {
        Log.info("ToggleRecordingCommand.isFinished()");
        return true;
    }

}
