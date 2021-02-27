package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ToggleRecordingCommand extends InstantCommand {
    
    private DriveTrainSubsystem driveTrainSubsystem;

    public ToggleRecordingCommand(DriveTrainSubsystem driveTrainSubsystem) {

        super();
		// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.driveTrainSubsystem = driveTrainSubsystem;
    }

    // Called once when the command executes
	protected void initialize() {
		driveTrainSubsystem.toggleProfileRecording();
	}

}
