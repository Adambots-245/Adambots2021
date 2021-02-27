package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.Log;

public class ToggleActionRecordingCommand extends CommandBase {
    
    private DriveTrainSubsystem driveTrainSubsystem;

    public ToggleActionRecordingCommand(DriveTrainSubsystem driveTrainSubsystem) {
        super();
        this.driveTrainSubsystem = driveTrainSubsystem;
    }

    @Override
	public void initialize() {
    }

    @Override
    public void execute() {
        driveTrainSubsystem.toggleProfileRecording();
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        Log.info("ToggleActionRecordingCommand.isFinished()!!!!!!!!!!!!!!!!!!!!!!");
        return true;
    }
}
