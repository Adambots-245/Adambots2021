package frc.robot.commands.autonCommands;

import frc.robot.sharkmacro.actions.ActionList;
import frc.robot.sharkmacro.actions.ActionListParser;
import frc.robot.sharkmacro.motionprofiles.Profile;
import frc.robot.sharkmacro.motionprofiles.ProfileParser;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExecuteMacroCommand extends CommandBase {
    
    private Profile profileToExecute;
	private ActionList actionListToExecute;
	private boolean hasName = false;
	private String name;
	private String alName;

    private WPI_TalonFX[] talons;
    private DriveTrainSubsystem driveTrainSubsystem;
    
    public ExecuteMacroCommand(DriveTrainSubsystem driveTrainSubsystem) {

        this.driveTrainSubsystem = driveTrainSubsystem;

    }

    public ExecuteMacroCommand(DriveTrainSubsystem driveTrainSubsystem, String name) {

        this.driveTrainSubsystem = driveTrainSubsystem;

        this.name = name;
    	this.alName = name;
    	this.hasName = true;

    }

    public void initialize() {
    	ProfileParser pParser;
		ActionListParser alParser;
		
		name = (name == null) ? ProfileParser.getNewestFilename() : name;
		//alName = (alName == null) ? ActionListParser.getNewestFilename() : alName;

		
    	pParser = new ProfileParser(name);
		//alParser = new ActionListParser(alName);
		System.out.println("Executing profile... " + name);
		//System.out.println("Executing ActionList... " + alName);
		
		profileToExecute = pParser.toObject(talons[0], talons[1], Constants.DRIVE_PID_SLOT, Constants.DRIVE_PID_SLOT);
		//actionListToExecute = alParser.toObject();
		profileToExecute.execute();
		//actionListToExecute.execute();
		
		name = hasName ? name : null;
		//alName = hasName ? alName : null;
    }

    public void execute() {
    }

    public boolean isFinished() {
        return profileToExecute.isFinished() && actionListToExecute.isFinished();
    }

    public void end() {

    }

    public void interrupted() {
        profileToExecute.onInterrupt();
    	actionListToExecute.onInterrupt();
    }

}
