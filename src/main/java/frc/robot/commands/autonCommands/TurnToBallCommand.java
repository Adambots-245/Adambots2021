package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToBallCommand extends CommandBase {
    
    //private DriveTrainSubsystem driveTrainSubsystem;
    private boolean ballDetected;
    private boolean aimedAtBall = false;
    private NetworkTable table;
    private DriveTrainSubsystem driveTrainSubsystem;

    public TurnToBallCommand(DriveTrainSubsystem driveTrainSubsystem) {
        super();
        //this.driveTrainSubsystem = driveTrainSubsystem;
        //addRequirements(driveTrainSubsystem);
        table = NetworkTableInstance.getDefault().getTable("limelight");
        System.out.println("TurnToBallCommand Constructor, Table: " + table.toString());
        this.driveTrainSubsystem = driveTrainSubsystem;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        double objectDetectedValue = table.getEntry("tv").getDouble(0);
        if((int) objectDetectedValue == 1) {
            ballDetected = true;
        } else {
            //begin scan for ball. turn 90 degrees, turn back (relatively) slowly
        }
    }

    @Override
    public void execute() {
        // Determine whether to aim left or right
        double horizontalDegreesToCenter = table.getEntry("tx").getDouble(0);
        if(horizontalDegreesToCenter > -1 && horizontalDegreesToCenter < 1) { // if aiming within acceptable window
            System.out.println("\n\nDRIVING FORWARD\n\n");
            aimedAtBall = true;
        } else { // ONLY run this when the robot is NOT aimed at the ball

        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return aimedAtBall;
    }
}
