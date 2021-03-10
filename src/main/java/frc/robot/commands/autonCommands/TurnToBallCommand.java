package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurnToBallCommand extends CommandBase {
    
    //private DriveTrainSubsystem driveTrainSubsystem;
    private boolean ballDetected;
    private boolean aimedAtBall = false;
    private NetworkTable table;

    public TurnToBallCommand() {
        super();
        //this.driveTrainSubsystem = driveTrainSubsystem;
        //addRequirements(driveTrainSubsystem);
        table = NetworkTableInstance.getDefault().getTable("limelight");
        System.out.println("TurnToBallCommand Constructor, Table: " + table.toString());
    }

    @Override
    public void initialize() {
        double objectDetectedValue = table.getEntry("tv").getDouble(0);
        if((int) objectDetectedValue == 1)
            ballDetected = true;
        else {
            // if a ball is not detected, make a scan for one
        }
        
        double distanceToCenter = table.getEntry("tx").getDouble(0);
        if(distanceToCenter > -1 && distanceToCenter < 1) { // if aiming within acceptable window
            System.out.println("\n\nDRIVING FORWARD\n\n");
            aimedAtBall = true;
        } else if (distanceToCenter < -1) {
            System.out.println("\n\nTURNING LEFT TOWARD BALL\n\n");
        } else if (distanceToCenter > 1) {
            System.out.println("\n\nTURNING RIGHT TOWARD BALL\n\n");
        }
    }

    @Override
    public void execute() {
        double distanceToCenter = table.getEntry("tx").getDouble(0);
        if(distanceToCenter > -1 && distanceToCenter < 1) { // if aiming within acceptable window
            System.out.println("\n\nDRIVING FORWARD\n\n");
            aimedAtBall = true;
        } else if (distanceToCenter < -1) { // if robot is aiming too far to the right
            System.out.println("\n\nTURNING LEFT TOWARD BALL\n\n");
        } else if (distanceToCenter > 1) { // if robot is aiming too far to the left
            System.out.println("\n\nTURNING RIGHT TOWARD BALL\n\n");
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
