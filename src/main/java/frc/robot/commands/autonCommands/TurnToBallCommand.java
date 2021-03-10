package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurnToBallCommand extends CommandBase {
    
    //private DriveTrainSubsystem driveTrainSubsystem;
    private boolean ballDetected;
    private NetworkTable table;

    public TurnToBallCommand(DriveTrainSubsystem driveTrainSubsystem) {
        super();
        //this.driveTrainSubsystem = driveTrainSubsystem;
        //addRequirements(driveTrainSubsystem);
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        table = instance.getTable("limelight");
    }

    @Override
    public void initialize() {
        double objectDetectedValue = table.getEntry("tv").getDouble(0);
        if(objectDetectedValue == 1.0)
            ballDetected = true;
        System.out.println("\nTurnToBallCommand.initialize() { ballDetected: " + ballDetected + "}\n");
    }

    @Override
    public void execute() {

    }
}
