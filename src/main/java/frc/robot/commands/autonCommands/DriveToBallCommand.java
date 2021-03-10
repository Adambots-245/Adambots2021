package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DriveToBallCommand extends CommandBase {
    
    private NetworkTable table;
    private boolean closeToBall = false;

    public DriveToBallCommand() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void initialize() {
        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);
        
        // calculate distance and drive if too far
        double distance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(verticalDegreesToCenter + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL);
        if(distance > Constants.ACCEPTABLE_FINAL_DISTANCE) {
            System.out.println("DRIVING TOWARD BALL. Distance: " + distance);
        } else {
            System.out.println("CLOSE ENOUGH TO BALL");
            closeToBall = true;
        }
    }

    @Override
    public void execute() {
        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);

        // calculate distance and drive if too far
        double distance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(verticalDegreesToCenter + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL);
        if(distance > Constants.ACCEPTABLE_FINAL_DISTANCE) {
            System.out.println("DRIVING TOWARD BALL. Distance: " + distance);
        } else {
            System.out.println("CLOSE ENOUGH TO BALL");
            closeToBall = true;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return closeToBall;
    }

}
