package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToBallCommand extends CommandBase {
    
    private NetworkTable table;
    private boolean closeToBall = false;

    public DriveToBallCommand() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);

        // calculate distance and drive if too far
        double distance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(Math.abs(verticalDegreesToCenter) * (Math.PI / 180.0) + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL * (Math.PI / 180.0));
        SmartDashboard.putNumber("Calculated Distance to Ball", distance);
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
