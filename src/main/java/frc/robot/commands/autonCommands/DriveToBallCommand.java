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
    private DriveTrainSubsystem driveTrainSubsystem;
    private double distanceFromIntakeArm;
    private double calculatedDistance;

    public DriveToBallCommand(DriveTrainSubsystem driveTrainSubsystem) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.driveTrainSubsystem = driveTrainSubsystem;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetEncoders();

        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);

        // calculate distance and drive if too far
        calculatedDistance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(Math.abs(verticalDegreesToCenter) * (Math.PI / 180.0) + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL * (Math.PI / 180.0));
        distanceFromIntakeArm = calculatedDistance - Constants.LIMELIGHT_DISTANCE_TO_INTAKE_ARM;
        SmartDashboard.putNumber("Calculated Distance to Ball", calculatedDistance);
        SmartDashboard.putNumber("Distance from Intake Arm", distanceFromIntakeArm);
    }

    @Override
    public void execute() {
        
        // if(distanceFromIntakeArm > Constants.ACCEPTABLE_FINAL_DISTANCE) {
        //     System.out.println("DRIVING TOWARD BALL. Distance: " + calculatedDistance);
        //     driveTrainSubsystem.arcadeDrive(-Constants.AUTON_DRIVE_FORWARD_SPEED, 0.0);
        // } else {
        //     System.out.println("CLOSE ENOUGH TO BALL");
        //     driveTrainSubsystem.arcadeDrive(0.0, 0.0);
        //     closeToBall = true;
        // }
        driveTrainSubsystem.arcadeDrive(-Constants.AUTON_DRIVE_FORWARD_SPEED, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.arcadeDrive(0.0, 0.0);
        if(interrupted)
            System.out.println("DRIVE TO BALL COMMAND INTERRUPTED");
        else
            System.out.println("DRIVE TO BALL COMMAND ENDED");
    }

    @Override
    public boolean isFinished() {
        System.out.println("Drive encoder values:" + driveTrainSubsystem.getAverageDriveEncoderValue());
        return (driveTrainSubsystem.getAverageDriveEncoderValue() >= calculatedDistance * Constants.ENCODER_TICKS_PER_INCH);
    }

}
