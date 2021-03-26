package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.*;

public class DriveToBallCommand extends CommandBase {
    
    private NetworkTable table;
    private boolean closeToBall = false;
    private DriveTrainSubsystem driveTrainSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double distanceFromIntakeArm;
    private double calculatedDistance;
    private boolean intakeDetectedBall = false;
    private PhotoEye intakePhotoEye;

    public DriveToBallCommand(DriveTrainSubsystem driveTrainSubsystem, PhotoEye intakePhotoEye) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.intakePhotoEye = intakePhotoEye;
        addRequirements(driveTrainSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetEncoders();
        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);

        // calculate distance
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
        if(driveTrainSubsystem.getAverageDriveEncoderValue() >= ( calculatedDistance * Constants.ENCODER_TICKS_PER_INCH ) + 70000)
            closeToBall = true;
        else if (intakePhotoEye.isDetecting())
             intakeDetectedBall = true;
        System.out.println("Intake Detected Ball: " + intakeDetectedBall + ". Close to Ball: " + closeToBall);
        return closeToBall || intakeDetectedBall;
    }

}
