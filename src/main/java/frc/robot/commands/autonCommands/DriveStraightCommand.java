// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraightCommand extends PIDCommand {

  private static final double kP = Constants.GYRO_kP;
  private static final double kI = Constants.GYRO_kI;
  private static final double kD = Constants.GYRO_kD;

  private final DriveTrainSubsystem driveTrain;
  private final double distance;
  
  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(DriveTrainSubsystem drive, double speed, double distance) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drive::getTurnRate,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drive.arcadeDrive(speed, output);
        },
        drive);

        this.driveTrain = drive;
        this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

@Override
public void initialize() {
  super.initialize();

  driveTrain.resetEncoders();

  System.out.println("Initialize - Heading:"+ driveTrain.getHeading());
  double relativeSetPoint = driveTrain.getHeading(); // stay at whatever angle is current value

  // relativeSetPoint = Math.signum(this.targetAngle) * relativeSetPoint;
  getController().setSetpoint(relativeSetPoint);
  System.out.println("Initialize - SetPoint:"+ relativeSetPoint);

  // driveTrain.resetGyro(true);

  // System.out.println("Heading after reset: " + driveTrain.getHeading());
  // System.out.println("Yaw after reset: " + Gyro.getInstance().getYaw());
  // System.out.println("Angle after reset: " + Gyro.getInstance().getAngle());

}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getAverageDriveEncoderValue() >= distance);
  }
}
