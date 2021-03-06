// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TurnCommand extends PIDCommand {

  private DriveTrainSubsystem driveTrain;

  private static final double kP = 0.0481; //Constants.GYRO_kP;
  private static final double kI = 0; //Constants.GYRO_kI;
  private static final double kD = 0; //0.01261; //0.0012;//Constants.GYRO_kD;

  /** Creates a new TurnCommand. */
  public TurnCommand(double targetAngleDegrees, DriveTrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> {
          // Set minimum output to turn the robot - anything lower than this and it may not move
          double rotationSpeed = MathUtil.clamp(Math.abs(output), 0.325, 1.00);
          rotationSpeed = rotationSpeed * Math.signum(output); // apply the sign (positive or negative)
          System.out.println("Heading - Output: " + drive.getHeading() + " - " + rotationSpeed);
          drive.arcadeDrive(0, rotationSpeed);
        },
        drive
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    this.driveTrain = drive;

    drive.resetGyro();

    SmartDashboard.putData("Turn PID Controller", getController());
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.GYRO_TOLERANCE, Constants.GYRO_RATE_TOLERANCE_DEG_PER_SEC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("At Setpoint:" + getController().atSetpoint());
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted){
    super.end(interrupted);

    driveTrain.zeroHeading();
  }
}
