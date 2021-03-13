// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TurnCommand extends PIDCommand {

  private DriveTrainSubsystem driveTrain;

  private static final double kP = Constants.GYRO_kP;
  private static final double kI = Constants.GYRO_kI;
  private static final double kD = Constants.GYRO_kD;

  private double targetAngle = 0;

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
          // Set minimum output to turn the robot - anything lower than this and it may
          // not move
          double rotationSpeed = MathUtil.clamp(Math.abs(output), 0.9, 1.5);

          // if (Math.abs(output) > 1.5) rotationSpeed = 1.5;

          // double rotationSpeed = output;
          rotationSpeed = rotationSpeed * Math.signum(output); // apply the sign (positive or negative)
          System.out.println("O:H:S => " + output + " : " + drive.getHeading() + " : " + rotationSpeed);
          drive.arcadeDrive(0, rotationSpeed);
        }, drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    // addRequirements(driveTrain);

    this.driveTrain = drive;
    this.targetAngle = targetAngleDegrees;

    SmartDashboard.putData("Turn PID Controller", getController());
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.GYRO_TOLERANCE); // , Constants.GYRO_RATE_TOLERANCE_DEG_PER_SEC);
  }

  @Override
  public void initialize() {
    super.initialize();

    driveTrain.resetEncoders();
    driveTrain.resetGyro(true);

    System.out.println("Initialize - Heading:"+ driveTrain.getHeading());
    double relativeSetPoint = Math.IEEEremainder(driveTrain.getHeading() + this.targetAngle, 180);
    relativeSetPoint = Math.signum(this.targetAngle) * relativeSetPoint;
    getController().setSetpoint(relativeSetPoint);
    System.out.println("Initialize - SetPoint:"+ relativeSetPoint);


    // driveTrain.resetGyro(true);
    // Gyro.getInstance().lowLevelReset();
    // try {
    //   Thread.sleep(600);
    // } catch (InterruptedException e) {
    //   e.printStackTrace();
    // }

    // System.out.println("Heading after reset: " + driveTrain.getHeading());
    // System.out.println("Yaw after reset: " + Gyro.getInstance().getYaw());
    // System.out.println("Angle after reset: " + Gyro.getInstance().getAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("At Setpoint:" + getController().atSetpoint() + " Heading: " + driveTrain.getHeading());

    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    System.out.println("At End:" + "Heading: " + driveTrain.getHeading());
    // driveTrain.arcadeDrive(0,0);

    // driveTrain.zeroHeading();
  }
}
