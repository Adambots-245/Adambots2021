/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainNew.
   */
  private Solenoid GearShifter;

  private WPI_TalonFX FrontRightMotor;
  private WPI_TalonFX FrontLeftMotor;
  private WPI_TalonFX BackLeftMotor;
  private WPI_TalonFX BackRightMotor;

  DifferentialDrive drive;

  private double speedModifier;

  private GyroSubsystem gyroSubsystem;

  private boolean hasGyroBeenReset = false;

  public DriveTrainSubsystem(GyroSubsystem gyroSubsystem) {
    super();

    this.gyroSubsystem = gyroSubsystem;

    speedModifier = Constants.NORMAL_SPEED_MODIFIER;

    GearShifter = new Solenoid(Constants.HIGH_GEAR_SOL_PORT);

    FrontRightMotor = new WPI_TalonFX(Constants.FR_TALON);
    FrontLeftMotor = new WPI_TalonFX(Constants.FL_TALON);

    BackLeftMotor = new WPI_TalonFX(Constants.BL_TALON);
    BackRightMotor = new WPI_TalonFX(Constants.BR_TALON);

    BackLeftMotor.follow(FrontLeftMotor);
    BackRightMotor.follow(FrontRightMotor);

    BackLeftMotor.setInverted(false);
    FrontLeftMotor.setInverted(false);
    BackRightMotor.setInverted(false);
    FrontRightMotor.setInverted(false);

    FrontLeftMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);
    FrontRightMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);

    drive = new DifferentialDrive(FrontLeftMotor, FrontRightMotor);
  }

  public void resetEncoders() {
    FrontLeftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVE_PID_SLOT, 0);
    FrontLeftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    FrontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    // BackLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
    // BackRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }

  // public double getRightDriveEncoderValue(){
  //   FrontRightMotor.getSelectedSensorPosition()
  // }
  public double getAverageDriveEncoderValue() {
    double averageEncoderPos = (Math
        .abs(FrontLeftMotor.getSelectedSensorPosition()) + Math.abs(FrontRightMotor.getSelectedSensorPosition()) / 2);
    // System.out.println(averageEncoderPos);
    return averageEncoderPos;
  }

  public double getLeftDriveEncoderVelocity() {
    return FrontLeftMotor.getSelectedSensorVelocity();
  }

  public double getRightDriveEncoderVelocity() {
    return FrontRightMotor.getSelectedSensorVelocity();
  }

  public void setLowSpeed() {
    speedModifier = Constants.LOW_SPEED_MODIFIER;
  }

  public void setNormalSpeed() {
    speedModifier = Constants.NORMAL_SPEED_MODIFIER;
  }

  public void arcadeDrive(double speed, double turnSpeed) {
    int frontRobotDirection = -1;
    double straightSpeed = frontRobotDirection * speed * speedModifier;
    // System.out.println("straightSpeed = " + straightSpeed);
    // System.out.println("turnSpeed = " + turnSpeed * speedModifier);

    // double leftSpeed = Math.min(straightSpeed + turnSpeed* speedModifier,
    // Constants.MAX_MOTOR_SPEED);
    // double rightSpeed = Math.min(straightSpeed - turnSpeed* speedModifier,
    // Constants.MAX_MOTOR_SPEED);

    // FrontRightMotor.set(ControlMode.PercentOutput, rightSpeed);
    // FrontLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
    // FrontLeftMotor.setExpiration(.5);
    // FrontRightMotor.setExpiration(.5);
    // System.out.println(FrontLeftMotor.getExpiration());
    SmartDashboard.putNumber("Yaw", gyroSubsystem.getYaw());

    drive.arcadeDrive(straightSpeed, turnSpeed * speedModifier);
  }

  public void shiftHighGear() {
    GearShifter.set(true);
  }

  public void shiftLowGear() {
    GearShifter.set(false);
  }
  /*
   * public void driveDistance(double distance){
   * FrontLeftMotor.set(ControlMode.Position, distance);
   * FrontRightMotor.set(ControlMode.Position, distance); }
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Double getAngle() {
    return (double) gyroSubsystem.getYaw();
  }

  public void resetGyro(boolean force) {
    if (!hasGyroBeenReset || force) {
      gyroSubsystem.reset();
      hasGyroBeenReset = true;
    }
  }

  public void resetGyro(){
    resetGyro(false);
  }
}
