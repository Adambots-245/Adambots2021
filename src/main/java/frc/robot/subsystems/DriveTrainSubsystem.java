/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.deser.std.FromStringDeserializer;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.utils.Log;

public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainNew.
   */
  private Solenoid gearShifter;

  private WPI_TalonFX frontRightMotor;
  private WPI_TalonFX frontLeftMotor;
  private WPI_TalonFX backLeftMotor;
  private WPI_TalonFX backRightMotor;

  DifferentialDrive drive;
  static DriveTrainSubsystem driveSubsystem = null;

  private double speedModifier;

  private Gyro gyro;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  private boolean hasGyroBeenReset = false;

  public DriveTrainSubsystem(Gyro gyro, Solenoid gearShifter, WPI_TalonFX frontRightMotor, WPI_TalonFX frontLeftMotor, WPI_TalonFX backLeftMotor, WPI_TalonFX backRightMotor) {
    super();

    this.gyro = gyro;

    speedModifier = Constants.NORMAL_SPEED_MODIFIER;

    this.gearShifter = gearShifter;

    this.frontRightMotor = frontRightMotor; 
    this.frontLeftMotor = frontLeftMotor; 

    this.backLeftMotor = backLeftMotor; 
    this.backRightMotor = backRightMotor; 

    this.backLeftMotor.follow(frontLeftMotor);
    this.backRightMotor.follow(frontRightMotor);

    this.backLeftMotor.setInverted(false);
    frontLeftMotor.setInverted(false);
    backRightMotor.setInverted(false);
    frontRightMotor.setInverted(false);

    frontLeftMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);
    frontRightMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    driveSubsystem = this;

    Log.info("Initializing Drive Subsystem");
  }

  public static DriveTrainSubsystem getCurrentDriveTrain() {
    return driveSubsystem;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {

    Log.info("Resetting encoders");

    frontLeftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVE_PID_SLOT, 0);
    frontLeftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    frontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    // BackLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
    // BackRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public double getAverageDriveEncoderValue() {
    double averageEncoderPos = (Math
        .abs(frontLeftMotor.getSelectedSensorPosition()) + Math.abs(frontRightMotor.getSelectedSensorPosition())) / 2;
    return averageEncoderPos;
  }

  public double getLeftDriveEncoderVelocity() {
    return frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getRightDriveEncoderVelocity() {
    return -frontRightMotor.getSelectedSensorVelocity();
  }

  public double getLeftDriveEncoderPosition() {
    return Math.abs(frontLeftMotor.getSelectedSensorPosition());
  }

  public double getRightDriveEncoderPosition() {
    return Math.abs(frontRightMotor.getSelectedSensorPosition());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftMotor.getSensorCollection().getIntegratedSensorVelocity(), -frontRightMotor.getSensorCollection().getIntegratedSensorVelocity());
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
    SmartDashboard.putNumber("Yaw", gyro.getAngle());

    //Log.infoF("Arcade Drive - Straight Speed = %f - Turn Speed = %f - Gyro Angle = %f", straightSpeed, turnSpeed * speedModifier, gyro.getAngle());
    drive.arcadeDrive(straightSpeed, turnSpeed * speedModifier);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVoltage(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(-leftVolts);
    frontRightMotor.setVoltage(rightVolts);

    System.out.printf("Left Voltage: %f | Right Voltage: %f\n", leftVolts, rightVolts);

    drive.feed();
  }

  public void shiftHighGear() {

    Log.info("Shifting to high gear");
    gearShifter.set(true);
  }

  public void shiftLowGear() {

    Log.info("Shifting to low gear");
    gearShifter.set(false);
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    //Debug odometry
    var translation = odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("odometryX", translation.getX());
    SmartDashboard.putNumber("odometryY", translation.getY());
  
    //Debug voltage
    SmartDashboard.putNumber("leftVoltage", frontLeftMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("rightVoltage", frontRightMotor.getMotorOutputVoltage());
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public void resetGyro(boolean force) {
    if (!hasGyroBeenReset || force) {
      gyro.reset();
      Log.info("Gyro has been reset");
      hasGyroBeenReset = true;
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
    // return gyro.getRotation2d().getDegrees() * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
    // return Math.IEEEremainder(gyro.getRotation2d().getDegrees(), 360) * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    resetGyro(false);
  }

  public void resetGyro(){
    Log.info("Gyro has been reset");

    resetGyro(false);
  }

  /**
   * Resets the odometry with the robot's current heading and starting at a specific pose
   * @param pose - The starting pose
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Updates the odometry with the robot's current heading and encoder positions.
   */
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDriveEncoderPosition(),
      getRightDriveEncoderPosition());
  }

}
