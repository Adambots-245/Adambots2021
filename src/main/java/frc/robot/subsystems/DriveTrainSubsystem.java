/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

  private final DifferentialDriveKinematics kDriveKinematics = Constants.kDriveKinematics;
  private final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(Constants.ksVolts,
  Constants.kvVoltSecondsPerMeter,
  Constants.kaVoltSecondsSquaredPerMeter);
  private final PIDController kLeftPidController = new PIDController(Constants.kPDriveVel, 0, 0);
  private final PIDController kRightPidController = new PIDController(Constants.kPDriveVel, 0, 0);

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
    backRightMotor.setInverted(true);
    frontRightMotor.setInverted(true);

    frontLeftMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);
    frontRightMotor.configOpenloopRamp(Constants.SEC_NEUTRAL_TO_FULL);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    drive.setRightSideInverted(false);

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
    return frontRightMotor.getSelectedSensorVelocity();
  }

  public double getLeftDriveEncoderPosition() {
    return frontLeftMotor.getSelectedSensorPosition();
  }

  public double getRightDriveEncoderPosition() {
    return frontRightMotor.getSelectedSensorPosition();
  }

  public double getLeftDriveEncoderMeters() {
    //position = distanceInches * ENCODER_TICKS
    //so: position / ENCODER_TICKS = distanceInches

    double inches = getLeftDriveEncoderPosition() / Constants.ENCODER_TICKS_PER_INCH;
    return Units.inchesToMeters(inches);

  }

  public double getRightDriveEncoderMeters() {
    double inches = getRightDriveEncoderPosition() / Constants.ENCODER_TICKS_PER_INCH;
    return Units.inchesToMeters(inches);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftDriveEncoderVelocity() / Constants.ENCODER_TICKS_PER_INCH, getRightDriveEncoderVelocity() / Constants.ENCODER_TICKS_PER_INCH);
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
  public void setTankVoltage(double leftVolts, double rightVolts) {
    // frontLeftMotor.setVoltage(leftVolts);
    // frontLeftMotor.set(ControlMode.PercentOutput, -leftVolts / 12 * 25);
    frontLeftMotor.set(ControlMode.PercentOutput, leftVolts / 12 * 25);
    frontRightMotor.set(ControlMode.PercentOutput, rightVolts / 12 * 25);

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

  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(-180)));
  }

  /**
   * Updates the odometry with the robot's current heading and encoder positions.
   */
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDriveEncoderMeters(),
      getRightDriveEncoderMeters());
  }

  /**
   * Get differential kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return kDriveKinematics;
  }

  /**
   * Forward motor feed
   */
  public SimpleMotorFeedforward getForwardFeed() {
    return kFeedforward;
  }

  /**
   * Log forward motor feed to SmartDashboard
   * @return
   */
  public void logForwardFeedValues() {
    double leftWheelSpeed = getForwardFeed().calculate(getWheelSpeeds().leftMetersPerSecond);
    double rightWheelSpeed = getForwardFeed().calculate(getWheelSpeeds().rightMetersPerSecond);


    SmartDashboard.putNumber("leftWheelSpeed", leftWheelSpeed);
    SmartDashboard.putNumber("rightWheelSpeed", rightWheelSpeed);

  }

  /**
   * Left PID Controller
   */
  public PIDController getLeftPIDController() {
    return kLeftPidController;
  }

  /**
   * Left PID Controller
   */
  public PIDController getRightPIDController() {
    return kRightPidController;
  }

  /**
   * Warp Drive - smooth Pathweaver trajectory generator for autonomous.
   */
  public static class WarpDrive {

    private static HashMap<String, Trajectory> cachedTrajectories = new HashMap<>();

    /**
     * Returns a runnable RamseteCommand that uses the provided trajectory.
     * @param driveTrain - The DriveTrainSubsystem to use for driving the trajectory.
     * @param trajectory - The trajectory object of the path to drive.
     */
    public static Command getRamseteCommand(DriveTrainSubsystem driveTrain, Trajectory trajectory) {
  
      SmartDashboard.putBoolean("reachedInnerRamsete", true);
      SmartDashboard.putNumber("preAutonHeading", driveTrain.getHeading());
  
      // driveTrain.resetOdometry(driveTrain.getPose());
  
      // var transform = driveTrain.getPose().minus(trajectory.getInitialPose());
      // trajectory = trajectory.transformBy(transform);
  
      driveTrain.resetOdometry(trajectory.getInitialPose());
  
      SmartDashboard.putNumber("totalRamseteSecs", trajectory.getTotalTimeSeconds());
  
      RamseteCommand ramsete = new RamseteCommand(
  
        // Generate an optimal trajectory from positions and translations
        trajectory,
  
        // Robot pose supplier
        driveTrain::getPose,
  
        // Ramsete controller
        Constants.RAMSETE_CONTROLLER,
        // Constants.DISABLED_RAMSETE_CONTROLLER,
  
        // Forward motor feed
        driveTrain.getForwardFeed(),
        
        // Drive kinematics
        driveTrain.getKinematics(),
  
        // Wheel speed supplier
        driveTrain::getWheelSpeeds,
  
        // PID controllers
        driveTrain.getLeftPIDController(),
        driveTrain.getRightPIDController(),
  
        // RamseteCommand passes volts to the callback through this supplier
        (leftVolts, rightVolts) -> {
          SmartDashboard.putBoolean("voltageSet", true);
          driveTrain.setTankVoltage(leftVolts, rightVolts);
          driveTrain.logForwardFeedValues();
        },
  
        // The drive subsystem itself
        driveTrain
      );
  
  
      return ramsete.andThen(() -> {
          driveTrain.setTankVoltage(0.0, 0.0);
          SmartDashboard.putBoolean("ranPath", true);
      });
  
    }

    /**
     * Returns a runnable RamseteCommand that uses a Pathweaver trajectory at the provided path.
     * @param driveTrain - The DriveTrainSubsystem to use for driving the trajectory.
     * @param path - The name of the Pathweaver path saved in the filepath deploy/[path]
     */
    public static Command getRamseteCommand(DriveTrainSubsystem driveTrain, String path) {

      Trajectory trajectory = generateTrajectory(path);

      if (trajectory == null) {
          return new InstantCommand();
      }
  
      return getRamseteCommand(driveTrain, trajectory);
  
    }

    /**
     * Generates a trajectory from a Pathweaver path at a specified filepath.
     * @param path - The filepath of the Pathweaver path.
     * @return Trajectory object
     */
    private static Trajectory generateTrajectory(String path) {
      Trajectory trajectory;
      try {
          trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path));
      } catch (IOException e) {
          System.out.println("///////////// Error occurred in WarpDrive.generateTrajectory(): " + e.getMessage());
          return null;
      }

      return trajectory;
    }

    /**
     * Returns a runnable RamseteCommand that uses a Pathweaver trajectory with the provided name.
     * @param filename - The name of the Pathweaver path saved in the filepath deploy/paths/[filename].wpilib.json
     */
    public static Command getRamseteCommand(String filename) {

      return getRamseteCommand(getCurrentDriveTrain(), "paths/" + filename + ".wpilib.json");

    }

    /**
     * Generates the Pathweaver trajectory with the specified name, and caches it for later (immediate) use.
     * Generating the trajectory takes a few seconds, so this allows you to pre-generate the trajectories before running commands.
     * @param filename - The name of the Pathweaver path saved in the filepath deploy/paths/[filename].wpilib.json
     */
    public static void addCachedPath(String filename) {
      Trajectory trajectory = generateTrajectory("paths/" + filename + ".wpilib.json");
      cachedTrajectories.put(filename, trajectory);
    }

    /**
     * Gets a cached, pre-generated Pathweaver trajectory with the specified name that was generated with WarpDrive.addCachedPath().
     * The command returned by this method can execute much faster than that returned by WarpDrive.getRamseteCommand().
     * @param filename - The name of the Pathweaver path saved in the filepath deploy/paths/[filename].wpilib.json
     * @return A RamseteCommand containing the specified cached Pathweaver trajectory.
     */
    public static Command getCachedPath(String filename) {

      Trajectory trajectory = cachedTrajectories.get(filename);

      if (trajectory == null) {
        System.out.println("/////////////// Error occurred in WarpDrive.getCachedPath(): requested trajectory does not exist in the cache.");
        return new InstantCommand();
      }
      else {
        return getRamseteCommand(getCurrentDriveTrain(), trajectory);
      }

    }

    /**
     * Generates a Ramsete Trajectory without using Pathweaver.
     * Requires manually defined waypoints, rotations, and translations.
     * This is <b><i>not working</i></b> as of the latest test.
     * 
     * @param startPosition - The position in which the robot starts its path.
     * @param endPosition - The position in which the robot ends its path.
     * @param translations - All translations and rotations forming the waypoints between start and end of the path.
     * @return A RamseteCommand using a manually generated trajectory.
     */
    public static Command drawManualPath(Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {
      // Generate an optimal trajectory from positions and translations
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        
        // Start pose
        startPosition,

        // Pass through these interior waypoints, making a curve path
        List.of(translations),
        
        // End pose
        endPosition,

        // Pass trajectory config
        Constants.TRAJECTORY_CONFIG
      );

      DriveTrainSubsystem driveTrain = getCurrentDriveTrain();
      driveTrain.resetOdometry(startPosition);

      Command ramsete = new RamseteCommand(

        // Generate an optimal trajectory from positions and translations
        trajectory,

        // Robot pose supplier
        driveTrain::getPose,

        // Ramsete controller
        Constants.RAMSETE_CONTROLLER,

        // Forward motor feed
        driveTrain.getForwardFeed(),
        
        // Drive kinematics
        driveTrain.getKinematics(),

        // Wheel speed supplier
        driveTrain::getWheelSpeeds,

        // PID controllers
        driveTrain.getLeftPIDController(),
        driveTrain.getRightPIDController(),

        // RamseteCommand passes volts to the callback through this supplier
        driveTrain::setTankVoltage,

        // The drive subsystem itself
        driveTrain
      );

      return ramsete.andThen(() -> {
        driveTrain.setTankVoltage(0.0, 0.0);
        SmartDashboard.putBoolean("ranPath", true);
      });      

    }

  }

}
