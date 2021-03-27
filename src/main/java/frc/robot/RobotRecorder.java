/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Gamepad.Buttons;
import frc.robot.Gamepad.GamepadConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SetLowSpeedCommand;
import frc.robot.commands.SetNormalSpeedCommand;
import frc.robot.commands.ShiftHighGearCommand;
import frc.robot.commands.ShiftLowGearCommand;
import frc.robot.commands.autonCommands.PathFollower;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.*;
import frc.robot.utils.PathRecorder;
import frc.robot.vision.GripPipeline;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Very Lightweight Robot class that focuses on the drivetrain.
 * Change the Robot.java's Main to use the other Robot.java.
 */
public class RobotRecorder extends TimedRobot {
  private Command m_autonomousCommand;
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor, RobotMap.GearShifter, RobotMap.FrontRightMotor, RobotMap.FrontLeftMotor, RobotMap.BackLeftMotor, RobotMap.BackRightMotor);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // private RobotContainer m_robotContainer;

  // private VisionProcessorSubsystem vision;
  // private Thread visionThread;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    if (Robot.isReal()) {
      // Starts vision thread only if not running in simulation mode
      // Vision System calculates the angle to the target and posts it to the NetworkTable
      // vision = new VisionProcessorSubsystem(RobotMap.RingLight, new GripPipeline());
      // visionThread = vision.getVisionThread();
      // visionThread.setDaemon(true);
      // visionThread.start();

      driveTrainSubsystem.setDefaultCommand(
        new DriveCommand(driveTrainSubsystem, 
        () -> deaden(Buttons.primaryJoystick.getY(Hand.kLeft)),
        () -> Buttons.primaryJoystick.getX(Hand.kRight))
        );  
    }

    SmartDashboard.putBoolean("Recording", false);

    Buttons.primaryStartButton.whenPressed(
      new InstantCommand(
        // ()-> PathRecorder.getInstance().createRecording("barrel-roll-test")));
        ()-> PathRecorder.getInstance().createRecording(PathRecorder.getFileNameFromSmartDashboard())).
          andThen(new InstantCommand(
            () -> SmartDashboard.putBoolean("Recording", true)
          ))
        );

    Buttons.primaryBackButton.whenPressed(
      new InstantCommand(
        ()-> PathRecorder.getInstance().stopRecording()).
          andThen(new InstantCommand(
            () -> SmartDashboard.putBoolean("Recording", false)
          ))
    );

    setupAutonRoutines();

    SmartDashboard.putData("Auton Mode", autoChooser);

    // Buttons.primaryAButton.whenPressed(new ShiftLowGearCommand(driveTrainSubsystem));
    // Buttons.primaryYButton.whenPressed(new ShiftHighGearCommand(driveTrainSubsystem));
    // Buttons.primaryLB.whenPressed(new SetLowSpeedCommand(driveTrainSubsystem));
    // Buttons.primaryRB.whenPressed(new SetNormalSpeedCommand(driveTrainSubsystem));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();

  }

  private void setupAutonRoutines(){
    for (String file: PathFollower.getListofRecordings()){
      String name = file.substring(file.lastIndexOf("/") + 1);
      autoChooser.addOption(name, new PathFollower(file, driveTrainSubsystem));
    }
  }

  private double deaden(double rawInput) {
    return Math.abs(rawInput) < GamepadConstants.DEADZONE ? 0 : rawInput;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // if (Robot.isReal()) {
      // SmartDashboard.putNumber("ANGLE", vision.getAngle());
    // }
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    PathRecorder.getInstance().stopRecording();
    
    String file = PathFollower.getLastRecordedFile();
    String name = file.substring(file.lastIndexOf("/") + 1);
    autoChooser.addOption(name, new PathFollower(file, driveTrainSubsystem));

    autoChooser.addOption("Last One", new PathFollower(file, driveTrainSubsystem));
    SmartDashboard.putData("Auton Mode", autoChooser);
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // SmartDashboard.putString("auton selected", m_autonomousCommand.toString());

    
    // System.out.println("Init Auton.........");
    // Gyro.getInstance().reset();
    // Gyro.getInstance().calibrationCheck(); // may take up to two seconds to complete
    // System.out.println("Gyro Yaw at Startup: " + Gyro.getInstance().getYaw());
    // // CommandScheduler.getInstance().cancelAll(); // cancel all teleop commands

    // m_autonomousCommand = new PathFollower("/home/lvuser/barrel-roll-test-1616853151.txt", driveTrainSubsystem);
    m_autonomousCommand = autoChooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("yaw",gyroSubsystem.getYaw());

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
    // System.out.println("Running test periodic");
    // SmartDashboard.putNumber("pitch",gyroSubsystem.getPitch());
    // SmartDashboard.putNumber("roll",gyroSubsystem.getRoll());
    // SmartDashboard.putNumber("yaw",gyroSubsystem.getYaw());
    // CommandScheduler.getInstance().run();

  }
}
