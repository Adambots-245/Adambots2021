/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Gamepad.Buttons;
import frc.robot.Gamepad.GamepadConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonCommands.PathFollower;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.PathRecorder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Very Lightweight Robot class that focuses on the drivetrain. Change the
 * Robot.java's Main to use the other Robot.java.
 */
public class RobotRecorder extends TimedRobot {
  private Command m_autonomousCommand;
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor,
      RobotMap.GearShifter, RobotMap.FrontRightMotor, RobotMap.FrontLeftMotor, RobotMap.BackLeftMotor,
      RobotMap.BackRightMotor);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.ArmMover, RobotMap.IntakeMotor,
      RobotMap.FeedToBlasterMotor);
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem(RobotMap.ConveyorMotor,
      RobotMap.AlignmentBeltMotor, RobotMap.IntakePhotoEye, RobotMap.SpacingPhotoEye, RobotMap.ExitPhotoEye);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

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
      // Vision System calculates the angle to the target and posts it to the
      // NetworkTable
      // vision = new VisionProcessorSubsystem(RobotMap.RingLight, new
      // GripPipeline());
      // visionThread = vision.getVisionThread();
      // visionThread.setDaemon(true);
      // visionThread.start();

      driveTrainSubsystem.setDefaultCommand(new DriveCommand(driveTrainSubsystem,
          () -> deaden(Buttons.primaryJoystick.getY(Hand.kLeft)), () -> Buttons.primaryJoystick.getX(Hand.kRight)));
    }

    SmartDashboard.putBoolean("Recording", false);
    SmartDashboard.putNumber("Added Recordings", 0);

    Buttons.primaryStartButton.whenPressed(new InstantCommand(
        // ()-> PathRecorder.getInstance().createRecording("barrel-roll-test")));
        () -> PathRecorder.getInstance().createRecording())
            .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Recording", true))));

    Buttons.primaryBackButton.whenPressed(new InstantCommand(() -> PathRecorder.getInstance().stopRecording())
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Recording", false))));

    Buttons.primaryXButton
        .whenPressed(new InstantCommand(() -> PathRecorder.getInstance().addRecording()).andThen(new InstantCommand(
            () -> SmartDashboard.putNumber("Added Recordings", SmartDashboard.getNumber("Added Recordings", 0) + 1))));

    Buttons.primaryYButton.whenPressed(new InstantCommand(() -> intakeSubsystem.intake(0))
        .andThen(new InstantCommand(() -> intakeSubsystem.RaiseIntake())
            .andThen(new InstantCommand(() -> conveyorSubsystem.stopConveyorMotor()))));

    Buttons.primaryBButton.whenPressed(new InstantCommand(() -> intakeSubsystem.LowerIntake())
        .andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> intakeSubsystem.intake(-1)))
        .andThen(new InstantCommand(() -> conveyorSubsystem.runConveyor(-1, false))));

    setupAutonRoutines();

    intakeSubsystem.RaiseIntake();
    // driveTrainSubsystem.shiftHighGear();

    // Buttons.primaryAButton.whenPressed(new
    // ShiftLowGearCommand(driveTrainSubsystem));
    // Buttons.primaryYButton.whenPressed(new
    // ShiftHighGearCommand(driveTrainSubsystem));
    // Buttons.primaryLB.whenPressed(new SetLowSpeedCommand(driveTrainSubsystem));
    // Buttons.primaryRB.whenPressed(new
    // SetNormalSpeedCommand(driveTrainSubsystem));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();

  }

  private void setupAutonRoutines() {
    // SmartDashboard.delete("Auton Mode");
    // autoChooser = new SendableChooser<>();

    for (String file : PathFollower.getListofRecordings()) {
      String name = file.substring(file.lastIndexOf("/") + 1);
      autoChooser.addOption(name, new PathFollower(file, driveTrainSubsystem));
    }

    SmartDashboard.putData("Auton Mode", autoChooser);
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
    conveyorSubsystem.checkPhotoEye();
    determinePath();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    PathRecorder.getInstance().stopRecording();

    // String file = PathFollower.getLastRecordedFile();
    // if (file != null) {
    // String name = file.substring(file.lastIndexOf("/") + 1);
    // autoChooser.addOption(name, new PathFollower(file, driveTrainSubsystem));

    // autoChooser.addOption("Last One", new PathFollower(file,
    // driveTrainSubsystem));
    // }

    // setupAutonRoutines();

    conveyorSubsystem.stopConveyorMotor();
    intakeSubsystem.intake(0);
    intakeSubsystem.RaiseIntake();

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
    // Gyro.getInstance().calibrationCheck(); // may take up to two seconds to
    // complete
    // System.out.println("Gyro Yaw at Startup: " + Gyro.getInstance().getYaw());
    // // CommandScheduler.getInstance().cancelAll(); // cancel all teleop commands

    // m_autonomousCommand = new
    // PathFollower("/home/lvuser/barrel-roll-test-1616853151.txt",
    // driveTrainSubsystem);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
    intakeSubsystem.LowerIntake();
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    intakeSubsystem.intake(-1);
    conveyorSubsystem.runConveyor(-1, false);
    
    
    // m_autonomousCommand = autoChooser.getSelected();
    // m_autonomousCommand = new PathFollower("/home/lvuser/barrel-roll-path1-1616860875.txt", driveTrainSubsystem).
    //   andThen(new PathFollower("/home/lvuser/barrel-roll-path2-1616861903.txt", driveTrainSubsystem)).
    //   andThen(new PathFollower("/home/lvuser/barrel-roll-path3-1616862754.txt", driveTrainSubsystem));
    // m_autonomousCommand = PathFollower.fromSegmentedPath("bounce-path", driveTrainSubsystem);

      m_autonomousCommand = PathFollower.fromSegmentedPath(determinePath(), driveTrainSubsystem);
      // m_autonomousCommand = new PathFollower("/home/lvuser/slalom-path1-1616870027.txt", driveTrainSubsystem).
      // andThen(new PathFollower("/home/lvuser/slalom-path2-1616875674.txt", driveTrainSubsystem)).
      // andThen(new PathFollower("/home/lvuser/slalom-path3-1616876271.txt", driveTrainSubsystem));

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

    // intakeSubsystem.LowerIntake();
    // intakeSubsystem.intake(-1);
    // conveyorSubsystem.runConveyor(-1, true);

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

  public String determinePath() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double x = table.getEntry("tx").getDouble(0);
    double y = table.getEntry("ty").getDouble(0);

    // Readability, I guess?
    // check x
    double lowerBound = x - Constants.ERROR_DISTANCE;
    double upperBound = x + Constants.ERROR_DISTANCE;
    boolean xnearRedA = (lowerBound < Constants.GSEARCH_RED_AX) && (upperBound > Constants.GSEARCH_RED_AX);
    boolean xnearRedB = (lowerBound < Constants.GSEARCH_RED_BX) && (upperBound > Constants.GSEARCH_RED_BX);
    boolean xnearBlueA = (lowerBound < Constants.GSEARCH_BLUE_AX) && (upperBound > Constants.GSEARCH_BLUE_AX);
    boolean xnearBlueB = (lowerBound < Constants.GSEARCH_BLUE_BX) && (upperBound > Constants.GSEARCH_BLUE_BX);

    // check y
    lowerBound = y - Constants.ERROR_DISTANCE;
    upperBound = y + Constants.ERROR_DISTANCE;
    boolean ynearRedA = (lowerBound < Constants.GSEARCH_RED_AY) && (upperBound > Constants.GSEARCH_RED_AY);
    boolean ynearRedB = (lowerBound < Constants.GSEARCH_RED_BY) && (upperBound > Constants.GSEARCH_RED_BY);
    boolean ynearBlueA = (lowerBound < Constants.GSEARCH_BLUE_AY) && (upperBound > Constants.GSEARCH_BLUE_AY);
    boolean ynearBlueB = (lowerBound < Constants.GSEARCH_BLUE_BY) && (upperBound > Constants.GSEARCH_BLUE_BY);

    // Where the paths are actually decided
    if(xnearRedA && ynearRedA) {
      //return ""; // path to Red A recording
      SmartDashboard.putString("determinedPath", "RED A");
      return Constants.RED_A_PATH;
    } else if(xnearRedB & ynearRedB) {
      //return ""; // path to Red B recording
      SmartDashboard.putString("determinedPath", "RED B");
      return Constants.RED_B_PATH;
    } else if(xnearBlueA && ynearBlueA) {
      //return ""; // path to Blue A recording
      SmartDashboard.putString("determinedPath", "BLUE A");
      return Constants.BLUE_A_PATH;
    } else if(xnearBlueB && ynearBlueB) {
      //return ""; // path to Blue B recording
      SmartDashboard.putString("determinedPath", "BLUE B");
      return Constants.BLUE_B_PATH;
    } else {
      //return "oh no";
      SmartDashboard.putString("determinedPath", "No valid path detected");
      return null;
    }
  }
}
