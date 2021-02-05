
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Gamepad.DPad_JoystickButton;
import frc.robot.Gamepad.GamepadConstants;

import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.commands.autonCommands.autonCommandGroups.CrossBaseline;
import frc.robot.commands.autonCommands.autonCommandGroups.NerdsAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.NoTurnAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.PushNom2Yeet5;
import frc.robot.commands.autonCommands.autonCommandGroups.PushNom2Yeet5Nom1;
import frc.robot.commands.autonCommands.autonCommandGroups.SnagNYeetCommandGroup;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3FinalsAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3Nom3;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3PushNom3;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  // initialize controllers
  private final XboxController primaryJoystick = new XboxController(GamepadConstants.PRIMARY_DRIVER);
  private final XboxController secondaryJoystick = new XboxController(GamepadConstants.SECONDARY_DRIVER);
  

  // subsystems
  private GyroSubsystem gyroSubsystem = GyroSubsystem.getInstance();
  private final BlasterSubsystem blasterSubsystem = new BlasterSubsystem();
  private final ControlPanelSubsystem panelSubsystem = new ControlPanelSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(gyroSubsystem);
  private final GondolaSubsystem gondolaSubsystem = new GondolaSubsystem();
  private final HangSubsystem hangSubsystem = new HangSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private LidarSubsystem lidarSubsystem = null;
  private TurretSubsystem turretSubsystem = null;
  
  // commands
  private BackboardToggleCommand backboardToggleCommand;
  private ConveyorCommand conveyorCommand;
  private DriveForwardDistanceCommand autonDriveForwardDistanceCommand;
  private TurnToAngleCommand autonTurn90DegreeCommand;
  private GondolaCommand gondolaCommand;
  private GyroDriveForDistCommand autonGyroDriveForwardDistanceCommand;
  private RaiseElevatorCommand raiseElevatorCommand;
  private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;
  private WinchCommand winchCommand;
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (Robot.isReal()) {

      // For tuning only
      SmartDashboard.putNumber("kP", Constants.GYRO_kP);
      SmartDashboard.putNumber("kI", Constants.GYRO_kI);
      SmartDashboard.putNumber("kD", Constants.GYRO_kD);
      
      lidarSubsystem = new LidarSubsystem();
      // gyroSubsystem = new GyroSubsystem();
      turretSubsystem = new TurretSubsystem();
    }
    
    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.resetEncoders();

    // configure the dashboard
    dash();


    //auton commands
    autonDriveForwardDistanceCommand = new DriveForwardDistanceCommand(driveTrainSubsystem,
        Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED);

    // autonGyroDriveForwardDistanceCommand = new GyroDriveForDistCommand(driveTrainSubsystem,
        // Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED, gyroSubsystem.getYaw());
        double autonSpeed = .75;
    autonDriveForwardGyroDistanceCommand = new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_PUSH_ROBOT_DISTANCE, autonSpeed*.5, 0, true).andThen(new WaitCommand(1)).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_FORWARD_BALL_PICKUP_DISTANCE, -autonSpeed, 0, false));
    
    autonTurn90DegreeCommand = new TurnToAngleCommand(driveTrainSubsystem, autonSpeed*0.5, 45, true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // deadzoning
  private double deaden(double rawInput) {
    return Math.abs(rawInput) < GamepadConstants.DEADZONE ? 0 : rawInput;
  }

  private void configureButtonBindings() {
    //primary buttons
    final JoystickButton primaryBackButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_BACK);
    final JoystickButton primaryStartButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_START);
    final JoystickButton primaryXButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_X);
    final JoystickButton primaryYButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_Y);
    final JoystickButton primaryBButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_B);
    final JoystickButton primaryAButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_A);
    final JoystickButton primaryLB = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_LB);
    final JoystickButton primaryRB = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_RB);
    final JoystickButton primaryLeftStickButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_LEFT_STICK);
    final JoystickButton primaryRightStickButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_RIGHT_STICK);
    
    //primary DPad
    final DPad_JoystickButton primaryDPadN = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_N_ANGLE);
    final DPad_JoystickButton primaryDPadNW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NW_ANGLE);
    final DPad_JoystickButton primaryDPadW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_W_ANGLE);
    final DPad_JoystickButton primaryDPadSW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SW_ANGLE);
    final DPad_JoystickButton primaryDPadS = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_S_ANGLE);
    final DPad_JoystickButton primaryDPadSE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SE_ANGLE);
    final DPad_JoystickButton primaryDPadE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_E_ANGLE);
    final DPad_JoystickButton primaryDPadNE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NE_ANGLE);
    
    //primary axes
    //RIGHT TRIGGER       primaryJoystick.getTriggerAxis(Hand.kRight)
    //LEFT TRIGGER        primaryJoystick.getTriggerAxis(Hand.kLeft)
    //LEFT STICK X AXIS   primaryJoystick.getX(Hand.kLeft)
    //LEFT STICK Y AXIS   primaryJoystick.getY(Hand.kLeft)
    //RIGHT STICK X AXIS  primaryJoystick.getX(Hand.kRight)
    //RIGHT STICK Y AXIS  primaryJoystick.getY(Hand.kRight)
    
    //secondary buttons
    final JoystickButton secondaryBackButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_BACK);
    final JoystickButton secondaryStartButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_START);
    final JoystickButton secondaryXButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_X);
    final JoystickButton secondaryYButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_Y);
    final JoystickButton secondaryBButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_B);
    final JoystickButton secondaryAButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_A);
    final JoystickButton secondaryLB = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_LB);
    final JoystickButton secondaryRB = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_RB);  
    final JoystickButton secondaryLeftStickButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_LEFT_STICK);
    final JoystickButton secondaryRightStickButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_RIGHT_STICK);
    
    //secondary DPad
    final DPad_JoystickButton secondaryDPadN = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_N_ANGLE);
    final DPad_JoystickButton secondaryDPadNW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NW_ANGLE);
    final DPad_JoystickButton secondaryDPadW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_W_ANGLE);
    final DPad_JoystickButton secondaryDPadSW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SW_ANGLE);
    final DPad_JoystickButton secondaryDPadS = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_S_ANGLE);
    final DPad_JoystickButton secondaryDPadSE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SE_ANGLE);
    final DPad_JoystickButton secondaryDPadE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_E_ANGLE);
    final DPad_JoystickButton secondaryDPadNE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NE_ANGLE);
    
    //secondary axes    
    //RIGHT TRIGGER       secondaryJoystick.getTriggerAxis(Hand.kRight)
    //LEFT TRIGGER        secondaryJoystick.getTriggerAxis(Hand.kLeft)
    //LEFT STICK X AXIS   secondaryJoystick.getX(Hand.kLeft)
    //LEFT STICK Y AXIS   secondaryJoystick.getY(Hand.kLeft)
    //RIGHT STICK X AXIS  secondaryJoystick.getX(Hand.kRight)
    //RIGHT STICK Y AXIS  secondaryJoystick.getY(Hand.kRight)


    // primary controls
      // drive subsystem
      driveTrainSubsystem.setDefaultCommand(new DriveCommand(driveTrainSubsystem, () -> deaden(primaryJoystick.getY(Hand.kLeft)),
      () -> primaryJoystick.getX(Hand.kRight)));    
      primaryAButton.whenPressed(new ShiftLowGearCommand(driveTrainSubsystem));
      primaryYButton.whenPressed(new ShiftHighGearCommand(driveTrainSubsystem));
      primaryLB.whenPressed(new SetLowSpeedCommand(driveTrainSubsystem));
      primaryRB.whenPressed(new SetNormalSpeedCommand(driveTrainSubsystem));

      //control panel
      primaryXButton.whenPressed(new RotatePanelCommand(panelSubsystem));
      primaryBButton.whenPressed(new AlignColorCommand(panelSubsystem));
      // secondaryXButton.whenHeld(new PanelMotor(panelSubsystem)); //CHANGE THIS TO PRIMARY SOMEHOW

    // secondary controls
      // intake 
      intakeSubsystem.setDefaultCommand(new StartIntakeCommand(intakeSubsystem, () -> deaden(secondaryJoystick.getY(Hand.kRight))));
      conveyorSubsystem.setDefaultCommand(new ConveyorCommand(conveyorSubsystem, ()-> deaden(secondaryJoystick.getY(Hand.kRight)/* *0.7 */)));
      secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
      secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      // secondaryYButton.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      secondaryBButton.whenHeld(new ReverseIndexToBlasterCommand(intakeSubsystem));
      secondaryRB.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      
      // turret 
      turretSubsystem.setDefaultCommand(new ManualTurretCommand(turretSubsystem, ()->Math.pow(secondaryJoystick.getTriggerAxis(Hand.kLeft), 2), ()->Math.pow(deaden(secondaryJoystick.getTriggerAxis(Hand.kRight)), 2)));
     
      secondaryXButton.whileHeld(new TurnToTargetCommand(turretSubsystem, lidarSubsystem), false);
      // turretSubsystem.setDefaultCommand(new TurretManualCommand(turretSubsystem, ()->secondaryJoystick.getTriggerAxis(Hand.kLeft), ()->secondaryJoystick.getTriggerAxis(Hand.kRight)));
      
      // lidar susbsystem
        // primaryXButton.whenPressed(new MeasureDistanceCommand(lidarSubsystem));
      
      // blaster  
      // secondaryLB.toggleWhenPressed(new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem));
      secondaryLB.toggleWhenPressed(new BlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem, secondaryJoystick));
      secondaryYButton.whenReleased(new BackboardToggleCommand(blasterSubsystem));
      // blasterSubsystem.setDefaultCommand(new BlasterPercentOutput(blasterSubsystem, () -> primaryJoystick.getTriggerAxis(Hand.kRight)));
    
      // hang 
      hangSubsystem.setDefaultCommand(new RaiseElevatorCommand(hangSubsystem, () -> deaden(secondaryJoystick.getY(Hand.kLeft)), secondaryStartButton));
      gondolaSubsystem.setDefaultCommand(new GondolaCommand(gondolaSubsystem, () -> deaden(secondaryJoystick.getX(Hand.kLeft))));
      secondaryAButton.whenHeld(new WinchCommand(hangSubsystem), false);
      //raiseElevatorCommand = new RaiseElevatorCommand(hangSubsystem, () -> secondaryJoystick.getY(Hand.kLeft));    
      //gondolaCommand = new GondolaCommand(hangSubsystem, ()->secondaryJoystick.getX(Hand.kLeft));
      
    // dashboard control buttons  
      SmartDashboard.putData("10 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY));
      // SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem, shooterVelocity));
      SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem, Constants.TRENCH_SHOOTER_VELOCITY));
      SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

    // mode switching 
      // startIntakeCommand.addRequirements(elevatorSubsystem, conveyorSubsystem, alignmentBeltSubsystem);
      //secondaryBackButton.whenPressed(startIntakeCommand);
      // secondaryStartButton.whenPressed(new StopIntakeOuttakeCommand(intakeSubsystem));

    // test stuff 
      // primaryYButton.whenPressed(new StartOuttakeCommand(intakeSubsystem));
      // primaryAButton.whenReleased(new StopIntakeOuttakeCommand(intakeSubsystem));
      // //primaryAButton.whileHeld(new TestCo  mmand());
      // primaryYButton.whenReleased(new StopIntakeOuttakeCommand(intakeSubsystem));

      // Turret subsystem
      //TurretManualCommand turretManualCommand = new TurretManualCommand(turretSubsystem,
      //    () -> secondaryJoystick.getTriggerAxis(Hand.kLeft), () -> secondaryJoystick.getTriggerAxis(Hand.kRight));
      //secondaryLB.whenHeld(new TurnToTargetCommand(turretSubsystem));

  }

  private void dash(){
    // autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Snag N' Yeet", new SnagNYeetCommandGroup(driveTrainSubsystem, intakeSubsystem, turretSubsystem, lidarSubsystem, blasterSubsystem, secondaryJoystick));
    // autoChooser.setDefaultOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, lidarSubsystem, conveyorSubsystem));
    autoChooser.addOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, lidarSubsystem, conveyorSubsystem, secondaryJoystick));
    // autoChooser.addOption("Yeet3Nom3", new Yeet3Nom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, lidarSubsystem, conveyorSubsystem));
    autoChooser.addOption("PushNom2Yeet5Nom1", new PushNom2Yeet5Nom1(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, lidarSubsystem, conveyorSubsystem, secondaryJoystick));
    autoChooser.setDefaultOption("yeet3", new Yeet3(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, lidarSubsystem, blasterSubsystem, secondaryJoystick));
    autoChooser.addOption("noturn", new NoTurnAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, lidarSubsystem, blasterSubsystem, secondaryJoystick));
    autoChooser.addOption("NERDSAUTO", new NerdsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, lidarSubsystem, blasterSubsystem));
    autoChooser.addOption("CrossBaseline", new CrossBaseline(driveTrainSubsystem));
    autoChooser.addOption("Yeet3FinalsAuton", new Yeet3FinalsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, lidarSubsystem, blasterSubsystem, secondaryJoystick));
    // autoChooser.addOption("90Degrees", autonTurn90DegreeCommand);
    // autoChooser.addOption("0 to 45 to 0", new );
    SmartDashboard.putData("Auton Mode", autoChooser);

    if (lidarSubsystem == null)
      lidarSubsystem = new LidarSubsystem();

    gyroSubsystem.reset();
    SmartDashboard.putNumber("Yaw", gyroSubsystem.getYaw());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // autonDriveForwardDistanceCommand will run in autonomous
    // return autonDriveForwardGyroDistanceCommand;
    // return new  DriveForwardGyroDistanceCommand(driveTrainSubsystem, 0, 0, 0, true).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 3500*48, -.75, 0, true)).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 3500*84, -.5, 90, false));
    // return autonTurn90DegreeCommand.andThen(new WaitCommand(3)).andThen(new TurnToAngleCommand(driveTrainSubsystem, 0.5, -45, false));
    System.out.println(autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
