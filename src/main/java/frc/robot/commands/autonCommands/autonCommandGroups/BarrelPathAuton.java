// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.autonCommands.DriveStraightCommand;
import frc.robot.commands.autonCommands.TurnCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BarrelPathAuton extends SequentialCommandGroup {
  /** Creates a new BarrelPathAuton. */
  public BarrelPathAuton(DriveTrainSubsystem driveTrainSubsystem) {
    
    super(

        // new ParallelCommandGroup( // deadline because it should move on after it has reached the position
        
        //new DriveStraightCommand(driveTrainSubsystem, -0.75, Constants.ENCODER_TICKS_PER_INCH * 120),
        // new TurnCommand(45, driveTrainSubsystem)
        
        // DRIVE TO D5
        // driveForward(driveTrainSubsystem, 25, true)
        new TurnCommand(90, driveTrainSubsystem)
            
        // LOOP AROUND D5
        // ,loopRight(driveTrainSubsystem)
        // new WaitCommand(1),
        // DRIVE TO B8
        // turn(driveTrainSubsystem, 90),
        // new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 160, -0.70, 0, true),
        // driveForward(driveTrainSubsystem, 165, true),
        
        // // // LOOP AROUND B8
        // loopLeft(driveTrainSubsystem),

        // // // DRIVE to D10
        // turn(driveTrainSubsystem, -45),
        // driveForward(driveTrainSubsystem, 65, true),
        // turn(driveTrainSubsystem, -45),
        // driveForward(driveTrainSubsystem, 55, true),

        // // // HALF-LOOP AROUND D10
        // turn(driveTrainSubsystem, -90),
        // driveForward(driveTrainSubsystem, 45, true),
        // turn(driveTrainSubsystem, -90),

        // // // RETURN TO START/FINISH ZONE
        // driveForward(driveTrainSubsystem, 255, true)
        //

              //  )

    );
  }


  /**
   * Drive forward command with Gyro.
   * @param driveTrainSubsystem - The DriveTrain Subsystem instance.
   * @param distance - The distance (in inches) to drive forward.
   * @param resetGyro - Whether or not to reset the Gyro.
   * @return DriveForwardGyroDistanceCommand
   */
  public static Command driveForward(DriveTrainSubsystem driveTrainSubsystem, double distance, boolean resetGyro) {
    // return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.70, 0, resetGyro);
    // return new DriveForwardDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.70);
    return new DriveStraightCommand(driveTrainSubsystem, -0.70, Constants.ENCODER_TICKS_PER_INCH * distance);
  }

  // public static DriveForwardGyroDistanceCommand driveBackward(DriveTrainSubsystem driveTrainSubsystem, double distance) {
  //   return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, 0.75, 0, false);
  // }

  public static Command turn(DriveTrainSubsystem driveTrainSubsystem, double angle) {
    // return new ParallelDeadlineGroup(new WaitCommand(3), new TurnToAngleCommand(driveTrainSubsystem, 0.30, angle, true));
    // return new TurnToAngleCommand(driveTrainSubsystem, 0.30, angle, true);
    return new TurnCommand(angle, driveTrainSubsystem);
  }

  public static SequentialCommandGroup loopRight(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, 90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 90, true),
        // shortPause(),
        turn(driveTrainSubsystem, 90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 90, true),
        // shortPause(),
        turn(driveTrainSubsystem, 90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 90, true)
    );
  }

  public static SequentialCommandGroup loopLeft(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, -90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 45, true),
        // shortPause(),
        turn(driveTrainSubsystem, -90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 70, true),
        // shortPause(),
        turn(driveTrainSubsystem, -90),
        // shortPause(),
        driveForward(driveTrainSubsystem, 50, true)
    );
  }

  public static WaitCommand shortPause() {
    return null; //new WaitCommand(0.25);
  }

}
