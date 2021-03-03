// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonCommands.DriveForwardDistanceCommand;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.commands.autonCommands.TurnToAngleCommand;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BarrelPathAuton extends SequentialCommandGroup {
  /** Creates a new BarrelPathAuton. */
  public BarrelPathAuton(DriveTrainSubsystem driveTrainSubsystem) {
    
    super(

        // new ParallelCommandGroup( // deadline because it should move on after it has reached the position
          
        // DRIVE TO D5
        driveForward(driveTrainSubsystem, 125, true),
            
        // LOOP AROUND D5
        loopRight(driveTrainSubsystem),
        
        // DRIVE TO B8
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 135, true),
        
        // LOOP AROUND B8
        loopLeft(driveTrainSubsystem),

        // DRIVE to D10
        turn(driveTrainSubsystem, -45),
        driveForward(driveTrainSubsystem, 40, true),
        turn(driveTrainSubsystem, -45),
        driveForward(driveTrainSubsystem, 40, true),

        // HALF-LOOP AROUND D10
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, true),
        turn(driveTrainSubsystem, -90),

        // RETURN TO START/FINISH ZONE
        driveForward(driveTrainSubsystem, 180, true)

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
  public static DriveForwardDistanceCommand driveForward(DriveTrainSubsystem driveTrainSubsystem, double distance, boolean resetGyro) {
    // return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.75, 0, resetGyro);
    return new DriveForwardDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.75);
  }

  // public static DriveForwardGyroDistanceCommand driveBackward(DriveTrainSubsystem driveTrainSubsystem, double distance) {
  //   return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, 0.75, 0, false);
  // }

  public static TurnToAngleCommand turn(DriveTrainSubsystem driveTrainSubsystem, double angle) {
    return new TurnToAngleCommand(driveTrainSubsystem, 0.35, angle, true);
  }

  public static SequentialCommandGroup loopRight(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 50, true),
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 50, true),
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 45, true)
    );
  }

  public static SequentialCommandGroup loopLeft(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, true),
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, true),
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, true)
    );
  }

}