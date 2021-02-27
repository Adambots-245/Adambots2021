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
        driveForward(driveTrainSubsystem, 145, false),
        
        // LOOP AROUND B8
        loopLeft(driveTrainSubsystem),

        // DRIVE to D10
        turn(driveTrainSubsystem, -45),
        driveForward(driveTrainSubsystem, 40, false),
        turn(driveTrainSubsystem, -45),
        driveForward(driveTrainSubsystem, 40, false),

        // HALF-LOOP AROUND D10
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, false),
        turn(driveTrainSubsystem, -90),

        // RETURN TO START/FINISH ZONE
        driveForward(driveTrainSubsystem, 180, false)

        //

              //  )

    );
  }

  public static DriveForwardGyroDistanceCommand driveForward(DriveTrainSubsystem driveTrainSubsystem, double distance, boolean initial) {
    return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.75, 0, initial);
  }

  public static DriveForwardGyroDistanceCommand driveBackward(DriveTrainSubsystem driveTrainSubsystem, double distance) {
    return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, 0.75, 0, false);
  }

  public static TurnToAngleCommand turn(DriveTrainSubsystem driveTrainSubsystem, double angle) {
    return new TurnToAngleCommand(driveTrainSubsystem, 0.35, angle, false);
  }

  public static SequentialCommandGroup loopRight(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 45, false),
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 45, false),
        turn(driveTrainSubsystem, 90),
        driveForward(driveTrainSubsystem, 40, false)
    );
  }

  public static SequentialCommandGroup loopLeft(DriveTrainSubsystem driveTrainSubsystem) {
    return new SequentialCommandGroup(
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, false),
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, false),
        turn(driveTrainSubsystem, -90),
        driveForward(driveTrainSubsystem, 45, false)
    );
  }

}
