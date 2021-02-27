// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
          new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 125, -0.75, 0, true),
            new TurnToAngleCommand(driveTrainSubsystem, 0.35, 90, false),
              new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 45, -0.75, 0, false),
                 new TurnToAngleCommand(driveTrainSubsystem, 0.35, 90, false),
                  new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 45, -0.75, 0, false),
                     new TurnToAngleCommand(driveTrainSubsystem, 0.35, 90, false),
                     new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 45, -0.75, 0, false),
                     new TurnToAngleCommand(driveTrainSubsystem, 0.35, 90, false),
                       new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 145, -0.75, 0, false)
                  
              //  )

    );
  }
}
