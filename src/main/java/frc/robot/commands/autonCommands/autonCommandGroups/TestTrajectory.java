// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.









// NOT CURRENTLY WORKING \\








package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectory extends SequentialCommandGroup {

  /** Creates a new TestTrajectory. */
  public TestTrajectory() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(

        DriveTrainSubsystem.WarpDrive.drawManualPath(
          //Initial pose
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),

          //End pose
          // End 2 meters straight ahead of where we started, facing forward
          new Pose2d(2, 0, new Rotation2d(0)),

          //Translations (midpoints to reach)
          new Translation2d(1, 1),
          new Translation2d(2, -1)
        )

    );
  }

}
