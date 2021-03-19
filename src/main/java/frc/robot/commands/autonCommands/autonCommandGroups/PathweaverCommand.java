// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PathweaverCommand extends SequentialCommandGroup {
  /** Creates a new PathweaverTestCommand. */
  public PathweaverCommand(DriveTrainSubsystem driveTrain, String path) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(getRamseteCommand(driveTrain, path));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Command getRamseteCommand(DriveTrainSubsystem driveTrain, String path) {

    Trajectory trajectory;
    try {
        trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/" + path + ".wpilib.json"));
    } catch (IOException e) {
        System.out.println("///////////// Error occurred in PathWeaverCommand: " + e.getMessage());
        return new InstantCommand();
    }

    SmartDashboard.putBoolean("reachedInnerRamsete", true);

    driveTrain.resetOdometry(driveTrain.getPose());

    // var transform = driveTrain.getPose().minus(trajectory.getInitialPose());
    // trajectory = trajectory.transformBy(transform);

    RamseteCommand ramsete = new RamseteCommand(

      // Generate an optimal trajectory from positions and translations
      trajectory,

      // Robot pose supplier
      () -> driveTrain.getPose(),

      // Ramsete controller
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),

      // Forward motor feed
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      
      // Drive kinematics
      Constants.kDriveKinematics,

      // Wheel speed supplier
      () -> driveTrain.getWheelSpeeds(),

      // PID controllers
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),

      // RamseteCommand passes volts to the callback through this supplier
      (leftVolts, rightVolts) -> {

        SmartDashboard.putBoolean("voltageSet", true);

          driveTrain.setVoltage(leftVolts, rightVolts);
      },

      // The drive subsystem itself
      driveTrain
    );

    return ramsete;
    // return ramsete.andThen(() -> {
    //     driveTrain.setVoltage(0.0, -0.0);
    //     SmartDashboard.putBoolean("ranPath", true);
    // });

  }

}
