// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryCommand extends SequentialCommandGroup {

  /** Creates a new TrajectoryCommand. */
  public TrajectoryCommand(DriveTrainSubsystem driveTrain, Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(PairedTrajectoryCommand(

      driveTrain,
      startPosition,
      endPosition,
      translations
      
    ).getFirst().andThen(() -> driveTrain.setVoltage(0, 0)));

  }

  /** Creates a new TrajectoryCommand. */
  private TrajectoryCommand(DriveTrainSubsystem driveTrain, Trajectory trajectory, Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(new RamseteCommand(

      // Generate an optimal trajectory from positions and translations
      trajectory,

      // Robot pose supplier
      driveTrain::getPose,

      // Ramsete controller
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),

      // Forward motor feed
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      
      // Drive kinematics
      Constants.kDriveKinematics,

      // Wheel speed supplier
      driveTrain::getWheelSpeeds,

      // PID controllers
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),

      // RamseteCommand passes volts to the callback through this supplier
      driveTrain::setVoltage,

      // The drive subsystem itself
      driveTrain
    ));

    driveTrain.resetOdometry(startPosition);

  }

  public static TrajectoryCommand RawTrajectoryCommand(DriveTrainSubsystem driveTrain, Trajectory trajectory, Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {
    return new TrajectoryCommand(driveTrain, trajectory, startPosition, endPosition, translations);
  }

  public static Pair<TrajectoryCommand, Trajectory> PairedTrajectoryCommand(DriveTrainSubsystem driveTrain, Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {

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

    return new Pair<TrajectoryCommand, Trajectory>(
      RawTrajectoryCommand(driveTrain, trajectory, startPosition, endPosition, translations),
      trajectory
    );

  }

}
