// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryCommand extends RamseteCommand {

  // Configuration for the trajectory. Can be statically modified if necessary
  private static TrajectoryConfig config = TrajectoryCommand.getDefaultConfig();

  /** Creates a new TrajectoryCommand. */
  public TrajectoryCommand(DriveTrainSubsystem driveTrain, Pose2d startPosition, Pose2d endPosition, Translation2d...translations) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(

      // Generate an optimal trajectory from positions and translations
      TrajectoryGenerator.generateTrajectory(
        
        // Start pose
        startPosition,

        // Pass through these interior waypoints, making a curve path
        List.of(translations),
        
        // End pose
        endPosition,

        // Pass trajectory config
        config
      ),

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
    );

    driveTrain.resetOdometry(startPosition);
    // this.andThen(() -> driveTrain.setVoltage(0, 0));

  }


  public static RamseteTrajectoryBuilder setConfig(TrajectoryConfig conf) {
    config = conf;
    return new RamseteTrajectoryBuilder();
  }

  public static class RamseteTrajectoryBuilder {
    public TrajectoryCommand buildTrajectory(DriveTrainSubsystem driveTrain, Pose2d startPose, Pose2d endPose, Translation2d...translations) {
      return new TrajectoryCommand(driveTrain, startPose, endPose, translations);
    }
  }

  public static TrajectoryConfig getDefaultConfig() {
    return new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
    
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.kDriveKinematics);

    // Apply the voltage constraint
    // .addConstraint(
    //   new DifferentialDriveVoltageConstraint(

    //     // Forward motor feed
    //     new SimpleMotorFeedforward(Constants.ksVolts,
    //                               Constants.kvVoltSecondsPerMeter,
    //                               Constants.kaVoltSecondsSquaredPerMeter),

    //     // Drive kinematics
    //     Constants.kDriveKinematics,

    //     // Maximum voltage (originally a constant 10)
    //     Constants.MAX_DRIVE_VOLTAGE
      
    //   )
    // );
  }

}
