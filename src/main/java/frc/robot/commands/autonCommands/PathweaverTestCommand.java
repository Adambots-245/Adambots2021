// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PathweaverTestCommand extends RamseteCommand {
  /** Creates a new PathweaverTestCommand. */
  public PathweaverTestCommand(DriveTrainSubsystem driveTrain) throws IOException {
    // Use addRequirements() here to declare subsystem dependencies.

    super(

      // Generate an optimal trajectory from positions and translations
      TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/TestPath.wpilib.json")),

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
}
