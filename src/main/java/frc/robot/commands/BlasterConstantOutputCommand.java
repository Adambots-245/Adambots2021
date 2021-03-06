/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.LidarSubsystem;

public class BlasterConstantOutputCommand extends CommandBase {
  /**
   * Creates a new BlasterConstantOutputCommand.
   */
  BlasterSubsystem blasterSubsystem;
  private LidarSubsystem lidarSubsystem;
  private double velocityInEncoderTicks;

  public BlasterConstantOutputCommand(BlasterSubsystem blasterSubsystem, LidarSubsystem lidarSubsystem, double velocityInEncoderTicks) {
    this.blasterSubsystem = blasterSubsystem;
    this.lidarSubsystem = lidarSubsystem;
    this.velocityInEncoderTicks = velocityInEncoderTicks;
    
    SmartDashboard.putNumber("Blaster Velocity", blasterSubsystem.getVelocity());
    SmartDashboard.putNumber("Distance To Target", lidarSubsystem.getInches());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blasterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // blasterSubsystem.setVelocity(10343);
    blasterSubsystem.setVelocity(velocityInEncoderTicks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Blaster Velocity", blasterSubsystem.getVelocity());
    SmartDashboard.putNumber("Distance To Target", lidarSubsystem.getInches());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    blasterSubsystem.output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
