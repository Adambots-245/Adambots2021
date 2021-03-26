/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngleFromCameraCommand extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  DriveTrainSubsystem driveTrain;
  double speed;
  private GyroPIDSubsystem gyroPIDSubsystem;
  private Gyro gyro;
  private double targetAngle;
  private boolean resetGyro = true;
  private double inputSpeed;

  public TurnToAngleFromCameraCommand(DriveTrainSubsystem inpuDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = inpuDriveTrain;
    // speed = inputSpeed;
    speed = 0;
    this.inputSpeed = 0.3;

    // gyroPIDSubsystem = new GyroPIDSubsystem();
    gyro = Gyro.getInstance();
    addRequirements(driveTrain);

  }

  // TurnToAngleFromCameraCommand(DriveTrainSubsystem inpuDriveTrain, double inputSpeed,
  //     double targetAngle) {
  //   this(inpuDriveTrain, inputSpeed, targetAngle, true);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (resetGyro) {
      gyro.reset();
    }
    driveTrain.resetEncoders();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double verticalDegreesToCenter = table.getEntry("ty").getDouble(0);
    double calculatedDistance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(Math.abs(verticalDegreesToCenter) * (Math.PI / 180.0) + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL * (Math.PI / 180.0));
    double distanceFromIntakeArm = calculatedDistance - Constants.LIMELIGHT_DISTANCE_TO_INTAKE_ARM;
    this.targetAngle = table.getEntry("tx").getDouble(0) + 3 * (1);
    SmartDashboard.putNumber("Target Angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = inputSpeed * (targetAngle/Math.abs(targetAngle)); //gyroPIDSubsystem.getController().calculate(gyroPIDSubsystem.getMeasurement(), targetAngle);
    System.out.println("executing turn to angle from camera");
    System.out.println("yaw:" + gyro.getYaw());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    // SmartDashboard.putNumber("yaw",gyroPIDSubsystem.getGyroSubsystem().getYaw());
    // SmartDashboard.putNumber("gyroPIDSubsystem.getMeasurement()", gyroPIDSubsystem.getMeasurement());

    SmartDashboard.putNumber("turnSpeed", turnSpeed);
    SmartDashboard.putNumber("leftSpeed", driveTrain.getLeftDriveEncoderVelocity());
    SmartDashboard.putNumber("rightSpeed", driveTrain.getRightDriveEncoderVelocity());

    driveTrain.arcadeDrive(speed, turnSpeed);
    // driveTrain.driveDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("turn to angle interrupted");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;

    // return gyroPIDSubsystem.getController().atSetpoint();
    if (targetAngle < 0)
      return gyro.getYaw() <= targetAngle;
    else
      return gyro.getYaw() >= targetAngle;
  }
}
