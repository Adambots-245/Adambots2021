/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.TurretSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Yeet3Nom3 extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3PushNom3.
   */
  public Yeet3Nom3(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, BlasterSubsystem blasterSubsystem, LidarSubsystem lidarSubsystem, ConveyorSubsystem conveyorSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      // YEET 3 BALLS (PHASE 1)
      // new BackboardNearCommand(blasterSubsystem),
      new TurnToTargetCommand(turretSubsystem, lidarSubsystem),
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem, joystick),
        new IndexToBlasterCommand(intakeSubsystem),
        new ConveyorCommand(conveyorSubsystem, ()->1.0)
      ),

      // DRIVE TO OTHER BALLS (diagonally)
      new LowerIntakeArmCommand(intakeSubsystem),
      new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.YEET3PUSHNOM3_DIAG_DISTANCE_TO_TRENCH, -.75, -45, false), 
        new StartIntakeCommand(intakeSubsystem, ()->1.0)
      ),

      // NOM/INTAKE 3 BALLS (FINAL PHASE) (also keep driving (parallel to balls and guardrail))
      new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.YEET3PUSHNOM3_3_BALL_STRAIGHT_DISTANCE, -.75, 0, false), 
        new StartIntakeCommand(intakeSubsystem, ()->1.0)
      )
      );
  }
}
