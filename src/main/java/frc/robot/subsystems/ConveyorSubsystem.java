/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Log;

public class ConveyorSubsystem extends SubsystemBase {
  /**
   * Creates a new ConveyorSubsystem.
   */
  private WPI_VictorSPX conveyorMotor;
  private WPI_VictorSPX alignmentBeltMotor;

  public ConveyorSubsystem(WPI_VictorSPX conveyorMotor, WPI_VictorSPX alignmentBeltMotor) {
    super();
    
    this.conveyorMotor = conveyorMotor; //new WPI_VictorSPX(Constants.INFEED_CONVEYOR_MOTOR_PORT);
    this.alignmentBeltMotor = alignmentBeltMotor; //new WPI_VictorSPX(Constants.INFEED_CONVEYOR_INDEXER_MOTOR_PORT);

    Log.info("Initializing Conveyor");
  }

  public void runConveyor(double speed){

    //Log.infoF("Running coveyor. %% Output: %f", speed);
    conveyorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runAlignmentBelt(double speed){

    //Log.infoF("Running Alignment Belt. %% Output: %f", speed);
    alignmentBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
