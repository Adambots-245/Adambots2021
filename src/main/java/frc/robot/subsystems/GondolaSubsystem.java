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
import frc.robot.Constants;

public class GondolaSubsystem extends SubsystemBase {
  /**
   * Creates a new GondolaSubsystem.
   */
  public static WPI_VictorSPX gondola;

  public GondolaSubsystem() {
    super();

    gondola = new WPI_VictorSPX(Constants.CLIMBING_GONDOLA_ADJUSTMENT_MOTOR_PORT);

  }
  public void gondola(double speed) {
    gondola.set(ControlMode.PercentOutput, speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
