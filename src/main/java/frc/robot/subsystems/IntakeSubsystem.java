/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private static DoubleSolenoid armRaiseLower;
  private WPI_VictorSPX IntakeMotor;
  private WPI_VictorSPX ConveyorMotor;
  private WPI_VictorSPX ConveyorIndexerMotor;
  private WPI_VictorSPX FeedToBlasterMotor;
  private DigitalInput IntakePhotoEye = new DigitalInput(Constants.INTAKE_PHOTO_EYE_DIO);
  private DigitalInput ConveyorPhotoEye = new DigitalInput(Constants.CONVEYOR_PHOTO_EYE_DIO);
  private DigitalInput BlasterPhotoEye = new DigitalInput(Constants.BLASTER_PHOTO_EYE_DIO);

  public IntakeSubsystem() {
    super();
    armRaiseLower = new DoubleSolenoid(Constants.RAISE_POWER_CELL_INTAKE_SOL_PORT, Constants.LOWER_POWER_CELL_INTAKE_SOL_PORT); // raise = kforward lower = kreverse
    IntakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR_PORT);
    FeedToBlasterMotor = new WPI_VictorSPX(Constants.FEED_TO_BLASTER_MOTOR_PORT);
    FeedToBlasterMotor.setInverted(true);
    IntakeMotor.setInverted(true);
  }

  public void intake(double speed) {
    IntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void outtake() {
    IntakeMotor.set(ControlMode.PercentOutput, Constants.OUTTAKE_SPEED);
  }
  public void conveyor(double conveyorSpeed) {
    ConveyorMotor.set(ControlMode.PercentOutput, conveyorSpeed);
  }

  public void indexToConveyor(double indexToConveyorSpeed) {
    ConveyorIndexerMotor.set(ControlMode.PercentOutput, indexToConveyorSpeed);
  }

  public void feedToBlaster() {
    FeedToBlasterMotor.set(ControlMode.PercentOutput, Constants.FEED_TO_BLASTER_SPEED);
  }
  public void reverseFeedToBlaster(){
    FeedToBlasterMotor.set(ControlMode.PercentOutput, -Constants.FEED_TO_BLASTER_SPEED);
  }
  public void stopIndex() {
    FeedToBlasterMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
  }
  public void stop(){
    IntakeMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
  }


  public void RaiseIntake() {
      // Supposedly, if(D-Pad up is pressed)
      //if (GamepadConstants.AXIS_DPAD_POV==true)
      // lol I tried
      armRaiseLower.set(Value.kForward);
      
  }
  
  public void LowerIntake(){
      //Supposedly, if(D-Pad down is pressed)
      //if (GamepadConstants.BUTTON_RB==false)
      //lol I tried
          armRaiseLower.set(Value.kReverse);
      
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
