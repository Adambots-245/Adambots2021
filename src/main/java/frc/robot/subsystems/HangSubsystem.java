
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class HangSubsystem extends SubsystemBase {

    // TODO Add hangmotor Constant
    public WPI_VictorSPX hangMotor;
    public WPI_VictorSPX winchMotor1;
    public WPI_VictorSPX winchmotor2;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;

    public HangSubsystem() {
        super();
        hangMotor = new WPI_VictorSPX(Constants.CLIMBING_RAISE_ELEVATOR_MOTOR_PORT);
        winchMotor1 = new WPI_VictorSPX(Constants.CLIMBING_1_MOTOR_PORT);
        winchmotor2 = new WPI_VictorSPX(Constants.CLIMBING_2_MOTOR_PORT);

        winchmotor2.setInverted(true);
        limitSwitch1 = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
        limitSwitch2 = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);
    }

    public void climb(double speed, JoystickButton overrideButton) {
        if (!limitSwitch1.get() && !limitSwitch2.get() && speed >= 0 && !overrideButton.get()) {
            hangMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
        } else {
            hangMotor.set(ControlMode.PercentOutput, speed);
        }
        // System.out.println(overrideButton.get());
    }

    public void winchDown() {
        // TODO: add actual constants for the winch mechanism
        winchmotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
    }

    public void winchOff() {
        winchmotor2.set(ControlMode.PercentOutput, 0);
        winchMotor1.set(ControlMode.PercentOutput, 0);

    }

    public void periodic() {

    }

}
