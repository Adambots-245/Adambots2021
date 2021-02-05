/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Color Sensor dependencies:
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;


//FMS dependencies:
import edu.wpi.first.wpilibj.DriverStation;


//Motor dependencies:
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanel Subsystem.
   */
 

  private String lastColor;
  public WPI_TalonSRX panelMotor;
  private boolean rotationsFinished;
  private boolean alignerFinished;

  public ControlPanelSubsystem() {
    super();

    //Init control panel code
    Constants.M_COLOR_MATCHER.addColorMatch(Constants.BLUE_TARGET);
    Constants.M_COLOR_MATCHER.addColorMatch(Constants.GREEN_TARGET);
    Constants.M_COLOR_MATCHER.addColorMatch(Constants.RED_TARGET);
    Constants.M_COLOR_MATCHER.addColorMatch(Constants.YELLOW_TARGET);

    lastColor = "Unknown";
    rotationsFinished = false;
    alignerFinished = false;

    panelMotor = new WPI_TalonSRX(Constants.PANEL_MOTOR_PORT);
  }

  //Gets the color that our color sensor currently detects
  public String getColor() {

        

    final Color detectedColor = Constants.M_COLOR_SENSOR.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    final ColorMatchResult match = Constants.M_COLOR_MATCHER.matchClosestColor(detectedColor);

    if (match.color == Constants.BLUE_TARGET) {
      colorString = "Blue";
    } else if (match.color == Constants.RED_TARGET) {
      colorString = "Red";
    } else if (match.color == Constants.GREEN_TARGET) {
      colorString = "Green";
    } else if (match.color == Constants.YELLOW_TARGET) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    //If beyond 3 inches, detected color is inaccurate
    if (getProximity() < 100) {
        colorString = "Unknown";
    }

    //Account for inaccurate colors detected in transition between two colors
    if (!mapNextColor(lastColor).equals(colorString) && !lastColor.equals("Unknown")) {
        //If the detected color does not appropriately match the predicted color to come after the last color,
        //And both the last color and direction clockwise/counter-clockwise are known,
        //Then stick to the value of the lastColor.
        colorString = lastColor;
    }

    lastColor = colorString;

    return colorString;
}

//Returns the direction we are spinning in
public String getDirection() {
    return Constants.DIRECTION;
}

//Predicts the next color our sensor will detect given our current color and the direction we are spinning
public String mapNextColor(String color) {
    final String currentColor = color;
    final String currentDirection = getDirection();
    String nextColor;

    final int index = Arrays.asList(Constants.COLOR_ORDER).indexOf(currentColor);
    int newIndex;

    if (currentDirection.equals("Clockwise")) {
        if (index + 1 > Constants.COLOR_ORDER.length - 1) {
            newIndex = 0;
        }
        else {
            newIndex = index + 1;
        }

        nextColor = Constants.COLOR_ORDER[newIndex];
    }
    else if (currentDirection.equals("Counterclockwise")) {
        if (index - 1 < 0) {
            newIndex = Constants.COLOR_ORDER.length - 1;
        }
        else {
            newIndex = index - 1;
        }

        nextColor = Constants.COLOR_ORDER[newIndex];
    }
    else {
        nextColor = "Unknown";
    }

    return nextColor;
}

//This method gets the proximity of the color sensor to the wheel
public double getProximity() {
    return Constants.M_COLOR_SENSOR.getProximity();
}

//Starts spinning
public void startMotor(Modes mode) {
    panelMotor.set(ControlMode.PercentOutput, mode == Modes.Rotations? Constants.PANEL_MOTOR_SPEED_ROTATION : Constants.PANEL_MOTOR_SPEED_ALIGNMENT);
}

//Stops spinning
public void stopMotor() {
    panelMotor.set(ControlMode.PercentOutput, 0.0);
}


private String rotationalStartingColor;
private int rotationalColorCount;
private boolean offStartingColor = false;

//For the first phase of the control panel: rotating the wheel 3-5 times
public void startRotations(Modes mode) {
    //We can use the color our sensor is detecting as opposed to the game's sensor, it will still work:
    rotationalStartingColor = getColor();
    rotationalColorCount = 0;
    offStartingColor = false;
    rotationsFinished = false;

    SmartDashboard.putBoolean("Start Rotations", true);
    //start rotating control wheel motor
    startMotor(mode);
}

//This method returns the number of rotations of the color wheel
public int getRotations() {
    return rotationalColorCount;
}

//This method monitors the rotations of the wheel and stops rotating once 3 rotations are complete
public void monitorRotations() {

    SmartDashboard.putNumber("Rotational Color Count", rotationalColorCount);

    if (rotationalStartingColor.equals(getColor()) && offStartingColor) {
        rotationalColorCount++;
        offStartingColor = false;
    }
    if (!rotationalStartingColor.equals(getColor())) {
        offStartingColor = true;
    }

    if (getRotations() >= Constants.MIN_ROTATIONS) {
        //stop rotating
        rotationsFinished = true;
        stopMotor();
        SmartDashboard.putBoolean("Start Rotations", false);

    }
}

public boolean isFinished(Modes event) {
    if (event == Modes.Rotations) {
        return rotationsFinished;
    }
    else if(event == Modes.Alignment) {
        return alignerFinished;
    }
    else {
        return false;
    }
}

//Gets color provided by FMS
public String getFmsColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
        case 'B':
            return "Blue";
        case 'G':
            return "Green";
        case 'R':
            return "Red";
        case 'Y':
            return "Yellow";
        default:
            // This is corrupt data
            return "Corrupt Data";
        }
    } else {
        // Code for no data received yet
        return "Unknown";
    }
}


private String targetColor;

/* For the second/final phase of control panel: aligning the game's sensor to a specific color on the panel */
public void startAligner(Modes mode) {
    targetColor = getFmsColor();
    alignerFinished = false;

    SmartDashboard.putString("FMS Color", targetColor);
    SmartDashboard.putBoolean("Start Alignment", true);
    //Start rotating
    startMotor(mode);
}

//Gets the distance, in color slices, between our sensor and the game sensor
public int getDifferential() {
    return Constants.DIFFERENTIAL;
}

//Gets the color that is two (or whatever the differential is) color slices away from our sensor's position
public String colorCorrector(String currentColor) {
    if (getDifferential() == 2) {
        return mapNextColor(mapNextColor(currentColor));
    }
    else if (getDifferential() == 3) {
        return mapNextColor(mapNextColor(mapNextColor(currentColor)));
    }
    else {
        return mapNextColor(currentColor);
    }
}

//Stops rotating once the correct color is reached
public void monitorAligner() {
    boolean isTarget = false;
    
    if (targetColor.equals(colorCorrector(getColor()))) {
        isTarget = true;
    }
    else {
        isTarget = false;
    }

    if (isTarget) {
        stopMotor();
        alignerFinished = true;

        SmartDashboard.putBoolean("Start Alignment", false);

    }
    
}



//This method puts stuff on the dashboard (only use if necessary) for first phase
public void putDashRotations() {

    SmartDashboard.putString("Detected Color", getColor());
    SmartDashboard.putString("Predicted Next Color", mapNextColor(getColor()));
    SmartDashboard.putNumber("Rotations", getRotations());

}

//This method puts stuff on the dashboard (only use if necessary) for second phase
public void putDashAligner() {

    SmartDashboard.putString("Detected Color", getColor());
    SmartDashboard.putString("Predicted Gamesensor Color", colorCorrector(getColor()));
    SmartDashboard.putString("Target Gamesensor Color", targetColor);

}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static enum Modes{
    Rotations,
    Alignment
  }
}
