// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PathFollower extends CommandBase {
  private String filePath;
  private DriveTrainSubsystem drive;
  private Scanner fileScanner;
  private boolean finishedFlag = false;

  /**
   * Follows path created by PathRecorder by sending the same joystick coordinates to the drive subsystem
   * 
   * @param filePath - Absolute filepath of recording
   * @param drive - DriveTrainSubsystem instance
   */
  public PathFollower(String filePath, DriveTrainSubsystem drive) {

    this.filePath = filePath;
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    File file = new File(filePath);

    try {
      this.fileScanner = new Scanner(file);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      this.finishedFlag = true;
    }
  }

  public static boolean matchesRecorderNameFormat(String string) {
    final String regex = "\\D+\\d+\\.\\d+";
        
    final Pattern pattern = Pattern.compile(regex, Pattern.CASE_INSENSITIVE);
    final Matcher matcher = pattern.matcher(string);

    return matcher.find();
  }

  public static boolean isValidSegmentedPathName(String pathName) {
    return pathName != null && !pathName.contains(".") && !pathName.contains(".txt");
  }

  public static Command fromSegmentedPath(String pathName, DriveTrainSubsystem drive) {
    Command output = null;
    ArrayList<String> paths = new ArrayList<>(List.of(getListofRecordings()));
    final boolean isValidPathName = isValidSegmentedPathName(pathName);

    //Sort lexicographically 
    paths.sort(String::compareToIgnoreCase);

    if (isValidPathName) {
      for (String absolutePath : paths) {
        //Original format: "/<home_path>/<rec_name>.<id>.<ext>"

        //Gets name in the format: "<rec_name>.<id>"
        String name = absolutePath.substring(absolutePath.lastIndexOf("/") + 1);
        name = name.substring(0, name.lastIndexOf("."));

        //Check if this file matches the PathRecorder name format:
        if (!matchesRecorderNameFormat(name)) continue;

        //Gets name in the format: "<rec_name>"
        //Gets identifier in the format: <id>
        int identifier = Integer.parseInt(name.substring(name.lastIndexOf(".") + 1));
        name = name.substring(0, name.lastIndexOf("."));

        //The successor of this path, aka a re-recording of the current path.
        //For example, if this path is: "barrelpath3.1.txt"
        //The successor is: "barrelpath3.2.txt"
        String successor = Constants.ROBOT_HOME_FOLDER + name + "." + (identifier + 1) + Constants.RECORDING_FILE_EXT;

        //Checks to pass in order to add this path to the output:
        boolean hasSuccessor = false;
        boolean isSegmentOfPath = false;

        if (name.contains(pathName)) isSegmentOfPath = true;
        if (isSegmentOfPath && (new File(successor)).isFile()) hasSuccessor = true;

        //If both checks pass, add this path to output
        if (isSegmentOfPath && !hasSuccessor) {

          if (output == null) output = new PathFollower(absolutePath, drive);
          else output.andThen(new PathFollower(absolutePath, drive));

        }

      }
    }
    else if (pathName != null) {
      SmartDashboard.putString("Playback: Auton Macro File", "Failed: invalid path name specified");
    }
    else {
      SmartDashboard.putString("Playback: Auton Macro File", "Failed: specified path name was null");
    }

    if (output == null) return new InstantCommand();
    return output;

  }

  public static String getLastRecordedFile(){
    File dir = new File (Constants.ROBOT_HOME_FOLDER);

    File[] files = dir.listFiles(File::isFile);
    long lastModifiedTIme = Long.MIN_VALUE;
    
    String chosenFile = null;

    if (files == null || files.length == 0)
      return chosenFile;
    else{
      for (File file:files){
        String fileName = file.getName();
        String ext = fileName.substring(fileName.lastIndexOf("."));
        if (ext.equals(Constants.RECORDING_FILE_EXT) && file.lastModified() > lastModifiedTIme)
        {
          chosenFile = file.getAbsolutePath();
          lastModifiedTIme = file.lastModified();
        }
      }
    }

    return chosenFile;
  }

  public static String[] getListofRecordings(){
    File dir = new File (Constants.ROBOT_HOME_FOLDER);

    File[] files = dir.listFiles(File::isFile);
    
    ArrayList<String> list = new ArrayList<String>();

    if (files!= null && files.length != 0)
    {
      for (File file:files){
        String fileName = file.getName();
        int extPos = fileName.lastIndexOf(".");

        if (extPos == -1)
          continue;

        String ext = fileName.substring(extPos);

        if (ext.equals(Constants.RECORDING_FILE_EXT)){
          list.add(file.getAbsolutePath());
        }
      }
    }

    return list.toArray(new String[list.size()]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Playback: Auton Macro File", filePath);

    if (fileScanner.hasNextLine()){
      String line = fileScanner.nextLine();

      String[] fields = line.split(",");

      // int delay = Integer.valueOf(fields[0]);
      double speed = Double.valueOf(fields[1]);
      double turnSpeed = Double.valueOf(fields[2]);

      // System.out.printf("Auto drive %f:%f%n", speed, turnSpeed);
      // SmartDashboard.putNumber("Auton Speed", speed);
      // SmartDashboard.putNumber("Auton Turn Speed", turnSpeed);

      drive.arcadeDrive(speed, turnSpeed);
    }
    else{
      finishedFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fileScanner.close();
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finishedFlag)
      return true;
    else
      return false;
  }
}
