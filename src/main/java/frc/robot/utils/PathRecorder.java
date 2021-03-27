// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Date;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Record joystick movements as path instructions that can be played back.
 * 
 */
public class PathRecorder {

    private static PathRecorder _instance = new PathRecorder();
    private long startTime = 0L;

    private PrintWriter file = null;
    private ArrayList<String> pathPoints = new ArrayList<String>();
    private long line = 1;
    private static ShuffleboardTab tab = Shuffleboard.getTab("Auton");
    private static NetworkTableEntry fileName = tab.add("RecordMacroTo", "autonPath").getEntry();

    public PathRecorder(){
        SmartDashboard.putString("Recording to File", "");
        // SmartDashboard.putString("Record Macro To", "autonPath");
    }

    public void createRecording(String filePath){

        SmartDashboard.putString("Recording to File", filePath);

        startTime = 0;

        int unique_id = (int) ((new Date().getTime() / 1000L) % Integer.MAX_VALUE); 

        filePath = Constants.ROBOT_HOME_FOLDER + filePath + "-" + unique_id + ".txt";
        // new File(filePath).delete();
        try {
            new File(filePath).createNewFile();
            file = new PrintWriter(new FileWriter(filePath));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void stopRecording(){
        if (file != null){

            for (String pathPoint : pathPoints) {
                file.printf("%s%n", pathPoint);
            }
            file.flush();
            file.close();
        }
    }

    public static String getFileNameFromSmartDashboard(){

        return fileName.getString("autonPath");
    }

    public static PathRecorder getInstance(){
        return _instance;
    }

    public void record(double speed, double rotationSpeed){
        // System.out.println("Writing to file: " + speed);

        if (file == null)
            return;

        long currentTime = System.currentTimeMillis();

        if (startTime == 0)
            startTime = currentTime;
        
        long delay = currentTime - startTime;
        
        // file.printf("%d,%f,%f%n", delay, speed, rotationSpeed);
        // file.flush();
        pathPoints.add(String.format("%d,%f,%f", delay, speed, rotationSpeed));

        if (delay > 25){
            // these lines where the robot skipped 20 ms delay should be manually inspected and corrected
            System.out.println("WARNING: Delay > 25 at line: " + this.line);
        }

        line++;
        startTime = currentTime;
    }
}
