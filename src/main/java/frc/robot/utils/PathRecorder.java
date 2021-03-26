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

/**
 * Record joystick movements as path instructions that can be played back.
 * 
 */
public class PathRecorder {

    private static PathRecorder _instance = new PathRecorder();
    private long startTime = 0L;

    private PrintWriter file = null;
    private ArrayList<String> pathPoints = new ArrayList<String>();
    
    public PathRecorder(){

    }

    public void createRecording(String filePath){

        startTime = 0;

        int unique_id = (int) ((new Date().getTime() / 1000L) % Integer.MAX_VALUE); 

        filePath = "/home/lvuser/" + filePath + "-" + unique_id + ".txt";
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

        startTime = currentTime;
    }
}
