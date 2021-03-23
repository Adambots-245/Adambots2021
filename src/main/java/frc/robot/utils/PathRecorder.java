// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;


/** Add your docs here. */
public class PathRecorder {

    private static PathRecorder _instance = new PathRecorder();

    private PrintWriter file = null;
    
    public PathRecorder(){

    }

    public void createRecording(String filePath) throws IOException{

        // new File(filePath).delete();
        new File(filePath).createNewFile();
        file = new PrintWriter(new FileWriter(filePath));
    }

    public void stopRecording(){
        if (file != null)
            file.close();
    }

    public static PathRecorder getInstance(){
        return _instance;
    }

    public void record(double speed, double rotationSpeed){
        file.printf("%f,%f,%f,", System.currentTimeMillis(), speed, rotationSpeed);
        file.flush();
    }

}
