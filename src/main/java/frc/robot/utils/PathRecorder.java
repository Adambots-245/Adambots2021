// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Record joystick movements as path instructions that can be played back.
 * 
 */
public class PathRecorder {

    private static PathRecorder _instance = new PathRecorder();
    private long startTime = 0L;

    private HashMap<String, ArrayList<String>> paths = new HashMap<>();
    private HashMap<String, PrintWriter> files = new HashMap<>();
    private long line = 1;
    private static ShuffleboardTab tab = Shuffleboard.getTab("Auton");
    private static SimpleWidget fileName = tab.add("RecordMacroTo", "autonPath");
    private ArrayList<String> currentPath = null;

    public PathRecorder(){
        SmartDashboard.putString("Recording to File", "");
    }

    /**
     * Creates a new recording and saves it to a new file in the lvuser folder of the RoboRIO.
     * If a file already exists with the specified file name, an unique ID is appended to the end of the new save file.
     * This method clears the file and path caches, so it can only record one path at a time without stopping.
     */
    public void createRecording(){

        files = new HashMap<>();
        paths = new HashMap<>();
        addNewPath(true);

    }

    /**
     * Adds a new recording and saves it to a new file in the lvuser folder of the RoboRIO.
     * Increments the number at the end of the specified file name to signify additional parts of a single path.
     * Useful for breaking up a single path into multiple segments, which increases accuracy and makes manual tuning easier.
     * This method does not clear the file and path caches, so it can record more than one path at a time without stopping.
     */
    public void addRecording() {
        addNewPath(false);
    }

    /**
     * Stops recording, and saves all cached recordings to their respective files.
     */
    public void stopRecording(){

        int totalPoints = 0;
        int totalPaths = 0;

        for (String path : paths.keySet()) {

            ArrayList<String> pathPoints = paths.get(path);
            PrintWriter file = files.get(path);
            totalPaths++;

            for (String pathPoint : pathPoints) {
                file.printf("%s%n", pathPoint);
                totalPoints++;
            }

            if (file != null) {
                file.flush();
                file.close();
                file = null;
                files.put(path, file);
                paths.put(path, null);
            }

        }

        files = new HashMap<>();
        paths = new HashMap<>();
        System.out.println("Finished saving recordings.\nTotal path points saved: " + totalPoints + "\nTotal paths saved: " + totalPaths);

    }

    /**
     * @return The name of the file to save the current recording to.
     */
    public static String getFileNameFromSmartDashboard(){

        return fileName.getEntry().getString("autonPath");
    }

    /**
     * Sets the name of the file to save the current recording to.
     * @param name - The name of the file (without path or file extension)
     */
    public static void setFileNameOnSmartDashboard(String name) {
        fileName.getEntry().setString(name);
    }

    public static PathRecorder getInstance(){
        return _instance;
    }

    /**
     * Gets the list of path points for the current recording.
     * @return ArrayList of path points.
     */
    private ArrayList<String> getCurrentPath() {
        return paths.get(getFileNameFromSmartDashboard());
    }

    /**
     * Gets the save file for the current recording.
     * @return PrintWriter for recording file.
     */
    private PrintWriter getCurrentFile() {
        return files.get(getFileNameFromSmartDashboard());
    }

    /**
     * Adds a new path for recording.
     * Creates an unique file for the new recording, and constructs a new PrintWriter and path point ArrayList.
     * Automatically alters the recording name on the SmartDashboard.
     * @param noIncrement - Whether or not to increment the number at the end of the recording's name.
     */
    private void addNewPath(boolean noIncrement) {

        String name = getFileNameFromSmartDashboard();
        int unique_id = 1;
        final String ext = Constants.RECORDING_FILE_EXT;

        if (name.substring(name.length() - 1).chars().allMatch( Character::isDigit )) {
            String num = "" + (Integer.parseInt(name.replaceAll("[^0-9]", "")));
            String path_name = name.replaceAll("[0-9]", "");

            if (!noIncrement) num = "" + (Integer.parseInt(num) + 1);

            if (num.length() == 1) num = "0" + num;
            name = path_name + num;
        } 
        else name += "01";

        setFileNameOnSmartDashboard(name);
        SmartDashboard.putString("Recording to File", name);

        String filePath = Constants.ROBOT_HOME_FOLDER + name;
        PrintWriter writer;
        File rawFile = new File(filePath + "." + unique_id + ext);

        while (rawFile.isFile()) {
            unique_id++;
            rawFile = new File(filePath + "." + unique_id + ext);
        } 

        try {
            rawFile.createNewFile();
            writer = new PrintWriter(new FileWriter(filePath + "." + unique_id + ext));
        } catch (IOException e) {
            e.printStackTrace();
            writer = null;
        }

        files.put(name, writer);
        paths.put(name, new ArrayList<>());
        currentPath = paths.get(name);

        line = 1;
        startTime = 0L;

    }

    public void record(double speed, double rotationSpeed){

        if (getCurrentFile() == null)
            return;

        long currentTime = System.currentTimeMillis();

        if (startTime == 0)
            startTime = currentTime;
        
        long delay = currentTime - startTime;
    
        // getCurrentPath().add(String.format("%d,%f,%f", delay, speed, rotationSpeed));
        currentPath.add(String.format("%d,%f,%f", delay, speed, rotationSpeed));

        if (delay > 25){
            // these lines where the robot skipped 20 ms delay should be manually inspected and corrected
            System.out.println("WARNING: Delay > 25 at line: " + this.line + " in path: " + getFileNameFromSmartDashboard());
        }

        line++;
        startTime = currentTime;

    }
}
