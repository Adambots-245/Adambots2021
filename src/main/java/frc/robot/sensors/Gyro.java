/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class that represents a Gyroscope sensor
 */
public class Gyro extends BaseSensor implements edu.wpi.first.wpilibj.interfaces.Gyro {

    private static Gyro _instance = null;
    private static AHRS _navx = null;

    private static class InstanceHolder {
        public static final Gyro instance = new Gyro();
    }

    private Gyro(){
        try {
            if (_navx == null){
                _navx = new AHRS(); // although this brings in depency, using setDevice this can be overwritten before calling getInstance
            }

            _navx.enableBoardlevelYawReset(true);
            _navx.reset();
        } catch (Exception e) {
            System.out.println("NavX Initialization Failed");
        }
    }

    public static void setDevice(AHRS navx){
        
        if (_navx == null)
            _navx = navx; // It should only be set once.
    }

    public static Gyro getInstance(){
        // if (_instance == null){
        //     _instance = new Gyro();
        // }
        _instance = InstanceHolder.instance;
        return _instance;
    }

    public void reset(){
        _navx.reset();
        // _navx.enableBoardlevelYawReset(true);
    }

    public void lowLevelReset(){
        _navx.enableBoardlevelYawReset(true);
        _navx.reset();
    }

    public void calibrationCheck() {

        boolean isCalibrating = _navx.isCalibrating();
        
        if (isCalibrating) {
            Timer.delay(0.02); // wait 0.02 seconds to let it complete calibration
        }
    }

    public float getRoll() {
        calibrationCheck();
        return _navx.getRoll();
    }

    public float getPitch() {
        calibrationCheck();
        return _navx.getPitch();
    }

    public float getYaw() {
        calibrationCheck();
        return _navx.getYaw();
    }

    @Override
    public void close() throws Exception {
        _navx.close();

    }

    @Override
    public void calibrate() {
        _navx.enableBoardlevelYawReset(true);

    }

    @Override
    public double getAngle() {
        return getYaw();
    }

    @Override
    public double getRate() {
        return _navx.getRate();
    }
}
