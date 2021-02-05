package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {

    public float roll;
    public float pitch;
    public float yaw;

    private AHRS ahrs;
    private static GyroSubsystem instance = null;

    private GyroSubsystem() {
        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
             * details.
             */
            ahrs = new AHRS(); // PUT IN PORT!!!
            ahrs.enableBoardlevelYawReset(true);
            //ahrs.setAngleAdjustment(0.05);
            ahrs.reset();
        } catch (RuntimeException ex) {
            System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
            throw ex;
        }
    }
    
    //TODO: Make it threadsafe
    public static GyroSubsystem getInstance(){
        if (instance == null){
            instance = new GyroSubsystem();
        }
        return instance;
    }

    public void reset(){
        ahrs.reset();
        // ahrs.enableBoardlevelYawReset(true);
    }

    public void calibrationCheck() {

        boolean isCalibrating = ahrs.isCalibrating();
        if (isCalibrating) {
            Timer.delay(0.02);

        }
    }

    public float getRoll() {
        calibrationCheck();
        return ahrs.getRoll();
    }

    public float getPitch() {
        calibrationCheck();
        return ahrs.getPitch();
    }

    public float getYaw() {
        calibrationCheck();
        return ahrs.getYaw();
    }
    
}