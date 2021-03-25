package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class PhotoEye extends BaseSensor {
    private DigitalInput photoEye;
    public PhotoEye (int port) {
        this.photoEye = new DigitalInput(port);
    }

    public boolean isDetecting() {
        return photoEye.get();
    }
}
