package frc.robot.Subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Vision {
    public static PhotonCamera photonCannon;

    private static Vision m_Instance = null;

    private boolean isReady = false;

    public Vision() {
        createNewStream();

        if (isReady) {
            System.out.println("Photon Vision is Ready to Roll!!");
        } else {
            System.out.println("Photon Vision is NOT READY!!");
        }

    }

    // Creates New Vision Stream
    public void createNewStream() {
        try {
            photonCannon = new PhotonCamera("photonvision");
            isReady = true;
        } catch (Exception e) {
            System.out.println("Can't Create Photon Vision Camera Stream");
            isReady = false;
        }
    }

    // Returns the Range to the Target
    public double getRangeToTarget() {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.PhotonCannon.CAMERA_HEIGHT_METERS,
                Constants.PhotonCannon.TARGET_HEIGHT_METERS,
                Constants.PhotonCannon.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getTargets().getBestTarget().getPitch()));
        return range;
    }

    // Returns the Yaw of Targets
    public double getYawOfTargets() {
        double yaw = getTargets().getBestTarget().getYaw();
        return yaw;
    }

    // Returns if Targets are Present
    public boolean hasTargets() {
        return getTargets().hasTargets();
    }

    // Returns Latest Result
    public PhotonPipelineResult getTargets() {
        return photonCannon.getLatestResult();
    }

    // Turn off LEDs on Limelight Hardware
    public void ledOn(boolean off) {
        if (off == true) {
            photonCannon.setLED(VisionLEDMode.kOff);
        } else {
            photonCannon.setLED(VisionLEDMode.kOn);
        }
    }

    // Returns Current Instance of PhotonCannon
    public static Vision getInstance() {
        if (m_Instance == null) {
            m_Instance = new Vision();
        }

        return m_Instance;
    }

}