package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Vision {
    public static PhotonCamera photonCannon;

    //Creates New Vision Stream
    public void createNewStream(){
        try {
            photonCannon = new PhotonCamera("photonvision"); 
            System.out.println("Created New Stream");
        } catch (Exception e) {
            System.err.println("Can't Recreate Existing Camera Stream");
        }
    }

    //Returns the Range to the Target
    public double getRangeToTarget(){
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.PhotonCannon.CAMERA_HEIGHT_METERS,
            Constants.PhotonCannon.TARGET_HEIGHT_METERS,
            Constants.PhotonCannon.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getTargets().getBestTarget().getPitch()));
            return range;
    }

    //Returns the Yaw of Targets
    public double getYawOfTargets(){
        double yaw = getTargets().getBestTarget().getYaw();
        return yaw;
    }

    //Returns if Targets are Present
    public boolean hasTargets(){
        return getTargets().hasTargets();
    }

    //Returns Latest Result
    public PhotonPipelineResult getTargets(){
        return photonCannon.getLatestResult();
    }

    //Turn off LEDs on Limelight Hardware
    public void ledOn(boolean off){
        if (off == true){
            photonCannon.setLED(VisionLEDMode.kOff);
        } else {
            photonCannon.setLED(VisionLEDMode.kOn);
        }
    }

}
