package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;


public class VisionSubsystem  extends SubsystemBase {
    private Vision visionInterface = new  Vision();
    public VisionSubsystem(){
    }
    @Override
    public void periodic(){
        Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
        //System.out.println(pose);
    }
public EstimatedRobotPose getEstimatedRobotPose(){
    Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
    if(pose.isEmpty())
        return null;
    else{
        return pose.get();
    }
    
}    

public double getTimestampSeconds(){
    return visionInterface.getLatestResult().getTimestampSeconds();

}
}
