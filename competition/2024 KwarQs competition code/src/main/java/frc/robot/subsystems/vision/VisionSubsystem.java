package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;

public class VisionSubsystem extends SubsystemBase {
    private Vision visionInterface = new Vision();

    public VisionSubsystem() {
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
        if (pose.isEmpty())
            return null;
        else {
            return pose.get();
        }

    }

    public Matrix<N3, N1> getStandardDeviations() {
        return visionInterface.getEstimationStdDevs(getEstimatedRobotPose().estimatedPose.toPose2d());
    }

    public double getTimestampSeconds() {
        return visionInterface.getLatestResult().getTimestampSeconds();

    }

    public void simulationPeriodic(Pose2d pose) {
        visionInterface.simulationPeriodic(pose);
    }

    public Optional<Transform3d> getLatestResult() {
        PhotonTrackedTarget getLatestResult = visionInterface.getLatestResult().getBestTarget();
        return Optional.ofNullable(getLatestResult.getBestCameraToTarget());
    }
}
