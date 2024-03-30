package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.vision.Vision;

public class VisionSubsystem extends SubsystemBase {
    private Vision visionInterface = new Vision();
    private Vision noteVision = new Vision(true);
    public double getLatestId = 0;
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    @Override
    public void periodic() {
        estimatedPose = visionInterface.getEstimatedGlobalPose();
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return estimatedPose;
    }

    public Optional<Matrix<N3, N1>> getStandardDeviations() {
        Optional<EstimatedRobotPose> pose = getEstimatedRobotPose();
        if (pose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(visionInterface.getEstimationStdDevs(pose.get().estimatedPose.toPose2d()));
    }

    public double getTimestampSeconds() {
        return visionInterface.getLatestResult().getTimestampSeconds();

    }

    public void simulationPeriodic(Pose2d pose) {
        visionInterface.simulationPeriodic(pose);
    }

    public Optional<Transform3d> getLatestResult() {
        if (!visionInterface.getLatestResult().hasTargets()) {
            return null;
        }
        PhotonTrackedTarget getLatestResult = visionInterface.getLatestResult().getBestTarget();
        if (getLatestResult != null) {
            getLatestId = getLatestResult.getFiducialId();
            // setLEDColor();
            return Optional.ofNullable(getLatestResult.getBestCameraToTarget());
        }
        return null;
    }
    public Optional<Transform3d> getLatestResult(boolean note) {
        if (noteVision.getLatestNoteResult() == null) {
            return null;
        }
        PhotonTrackedTarget getLatestResult = noteVision.getLatestNoteResult().getBestTarget();
        if (getLatestResult != null) {
            // setLEDColor();
            return Optional.ofNullable(getLatestResult.getBestCameraToTarget());
        }
        return null;
    }

    public boolean seesAprilTag() {
        if (getLatestResult() != null) {
            return true;
        }
        return false;
    }

    public double getOffset(){
        var result = noteVision.getLatestNoteResult();
        if (result == null) {
            return 0; //
        }
        double x = getX(result.getBestTarget().getPitch());
        double y = getY(result.getBestTarget().getYaw());
        
        double angle = Math.tan(x/y);
        return angle;
    }

    private double getX(double camx){
        double x = (.163 * Math.pow(camx, 2)) + (1.298 * camx) + 28.7; 
        return x;
    }

    private double getY(double camy){
        double y = (.0376 * Math.pow(camy, 2)) + 4.2598;
        return y;
    }

    public Rotation2d getTurn(){
        //gets how much the robot needs to turn to get the note in the center
        return new Rotation2d(getOffset());
    }

    // public Pose2d getNotePose(Pose2d robotPose){
    //     var result = noteVision.getLatestNoteResult();
    //     if (result != null) {
    //         //return 0; //
    //     }
    //     double x = robotPose.getX() + getX(result.getBestTarget().getPitch());
    //     double y = robotPose.getY() + getY(result.getBestTarget().getYaw());

        
    // }

    public void updateNoteV(){
         NTHelper.setBoolean("noteVision/connected", noteVision.isCameraConnected());
        var result = noteVision.getLatestNoteResult();
        if (result != null) {

                  NTHelper.setDouble("noteVision/x", result.getBestTarget().getPitch());
                  NTHelper.setDouble("noteVision/y", result.getBestTarget().getYaw());
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addBooleanProperty("has targets", () -> visionInterface.getLatestResult().hasTargets(), null);
        builder.addBooleanProperty("is connected", () -> visionInterface.getCamera().isConnected(), null);
        builder.addIntegerProperty("pipeline index", () -> visionInterface.getCamera().getPipelineIndex(), null);
        var result = getLatestResult(true);
        if (result != null) {
                  NTHelper.setDouble("noteVision/x", result.get().getX());
                  NTHelper.setDouble("noteVision/y", result.get().getY());

            // builder.addDoubleProperty("x", () -> getLatestResult(true).get().getX(), null);
            // builder.addDoubleProperty("y", () -> getLatestResult(true).get().getY(), null);
        }
    }
}
