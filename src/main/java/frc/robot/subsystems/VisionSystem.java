// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

  public VisionSystem() {
    camera = new PhotonCamera("Photon Camera");
  }

  public PhotonPipelineResult getLatestResult() {
    PhotonPipelineResult latestResult = camera.getLatestResult();
    return latestResult;
  }

  public List<PhotonTrackedTarget> getTargets() {
    return getLatestResult().getTargets();
  }

  public boolean hasTargets() {
    boolean hasTargets = getLatestResult().hasTargets();
    return hasTargets;
  }

  public Vector<N3> getStandardDeviations(EstimatedRobotPose robotPose, double averageDistance, int tagCount) {
    double stdDevScale = 1 + (averageDistance * averageDistance / 30);

    if (tagCount > 1) {
      Vector<N3> standardDeviation =
          VecBuilder.fill(Units.inchesToMeters(4.5), Units.inchesToMeters(4.5), Double.MAX_VALUE);

      return standardDeviation.times(stdDevScale);
    } else {
      double xyStandardDev = VisionConstants.xyPolynomialRegression.predict(averageDistance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, Double.MAX_VALUE);
    }
  }

  public int getTagCount() {
    int tagCount = 0;

    Optional<EstimatedRobotPose> optionalVisionPose =
        poseEstimator.update(getLatestResult());
    
    EstimatedRobotPose visionPose = optionalVisionPose.get();
    
    for (PhotonTrackedTarget target : visionPose.targetsUsed) {
      tagCount++;
    }

    return tagCount;
  }

  public double getTagTotalDistance() {
    double totalDistance = 0;

    Optional<EstimatedRobotPose> optionalVisionPose =
        poseEstimator.update(getLatestResult());
    
    EstimatedRobotPose visionPose = optionalVisionPose.get();
    
    for (PhotonTrackedTarget target : visionPose.targetsUsed) {
      totalDistance += target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    return totalDistance;
  }

  public Pose2d getPose() {
    PhotonTrackedTarget target = getLatestResult().getBestTarget();

    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
      target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);

    return robotPose.toPose2d();
  }

  public EstimatedRobotPose getPoseFromVisionPoseEstimator() {
    Optional<EstimatedRobotPose> optionalVisionPose =
        poseEstimator.update(getLatestResult());
    
    EstimatedRobotPose estimatedRobotPose = optionalVisionPose.get();

    return estimatedRobotPose;
  }

  public double getLatency() {
    double latency = getLatestResult().getLatencyMillis()*0.001;
    return latency;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
