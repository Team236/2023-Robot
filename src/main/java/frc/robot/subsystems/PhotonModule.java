// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonModule extends SubsystemBase {
  /** Creates a new PhotonModule. */
  public PhotonModule() {
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result();

    if (hasTargets) {
      // Get a list of currently tracked targets.
      List<PhotonTrackedTarget> targets = result.getTargets();
      // Get the current best target.
      PhotonTrackedTarget target = result.getBestTarget();

      // Get information from target.
      SmartDashboard.putNumber("TargetCount", targets.size());

      // Get information from target.
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

      double yaw = targets.getYaw();
      SmartDashboard.putNumber("targetYaw", yaw);

      double pitch = targets().getPitch();
      SmartDashboard.putNumber("targetPitch", pitch);

      double area = targets.getArea();
      SmartDashboard.putNumber("targetArea", area);

      double skew = targets.getSkew();
      SmartDashboard.putNumber("targetSkew", skew);

      Transform3d pose = targets.get();
      SmartDashboard.putString("poseString", "x: " + pose.getX() + " y:" + pose.getY());

      List<TargetCorner> corners = targets.getCorners();
    } // end-if

  }
// end periodic
