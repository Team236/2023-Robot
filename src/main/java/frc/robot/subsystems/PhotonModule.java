// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonModule extends SubsystemBase {
  /** Creates a new PhotonModule. */

  private static PhotonCamera camera;

  public PhotonModule() {
    // Change this to match the name of your camera
    camera = new PhotonCamera("photonvision");
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      // Get a list of currently tracked targets.
      List<PhotonTrackedTarget> targets = result.getTargets();
      // Get the current best target.
      PhotonTrackedTarget target = result.getBestTarget();

      // Get information from target.
      SmartDashboard.putNumber("TargetCount", targets.size());

      // Get information from target.
      //int targetID = target.getFiducialId();
      //double poseAmbiguity = target.getPoseAmbiguity();
      //Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      //Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

     double yaw = target.getYaw();
      SmartDashboard.putNumber("targetYaw", yaw);

      double pitch = target.getPitch();
      SmartDashboard.putNumber("targetPitch", pitch);

      double area = target.getArea();
      SmartDashboard.putNumber("targetArea", area);

      double skew = target.getSkew();
      SmartDashboard.putNumber("targetSkew", skew);

    /*   Transform2d pose = target.getPoseAmbiguity();
      SmartDashboard.putString("poseString", "x: " + pose.getX() + " y:" + pose.getY());

      List<TargetCorner> corners = target.getCorners();
      */
    } // end-if
  }
}
// end periodic
