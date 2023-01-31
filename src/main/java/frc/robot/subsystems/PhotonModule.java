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
  PhotonTrackedTarget target;

  public PhotonModule() {
    // Change this to match the name of your camera
    camera = new PhotonCamera("vision1");
    target = new PhotonTrackedTarget();
    target.getBestCameraToTarget();
    SmartDashboard.putNumber("targetID", target.getFiducialId());
    double yaw = target.getYaw();
    SmartDashboard.putNumber("targetYaw", yaw);

    double pitch = target.getPitch();
    SmartDashboard.putNumber("targetPitch", pitch);

    double area = target.getArea();
    SmartDashboard.putNumber("targetArea", area);

    double skew = target.getSkew();
    SmartDashboard.putNumber("targetSkew", skew);
    SmartDashboard.putNumber("targetID", target.getFiducialId());
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    // Check if the latest result has any targets.s
    if (result.hasTargets()) {

      SmartDashboard.putBoolean("getName()", hasTargets);

      // Get a list of currently tracked targets.
      List<PhotonTrackedTarget> targets = result.getTargets();
      // Get the current best target.
      target = result.getBestTarget();

      // Get information from target.
      SmartDashboard.putNumber("TargetCount", targets.size());

      // Get information from target.
      int targetId = target.getFiducialId();
      // double poseAmbiguity = target.getPoseAmbiguity();
      // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

      var yaw = target.getYaw();
      SmartDashboard.putNumber("targetYaw", yaw);
      var pitch = target.getPitch();
      SmartDashboard.putNumber("targetPitch", pitch);
      var area = target.getArea();
      SmartDashboard.putNumber("targetArea", area);
      var skew = target.getSkew();
      SmartDashboard.putNumber("targetSkew", skew);
      SmartDashboard.putNumber("targetID", target.getFiducialId());
    }
  }
}
// end periodic
