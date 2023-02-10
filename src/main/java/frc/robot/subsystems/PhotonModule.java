// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonModule extends SubsystemBase {
  /** Creates a new PhotonModule. */

  private static PhotonCamera camera;
  PhotonTrackedTarget target;

  public PhotonModule() {
    // Change this to match the name of your camera
    camera = new PhotonCamera("OV5647");
    target = new PhotonTrackedTarget();

    SmartDashboard.putNumber("targetID", target.getFiducialId());
    SmartDashboard.putNumber("targetYaw", target.getYaw());
    SmartDashboard.putNumber("targetPitch",  target.getPitch());
    SmartDashboard.putNumber("targetArea", target.getArea());

    // var skew = target.getSkew();
    // SmartDashboard.putNumber("targetSkew", skew);
    // SmartDashboard.putNumber("targetID", target.getFiducialId());
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var target = camera.getLatestResult();
    // boolean hasTargets = target.hasTargets();
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOn);

    
    SmartDashboard.putNumber("pipeline_index", camera.getPipelineIndex() );

    // Check if the latest result has any targets
    if (target.hasTargets()) {

      // Get a list of currently tracked targets
      // List<PhotonTrackedTarget> targetList = target.getTargets();

      // Get the current best target
      var result = target.getBestTarget();

      // Get information from target
      // SmartDashboard.putNumber("TargetCount", targets.size());

      // Get information from target
      
      int targetId = result.getFiducialId();
      SmartDashboard.putNumber("targetID", targetId);

      var yaw = result.getYaw();
      SmartDashboard.putNumber("targetYaw", yaw);

      var pitch = result.getPitch();
      SmartDashboard.putNumber("targetPitch", pitch);

      var area = result.getArea();
      SmartDashboard.putNumber("targetArea", area);

      var skew = result.getSkew();
      SmartDashboard.putNumber("targetSkew", skew);

      // double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = result.getBestCameraToTarget();
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    }  
    else {

    }       
  } // end periodic

  // method to get the best camera x distance
  public double getX() {
    return this.target.getBestCameraToTarget().getX();
  }

  // method to get the best camera Y distance
  public double getY() {
    return this.target.getBestCameraToTarget().getY();
  }

}

