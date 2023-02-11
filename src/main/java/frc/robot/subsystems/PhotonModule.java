// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonModule extends SubsystemBase {
  /** Creates a new PhotonModule. */

  private static PhotonCamera camera;
  PhotonTrackedTarget target;
  private double lx, ly;

  public PhotonModule() {
    // Change this to match the name of your camera
    camera = new PhotonCamera("OV5647");

    // SmartDashboard.putNumber("getX", target.getBestCameraToTarget().getX());
    // SmartDashboard.putNumber("targetID", target.getFiducialId());
    // SmartDashboard.putNumber("targetYaw", target.getYaw());
    // SmartDashboard.putNumber("targetPitch", target.getPitch());
    // SmartDashboard.putNumber("targetArea", target.getArea());

    // var skew = target.getSkew();
    // SmartDashboard.putNumber("targetSkew", skew);
    // SmartDashboard.putNumber("targetID", target.getFiducialId());
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    // PhotonTrackedTarget target = camera.getLatestResult();
    // boolean hasTargets = target.hasTargets();
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOn);
    
    PhotonPipelineResult results = camera.getLatestResult();
    SmartDashboard.putNumber("pipeline_index", camera.getPipelineIndex());

    // Check if the latest result has any targets
    if (camera.getLatestResult().hasTargets()) {
      target = camera.getLatestResult().getBestTarget();

      // Get a list of currently tracked targets
      // List<PhotonTrackedTarget> targetList = target.getTargets();
      var best = target.getBestCameraToTarget();
      // Get the current best target
      // var result = target.getBestTarget();

      // Get information from target
      SmartDashboard.putNumber("getX", best.getX());

    } else {

    }
  } // end periodic

  // method to get the best camera x distance
  public double getX() {
    if (camera.getLatestResult().hasTargets()) {
      target = camera.getLatestResult().getBestTarget();
       lx =target.getBestCameraToTarget().getX();
    } else {
       lx = 0;
    }
    return lx;
  
  }

  // method to get the best camera Y distance
  public double getY() {
    if (camera.getLatestResult().hasTargets()) {
      target = camera.getLatestResult().getBestTarget();
       ly =target.getBestCameraToTarget().getY();
    } else {
       ly = 0;
    }
    return lx;
  }

}

