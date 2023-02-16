// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AprilTags extends CommandBase {
   //*Causes the robot to drive to a set distance ("driveDistance") from a targeted AprilTag
   private Drive drive;
   private double driveDistance;  //inches
   private double kPdistance = 0.005, kIdistance = 0, kDdistance = 0; 
   private double kPangle = 0.002, kIangle = 0, kDangle = 0; 
   private final PIDController distanceController, turnController;
   private PhotonPipelineResult result;
   private PhotonCamera camera;
   private double xPos, yPos, zPos, skew;

  public AprilTags(Drive m_drive, PhotonCamera m_camera, double m_driveDistance) {
    this.camera = m_camera;
    this.drive = m_drive;
    this.driveDistance = m_driveDistance;
    addRequirements(drive);

    distanceController = new PIDController(kPdistance, kIdistance, kDdistance);
    turnController = new PIDController(kPangle, kIangle, kDangle);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    distanceController.reset();
    turnController.reset();
    // goal is to maintain a set distance (called driveDistance) between the Apriltag and the camera
    distanceController.setSetpoint(driveDistance);
    turnController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOff);
    result = camera.getLatestResult();


    if (result.hasTargets()){
         //Meters to inches (0.0254)
     xPos = result.getBestTarget().getBestCameraToTarget().getX() / 0.0254;
     yPos = result.getBestTarget().getBestCameraToTarget().getY() / 0.0254;
     // zPos = result.getBestTarget().getBestCameraToTarget().getZ() / 0.0254;
     //skew = result.getBestTarget().getSkew() / 0.0254;
      SmartDashboard.putBoolean("hasTarget", result.hasTargets());
      SmartDashboard.putNumber("xPos", xPos);
      SmartDashboard.putNumber("yPos", yPos);
     // SmartDashboard.putNumber("zPos", zPos);
     // SmartDashboard.putNumber("Skew", skew);
     
     double dC = distanceController.calculate(xPos);
     double tC = turnController.calculate(yPos);
    

     double leftSpeed = dC + tC;
     //double rightSpeed = leftSpeed;
     double rightSpeed = dC - tC;
    // SmartDashboard.putNumber("left input",leftSpeed); // (-) For testing; Camera on back of Robot
    // SmartDashboard.putNumber("left input",right Speed);
     
     drive.setLeftSpeed(leftSpeed);
     drive.setRightSpeed(rightSpeed);
      
    }
    else{
      SmartDashboard.putNumber("xPos", 0);
      SmartDashboard.putNumber("yPos", 0);
    //  SmartDashboard.putNumber("zPos", 0);
    //  SmartDashboard.putNumber("Skew", 0);
  } 

 
    
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
