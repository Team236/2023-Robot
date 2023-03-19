// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AprilFollow extends CommandBase {
   //*Causes the robot to drive to a set distance ("driveDistance") from a targeted AprilTag
   private Drive drive;
   private double driveDistance;  //inches - desired x distance from PV camera to AprilTag center
   private double yOffset; // inches - desired y distance from PV camera to AprilTag center
   private double kPdistance = 0.006, kIdistance = 0, kDdistance = 0;  //kpdistance 0.008 teleop?
   private double kPangle = 0.003, kIangle = 0, kDangle = 0; //kpangle 0.004 teleop?
   private final PIDController distanceController;// turnController;
   private PhotonPipelineResult result;
   private PhotonCamera camera;
   private double xPos, yPos, zPos, skew;
   private double kPgyro, angleFix, dC, LS, RS;


   public AprilFollow(Drive _drive, PhotonCamera _camera, double _driveDistance, double _yOffset) {
    this.drive = _drive;
    this.camera = _camera;
    this.driveDistance = _driveDistance;
    this.yOffset = _yOffset;
    addRequirements(drive);

    distanceController = new PIDController(kPdistance, kIdistance, kDdistance);
   // turnController = new PIDController(kPangle, kIangle, kDangle);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    distanceController.reset();
    //turnController.reset();
    // goal is to maintain a set distance (called driveDistance) between the Apriltag and the camera
    distanceController.setSetpoint(driveDistance);
    //turnController.setSetpoint(yOffset); //this is the setpoint for the Y, may need an offset rather than 0
    //camera.setDriverMode(false);
    //camera.setPipelineIndex(0);
    //camera.setLED(VisionLEDMode.kOff);
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

     yPos = ((result.getBestTarget().getBestCameraToTarget().getY() / 0.0254));// + yOffset);
     angleFix = yPos*kPgyro;
     // zPos = result.getBestTarget().getBestCameraToTarget().getZ() / 0.0254;
     //skew = result.getBestTarget().getSkew() / 0.0254;
      SmartDashboard.putBoolean("hasTarget", result.hasTargets());
      SmartDashboard.putNumber("xPos", xPos);
      SmartDashboard.putNumber("yPos", yPos);
      SmartDashboard.putNumber("angleFix", angleFix);
     // SmartDashboard.putNumber("zPos", zPos);
     // SmartDashboard.putNumber("Skew", skew);
     
     dC = distanceController.calculate(xPos);
     //double tC = turnController.calculate(yPos);

     //reverse second term for 2022 robot?
     if ((dC <= 0) && (angleFix <=0)) {
      LS = dC - angleFix;
      RS = dC;//- angleFix;
     }
      else if ((dC <= 0) && (angleFix > 0)) {
        LS = dC; //+ angleFix;
        RS = dC + angleFix;
    }
    else if ((dC > 0) && (angleFix <= 0)) {
      LS = dC; //+ angleFix;
      RS = dC+ angleFix;
  }
    else if ((dC > 0) && (angleFix > 0)) {
      LS = dC;
      RS = dC - angleFix;
}
double LS = dC + angleFix; // - second term for 2022 robot
double RS = dC - angleFix;  // + second term for 2022 robot
    
double max = Math.abs(LS);
 if (max < Math.abs(RS)) {max = Math.abs(RS);}
 if (max > 1) {LS /= max; RS/= max;}

     double leftSpeed = LS;
     //double rightSpeed = leftSpeed;
     double rightSpeed = RS;
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