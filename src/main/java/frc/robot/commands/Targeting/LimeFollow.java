// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class LimeFollow extends CommandBase {
   //*Causes the robot to drive to a set distance ("driveDistance") from a targeted AprilTag
   private Drive drive;
   private double driveDistance;  //inches - desired x distance from PV camera to AprilTag center
   private double yOffset; // inches - desired y distance from PV camera to AprilTag center
   private double kPdistance = 0.006, kIdistance = 0, kDdistance = 0;  //kpdistance 0.008 teleop?
   private double kPangle = 0.003, kIangle = 0, kDangle = 0; //kpangle 0.004 teleop?
   private final PIDController distanceController, turnController;
   private PhotonPipelineResult result;
   private Limelight camera;
   private double xPos, yPos, zPos, skew;

  public LimeFollow(Drive m_drive, Limelight m_camera) {
    this.camera = m_camera;
    this.drive = m_drive;
    this.driveDistance = Units.inchesToMeters(40);  // hardcode distance offset
    this.yOffset = 0;  // angle offset
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
    turnController.setSetpoint(yOffset); //this is the setpoint for the Y, may need an offset rather than 0
    camera.setPipeline(1); 
    // camera.setDriverMode(false);
    // camera.setLED(VisionLEDMode.kOff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("has TArget", camera.getTv() );
    // camera.setLED(VisionLEDMode.kOff);
    double[] result = camera.getCameraToTargetPose();

    if (camera.getTv()) {
         //Meters to inches (0.0254)
     xPos = result[0] ;   // use the x direction from the pose
     SmartDashboard.putNumber("xPos", result[0]);

     yPos = result[1];    // use the y direction from the pose
     SmartDashboard.putNumber("yPos", result[1]);
     // zPos = result.getBestTarget().getBestCameraToTarget().getZ() / 0.0254;
     //skew = result.getBestTarget().getSkew() / 0.0254;
    
     
     double dC = distanceController.calculate(xPos);
     double tC = turnController.calculate(yPos);

    double LS = dC + tC ; // - tc for 2022 robot
    double RS = dC - tC;  // + tc for 2022 robot
    
//double max = Math.abs(LS);
 //  if (max < Math.abs(RS)) {max = Math.abs(RS);}
 //   if (max > 1) {LS /= max; RS/= max;}

     double leftSpeed = LS;
     //double rightSpeed = leftSpeed;
     double rightSpeed = RS;
    // SmartDashboard.putNumber("left input",leftSpeed); // (-) For testing; Camera on back of Robot
    // SmartDashboard.putNumber("left input",right Speed);
     
     drive.setLeftSpeed(leftSpeed/2);
     drive.setRightSpeed(rightSpeed/2);
      
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