// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive;


public class LLTagDriveAngleDistance extends CommandBase {


  private double distancekP = 0.005;  //0.005;
  private double distancekI = 0.0;      //0.0;
  private double distancekD = 0.000;  //0.0009;

  private double anglekP = 0.005;  //0.005;
  private double anglekI = 0.0;      //0.0;
  private double anglekD = 0.000;  //0.0009;
 
 
  private Drive drive;
  private boolean target;
  PIDController distanceTagController, angleTagController;

  /** Creates a new LLAngle. */
  public LLTagDriveAngleDistance(Drive _drive, int _pipeline, double OffsetDistance) {
    this.drive = _drive;
    addRequirements(drive);

    LimelightHelpers.setPipelineIndex("",_pipeline);
    LimelightHelpers.setLEDMode_ForceOff("");

    distanceTagController = new PIDController(distancekP, distancekI, distancekD);
    angleTagController = new PIDController(anglekP, anglekI, anglekD);
    distanceTagController.setSetpoint(OffsetDistance);
    angleTagController.setSetpoint(0); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // verify the camera  sees a tag other wise do nothing 
   

   target = LimelightHelpers.getTV("");
   SmartDashboard.putBoolean("Has Target", target);

    if(target) {
        SmartDashboard.putNumber ("found distance tag", LimelightHelpers.getFiducialID("") );

        // TODO make sure dX right is positive, use abs value for distance in dZ 
        double[] currentPose = LimelightHelpers.getBotPose_TargetSpace("");

        double DistanceX = currentPose[0];    // the X direction is robot out right side
        double absoluteDistanceZ = Math.abs (currentPose[2]);   // the Z direction is out the front of robot
                 
        double angleAdjust = angleTagController.calculate(DistanceX);
        double distanceAdjust = distanceTagController.calculate(absoluteDistanceZ);
        
        // make sure stearing plus drive command not > 100 percent
        if (distanceAdjust+angleAdjust <= 1) {
          drive.setLeftSpeed(distanceAdjust-angleAdjust);
          drive.setRightSpeed(distanceAdjust+angleAdjust);
        } else {
        drive.setLeftSpeed(distanceAdjust-angleAdjust);
        drive.setRightSpeed(distanceAdjust);
        }
    } 
   else {
    SmartDashboard.putNumber ("found distance tag", 999 );
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}