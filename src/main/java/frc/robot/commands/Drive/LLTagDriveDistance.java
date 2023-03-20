// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive;

public class LLTagDriveDistance extends CommandBase {

  private double kP = 0.005;  //0.005;
  private double kI = 0.0;      //0.0;
  private double kD = 0.000;  //0.0009;
  private Drive drive;
  private boolean target;
  PIDController distanceTagController;


  /** Creates a new LLAngle. */
  public LLTagDriveDistance(Drive _drive, int _pipeline, double OffsetDistance) {
    this.drive = _drive;
    LimelightHelpers.setPipelineIndex("", _pipeline);

    distanceTagController = new PIDController(kP, kI, kD);
    distanceTagController.setSetpoint(OffsetDistance);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // for all LimelightHelpers the empty string value is assumed to be "limelight" if blank
    LimelightHelpers.setStreamMode_PiPMain("");
    LimelightHelpers.setLEDMode_PipelineControl("");  
    LimelightHelpers.setLEDMode_ForceOff("");         
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // verify the limelight  sees a tag otherwise do nothing 
   target = LimelightHelpers.getTV("");
    
    if(target) {  
        // TO make sure dx is positive, use abs value for disY and (h1-h2)
        SmartDashboard.putBoolean("No Target", target);
        double[] robotpose = LimelightHelpers.getBotPose_TargetSpace("");
        double absoluteDistanceZ = Math.abs(robotpose[2]);    // index two is z-positive out the front of robot

        // the Z direction is out the front of robot 
        double distanceAdjust = distanceTagController.calculate(absoluteDistanceZ);
        drive.setLeftSpeed(distanceAdjust);
        drive.setRightSpeed(distanceAdjust);
        SmartDashboard.putNumber ("TagDistance tag id: ", LimelightHelpers.getFiducialID("") );
    } 
   else {
    SmartDashboard.putNumber ("found distance tag", 999 );
    SmartDashboard.putBoolean("No Target", target);
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