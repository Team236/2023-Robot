// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive;

public class LLTagDriveAngle extends CommandBase {
  private double kPX = 0.0002;  //0.005??
  private double kIX = 0.0;  //0.005??
  private double kDX = 0.0;  //0.005??

  private Drive drive;
  private PIDController anglePidController;


  public LLTagDriveAngle(Drive _drive, int _pipeline) {
    //public LLAngle(Drive passed_drive, Limelight lime, double m_pipeline) {
      this.drive = _drive;
      LimelightHelpers.setPipelineIndex("", _pipeline);
      LimelightHelpers.setLEDMode_ForceOff("");
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       anglePidController = new PIDController(kPX, kIX, kDX );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // determine if camera has a target  
    boolean target = LimelightHelpers.getTV("");
    
    // get the published X value in limelight tables to use in PID loop to steer with 
    // if the camera to robot is not set then it should align with camera not robot
    double[] currentPose = LimelightHelpers.getBotPose_TargetSpace("");
    double distanceX = currentPose[0]; 
    SmartDashboard.putBoolean("Has Target", target);

    // test if target is visible by camera
    if( target ){
        double steeringAdjust = anglePidController.calculate(distanceX);
        drive.setLeftSpeed(steeringAdjust);
        drive.setRightSpeed(-steeringAdjust); 
        } 
        else { }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    LimelightHelpers.setLEDMode_ForceOff("");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
