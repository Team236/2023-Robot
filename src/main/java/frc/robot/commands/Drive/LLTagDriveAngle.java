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
  private boolean target;
  private PIDController anglePidController;

  public LLTagDriveAngle(Drive _drive, int _pipeline) {
      this.drive = _drive;
      addRequirements(drive);
      LimelightHelpers.setPipelineIndex("", _pipeline);
      LimelightHelpers.setLEDMode_ForceOff("");

      anglePidController = new PIDController(kPX, kIX, kDX );
      anglePidController.setSetpoint(0);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // verify the camera  sees a tag otherwise do nothing   
    target = LimelightHelpers.getTV("");
    SmartDashboard.putBoolean("Has Target", target);

    // get the published X value in limelight tables to use in PID loop to steer with 
    double[] currentPose = LimelightHelpers.getBotPose_TargetSpace("");
    double distanceX = currentPose[0]; 
    
    // test if target is visible by camera
    if( target ){
        double steeringAdjust = anglePidController.calculate(distanceX);
        drive.setLeftSpeed(steeringAdjust);
        drive.setRightSpeed(-steeringAdjust); 
        SmartDashboard.putNumber ("found distance tag", LimelightHelpers.getFiducialID("") );
        } 
        else { }
    }  // enf-if target

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
