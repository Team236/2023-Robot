// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class LLTagAngle extends CommandBase {
  private double kPX = 0.02;  //0.005??
  private double kIX = 0.0;  //0.005??
  private double kDX = 0.002;  //0.005??
  private Drive drive;
  private int pipeline;
  private Limelight camera;
  PIDController anglePidController;


  public LLTagAngle(Drive _drive, double _pipeline,Limelight _camera) {
    //public LLAngle(Drive passed_drive, Limelight lime, double m_pipeline) {
      this.drive = _drive;
      this.pipeline = (int) _pipeline;
      this.camera = _camera;
     // this.limelight = lime;
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn off the LEDs
    camera.setLedModeOff();
   
    // set the pipeline to 7 
    camera.setPipeline(pipeline);

     anglePidController = new PIDController(kPX, kIX, kDX );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set the pipeline to 7 
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(7);
    
    // determine if camera has a target  
    boolean target = camera.HasTarget();
    
    // get the published X value in limelight tables to use in PID loop to steer with 
    // if the camera to robot is not set then it should align with camera not robot
    double distanceX = camera.getRobotToTargetPoseX(); 

    // test if target is visible by camera
    if(target){
        double steeringAdjust = anglePidController.calculate(distanceX);
        drive.setLeftSpeed(steeringAdjust);
        drive.setRightSpeed(-steeringAdjust); 
        } 
        else {
      SmartDashboard.putBoolean("Has Target", target);
      }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    camera.setLedModeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
