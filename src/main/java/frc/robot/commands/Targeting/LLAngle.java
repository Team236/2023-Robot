// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LLAngle extends CommandBase {
  private double kX = 0.03;  //0.005??
  private double tv, distX, errorX;
  private Drive drive;
  private double pipeline, cameraXoffset;
  private Limelight limelight;

  public LLAngle(Drive _drive, double _pipeline) {
    //public LLAngle(Drive _drive, Limelight _lime, double _pipeline) {
      this.drive = _drive;
      this.pipeline = _pipeline;
      // this.limelight = _lime;
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("LLangle init", pipeline);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 0; //need to figure out

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    distX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    errorX = distX - cameraXoffset;////
//TRY FOR errorX
     // double errorY = NetworkTableInstance.getDefault().getTable("limelight").
       // getIntegerTopic("targetpose_cameraspace").subscribe(new double[]{}).get()[2];  //or 0?
    if(tv==1) {
      if (Math.abs(errorX)>2){
      //double x = (errorX-160)/320;
      //Establishes a minimum error in the x axis 
      SmartDashboard.putNumber("Adjust Angle, ErrorX is:", errorX);
        double steeringAdjust = kX * errorX;
        drive.setLeftSpeed(steeringAdjust);
        drive.setRightSpeed(-steeringAdjust); 
        }   
      else{
      SmartDashboard.putNumber("No Shoot Target", tv);
      }
     }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return false;
/* 
    if(tv==1 && Math.abs(errorX)<=2){
      SmartDashboard.putBoolean("LLAngle isFinished:", true);
      return true;
      }   
      else if(tv==1 && Math.abs(errorX)>2){
        return false;
      }
      else
      {
      SmartDashboard.putNumber("No Shoot Target", tv);
      return true;
      }
  */
}

}
