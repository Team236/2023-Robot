// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LLDistance extends CommandBase {
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kY = 0.00785; //0.00725;
  
  private double h1 = 33; //inches, from ground to center of camera lens
  private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(28); //20 degrees, camera tilt
  private double d; // desired distance from camera to target; pass into command
  //private Limelight limelight;
  private Drive drive;
  private double pipeline2;
  
  /** Creates a new LLAngle. */
  public LLDistance(Drive m_drive, double m_pipeline, double m_standoff) {
  //public LLDistance(Drive passed_drive, Limelight lime, double m_pipeline) {
    this.drive = m_drive;
    this.pipeline2 = m_pipeline;
    this.d = m_standoff;
   // this.limelight = lime;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
   double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  //double tv = limelight.getTv();
 
    // TO make sure dx is positive, use abs value for disY and (h1-h2)
   double disY = Math.abs (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  // double disY= Math.abs(limelight.getTy());  
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if(tv==1){
        double a2 = disY*Math.PI/180;
        double dx = Math.abs(h2 - h1) / Math.tan(a1+a2);  
        double errorY = d - dx;  
    //NOTE:  CAN TRY TO USE THE Z VALUE OF THE POSE FOR errorY (use [2] or [0] for other directions)
    // double errorY = NetworkTableInstance.getDefault().getTable("limelight").
    // getIntegerTopic("targetpose_cameraspace").subscribe(new double[]{}).get()[3];
      double distanceAdjust = kY * errorY;
       drive.setLeftSpeed(distanceAdjust);
       drive.setRightSpeed(distanceAdjust);
      SmartDashboard.putNumber("dx", dx);
   } else{
      SmartDashboard.putNumber("No Target", tv);
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