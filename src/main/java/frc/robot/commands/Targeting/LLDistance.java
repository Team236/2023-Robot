// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class LLDistance extends CommandBase {
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kY = 0.02; //0.00725;
  
  private double h1 = 32.5; //inches, from ground to center of camera lens
  //private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(21); //20 degrees, camera tilt
  private double dist11; // desired distance from camera to target in inches; pass into command
  //private Limelight limelight;
  private Drive drive11;
  private double pipeline11;
  private double targetHeight11;//18" for Atag, from floor to center of target
  private double tv, disY, a2, dx, errorY;
  
  /** Creates a new LLAngle. */
  public LLDistance(Drive d_drive, double d_pipeline, double d_standoff, double d_targetHeight) {
  //public LLDistance(Drive passed_drive, Limelight lime, double m_pipeline) {
    this.drive11 = d_drive;
    this.pipeline11 = d_pipeline;
    this.dist11 = d_standoff;
    this.targetHeight11 = d_targetHeight;
   // this.limelight = lime;
    addRequirements(drive11);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("LLDistance init", pipeline11);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline11);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
   tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  //double tv = limelight.getTv();
 
    // TO make sure dx is positive, use abs value for disY and (h1-h2)
   disY = Math.abs (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  // double disY= Math.abs(limelight.getTy());  

    if(tv==1){
        a2 = disY*Math.PI/180; // in radians, if disY in degrees
        dx = Math.abs(targetHeight11 - h1) / Math.tan(a1+a2);  
        errorY = dist11 - dx;  
    //NOTE:  CAN TRY TO USE THE Z VALUE OF THE POSE FOR errorY (use [2] or [0] for other directions)
    // double errorY = NetworkTableInstance.getDefault().getTable("limelight").
    // getIntegerTopic("targetpose_cameraspace").subscribe(new double[]{}).get()[3];
      double distanceAdjust = kY * errorY;
       drive11.setLeftSpeed(distanceAdjust);
       drive11.setRightSpeed(distanceAdjust);
      SmartDashboard.putNumber("dx, Y dist from target:", dx);
      SmartDashboard.putNumber("ErrorY:", errorY);
      SmartDashboard.putNumber("Ty, degrees:", disY);
   } else{
      SmartDashboard.putNumber("No Target", tv);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive11.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if(tv==1 && Math.abs(errorY)<=1){
      SmartDashboard.putBoolean("LLDistance isFinished:", true);
      return true;
      }   
      else if(tv==1 && Math.abs(errorY)>1){
        return false;
      }
      else
      {
      SmartDashboard.putNumber("No Shoot Target", tv);
      return true;
      }
      
}
  }