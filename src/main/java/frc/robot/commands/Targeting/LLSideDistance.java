// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Targeting;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;

import java.lang.Math;

public class LLSideDistance extends CommandBase {
  private double kX = 0.017;  //0.005??
  private double tv, distX, errorX;
  private Drive drive13;
  private Turret turret13;
  private double pipeline13, cameraXoffset;

  /** Creates a new LLSideDistace. */
  public LLSideDistance(Drive s_drive, Turret s_turret, double s_pipeline) {
    //public LLAngle(Drive passed_drive, Limelight lime, double m_pipeline) {
      this.drive13 = s_drive;
      this.pipeline13 = s_pipeline;
      this.turret13 = s_turret;
     // this.limelight = lime;
      addRequirements(drive13);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize()  {  
      SmartDashboard.putNumber("LLangle init", pipeline13);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline13);
      cameraXoffset = 4; //need to figure out- camera x offset as angle in degrees
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
        if (Math.abs(errorX)>0.5){
        //double x = (errorX-160)/320;
        //Establishes a minimum error in the x axis 
        SmartDashboard.putNumber("Adjust Position Sideways, ErrorX is:", errorX);
          double steeringAdjust = kX * errorX;
          //if Turret at 270, want to drive fwd for positive error
          
          if (turret13.getTurretAngle() <= 0) {
          drive13.setBothSpeeds(steeringAdjust);
          } else  {
          drive13.setBothSpeeds(-steeringAdjust);
          }
          //if Turret at 90, want to drive revers for positive error - ADD LOGIC
          //drive13.setBothSpeeds(-sterringAdjust);
        }
        else{
        SmartDashboard.putNumber("No Shoot Target", tv);
        }
       }
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive13.stop();
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
 