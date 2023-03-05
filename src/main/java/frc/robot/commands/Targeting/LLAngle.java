// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LLAngle extends CommandBase {
  private double kX = 0.02;  //0.005??
  private Drive drive;
  private double pipeline1;


  public LLAngle(Drive m_drive, double m_pipeline) {
    //public LLAngle(Drive passed_drive, Limelight lime, double m_pipeline) {
      this.drive = m_drive;
      this.pipeline1 = m_pipeline;
     // this.limelight = lime;
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double disX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double errorX = disX - Units.inchesToMeters(-4); //camera offset is approx 4 inches? Negative works?
 
//TRY FOR errorX
     // double errorY = NetworkTableInstance.getDefault().getTable("limelight").
       // getIntegerTopic("targetpose_cameraspace").subscribe(new double[]{}).get()[2];  //or 0?


    if(tv==1){
      //double x = (errorX-160)/320;
      //Establishes a minimum error in the x axis 
      if(Math.abs(errorX)>2){
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
  }

}
