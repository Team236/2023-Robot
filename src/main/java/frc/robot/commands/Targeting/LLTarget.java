// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class LLTarget extends CommandBase {
  /** Creates a new LLTarget. */
  private double kX = 0.005;
  private double kY = 0.00785; //0.00725;
  private Drive drive3;
  private double h1 = 21.5; //inches, same unit as d  9.5 for testbot; 21.5 for 2023 bot 
  private double h2 = 18; // inches, same unit as d
  private double a1 = 0.436; //25 degrees - adjusgt for 2023 bot
  private double d = 40; // 12 inches away? idk how far we want to be
  private double steeringAdjust;
  //private Limelight limelight;
  
  /** Creates a new LLAngle. */
  public LLTarget(Drive passed_drive3) {
    this.drive3 = passed_drive3;
    //this.limelight = passed_limelight;
    addRequirements(this.drive3);
   // limelight.setPipeline(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double disY= NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double disX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double errorX = disX - Units.inchesToMeters(7); //camera offset is 7
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if(tv==1){
      if(Math.abs(errorX)>2){
        steeringAdjust = (kX * errorX); 
        }
      else {
        steeringAdjust = 0;  
      }
         double a2 = disY*Math.PI/180;
         double dx = -(h2-h1) / Math.tan(a1+a2);
         double errorY = d - dx;
         double distanceAdjust = kY * errorY;
         //adding offset for camera; not centered
       drive3.setLeftSpeed(distanceAdjust + steeringAdjust);
       drive3.setRightSpeed(distanceAdjust - steeringAdjust);
      SmartDashboard.putNumber("dx", dx);  
     SmartDashboard.putNumber("ErrorX", errorX);


   } else{
   SmartDashboard.putNumber("No Target", tv);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive3.stop();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
