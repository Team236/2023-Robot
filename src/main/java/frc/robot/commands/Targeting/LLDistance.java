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
  private Drive drive;
  private double h1 = 35; //inches, same unit as d
  private double h2 = 50; // inches, same unit as d
  private double a1 = 0.8340573450259819; //47.78 degrees or 17.98//taken from last year - only true if LL is still at 40 degrees
  private double d = 14; // 12 inches away? idk how far we want to be
  private Limelight limelight;
  
  /** Creates a new LLAngle. */
  public LLDistance(Drive passed_drive, Limelight lime) {
    this.drive = passed_drive;
    this.limelight = lime;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = limelight.getTv();
    double disY= limelight.getTy();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if(tv==1){
         double a2 = disY*Math.PI/180;
         double dx = (h2-h1) / Math.tan(a1+a2);
         double errorY = d - dx;
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
