// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;
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
  private double kX = 0.002;
  private Drive drive2;
  /** Creates a new LLAngle. */
  public LLAngle(Drive passed_drive2) {
    this.drive2 = passed_drive2;
    addRequirements(this.drive2);
    // Use addRequirements() here to declare subsystem dependencies.
    //limelight.setPipeline(0);
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
    double errorX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if(tv==1){
      //double x = (errorX-160)/320;
      //Establishes a minimum error in the x axis 
      if(Math.abs(errorX)>2){
        double steeringAdjust = kX * errorX;
        drive2.setLeftSpeed(steeringAdjust);
        drive2.setRightSpeed(-steeringAdjust); 
        }
       
      } else{
      SmartDashboard.putNumber("No Shoot Target", tv);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive2.stop();
   NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
   //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
