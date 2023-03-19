// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class LLTagDriveDistance extends CommandBase {
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kP = 0.005;  //0.005;
  private double kI = 0.0;      //0.0;
  private double kD = 0.000;  //0.0009;
  private double tv;
  
  //private Limelight limelight;
  private Drive drive;
  private double pipeline;
  private Limelight camera;
  private boolean target;
  PIDController distanceTagController;


  /** Creates a new LLAngle. */
  public LLTagDriveDistance(Drive _drive, double _pipeline, double OffsetDistance, Limelight _camera) {
    this.drive = _drive;
    this.pipeline = _pipeline;
    this.camera = _camera;
    distanceTagController = new PIDController(kP, kI, kD);
    distanceTagController.setSetpoint(OffsetDistance);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setDriverModeOff();
    camera.setLedModeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // verify the camera  sees a tag other wise do nothing 
   target = camera.HasTarget();
   tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    
    if(target || tv == 1 ) {
        
        // TO make sure dx is positive, use abs value for disY and (h1-h2)
        double absoluteDistanceZ = Math.abs (camera.getCameraToTargetPoseZ());

        // the Z direction is out the front of robot 
        double distanceAdjust = distanceTagController.calculate(absoluteDistanceZ);
        drive.setLeftSpeed(distanceAdjust);
        drive.setRightSpeed(distanceAdjust);
        SmartDashboard.putNumber ("found distance tag", camera.getTagData() );
    } 
   else {
    SmartDashboard.putNumber ("found distance tag", 999 );
    SmartDashboard.putBoolean("No Target", target);
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