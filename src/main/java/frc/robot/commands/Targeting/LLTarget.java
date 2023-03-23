package frc.robot.commands.Targeting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


public class LLTarget extends CommandBase {
     //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kX = 0.017;//ADJUST!!!  0.005??
  private double kY = 0.03; //0.00725;
  private Drive drive12;
  private double h1 = 32.5; //inches, distance from floor to center of camera lens
  //private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(20); //20 degrees - camera angle
  private double dist12; //desired distance from camera to target - pass into command
  private double steeringAdjust;
  private double cameraXoffset; 
  //private Limelight limelight;
  private double pipeline12;
  private double targetHeight12;//18" for Atag, from floor to center of target
  private double a2, dx, errorY, distanceAdjust;
  /** Creates a new LLTarget. */
  public LLTarget(Drive t_drive, double t_pipeline, double t_standoff, double t_targetHeight) {
    this.drive12 = t_drive;
    this.pipeline12 = t_pipeline;
    this.dist12 = t_standoff;
    this.targetHeight12 = t_targetHeight;
    //this.limelight = passed_limelight;
    addRequirements(this.drive12);
   // limelight.setPipeline(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("LLTarget init", pipeline12);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline12);
    cameraXoffset = 4; 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   // TO make sure dx is positive, use abs value for disY and (h1-h2)
    double disY= Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    double disX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double errorX = disX - cameraXoffset; 
    
    if(tv==1){
      if(Math.abs(errorX)>0.5){
        steeringAdjust = (kX * errorX); 
        }
      else {
        steeringAdjust = 0;  
      }
         a2 = disY*Math.PI/180;  //make sure disY is positive
         dx = Math.abs((h1-targetHeight12)) / Math.tan(a1+a2);
         errorY = dist12 - dx;
         distanceAdjust = kY * errorY; 
         
       drive12.setLeftSpeed(distanceAdjust + steeringAdjust);
       drive12.setRightSpeed(distanceAdjust - steeringAdjust); 
       
      SmartDashboard.putNumber("ErrorX - Angle Error tX", errorX);
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
    drive12.stop();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
/*if(tv==1 && Math.abs(errorY)<=2 && Math.abs(errorX <= 5)){
      SmartDashboard.putBoolean("LLDistance isFinished:", true);
      return true;
      }   
      else if(tv==1 && (Math.abs(errorY) > 2 || Math.abs(errorX > 5) {
         SmartDashboard.putBoolean("LLDistance still working angle or distance", true);
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
