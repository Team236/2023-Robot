package frc.robot.commands.Targeting;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class LLTarget extends CommandBase {
     //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kX = 0.02;//ADJUST!!!  0.005??
  private double kY = 0.00785; //0.00725;
  private Drive drive;
  private double h1 = 33; //inches, distance from floor to center of camera lens
  private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(28); //20 degrees - camera angle
  private double d; //desired distance from camera to target - pass into command
  private double steeringAdjust;
  //private Limelight limelight;
  private double pipeline3;
  
  /** Creates a new LLTarget. */
  public LLTarget(Drive m_drive, double m_pipeline3, double m_standoff) {
    this.drive = m_drive;
    this.pipeline3 = m_pipeline3;
    this.d = m_standoff;
    //this.limelight = passed_limelight;
    addRequirements(this.drive);
   // limelight.setPipeline(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline3);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   // TO make sure dx is positive, use abs value for disY and (h1-h2)
    double disY= Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    double disX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double errorX = disX - Units.inchesToMeters(-4.25); //camera offset is 4, negative works?
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if(tv==1){
      if(Math.abs(errorX)>2){
        steeringAdjust = (kX * errorX); 
        }
      else {
        steeringAdjust = 0;  
      }
         double a2 = disY*Math.PI/180;  //make sure disY is positive
         double dx = Math.abs((h1-h2)) / Math.tan(a1+a2);
         double errorY = d - dx;
         double distanceAdjust = kY * errorY; 
         
       drive.setLeftSpeed(distanceAdjust + steeringAdjust);
       drive.setRightSpeed(distanceAdjust - steeringAdjust);
      SmartDashboard.putNumber("dx, distance from target", dx);  
      SmartDashboard.putNumber("ErrorY - Distance Error", errorY);
      SmartDashboard.putNumber("ErrorX - Angle Error", errorX);
   } else{
   SmartDashboard.putNumber("No Target", tv);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
