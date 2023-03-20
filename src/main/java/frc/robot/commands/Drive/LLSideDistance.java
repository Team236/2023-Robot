// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;

public class LLSideDistance extends CommandBase {
  private double kX = 0.03;  //0.005??
  private double kY = 0.00;  //0.000??
  private double kZ = 0.00;  //0.000??
  private boolean target;
  private double distX, errorX;
  private Drive drive;
  private Turret turret;
  private double cameraXoffset, driveAdjust;
  private PIDController driveController;

  /** Creates a new LLSideDistace. */
  public LLSideDistance(Drive _drive, int _pipeline,Turret _turret, double _offset) {
      this.drive = _drive;
      this.turret= _turret;
      cameraXoffset = _offset; //need to figure out - camera x offset as angle in degrees
      LimelightHelpers.setLEDMode_PipelineControl("");
      LimelightHelpers.setPipelineIndex("",_pipeline); // should be reflective tape
    
      addRequirements(drive);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize()  {  
      driveController = new PIDController(kX, kY, kZ );
      driveController.setSetpoint(cameraXoffset);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      target = LimelightHelpers.getTV("");
      distX = LimelightHelpers.getTX("");
       
      if(target) {
        SmartDashboard.putNumber("Adjust Position Sideways, ErrorX is:", errorX);
        driveAdjust = driveController.calculate(distX);

        if (turret.getTurretAngle()> 55 && turret.getTurretAngle() < 125) { 
          
          // TODO test if Turret at 90, want to drive revers for positive error
          drive.setBothSpeeds(-driveAdjust);

        } else if (turret.getTurretAngle()>=180 && turret.getTurretAngle()<270) {
          
          // TODO test if Turret at 270, want to drive fwd for positive error
          drive.setBothSpeeds(driveAdjust);
          
        } else {  }  // not pointing sideways do nothing
      }  
      else {      // else no target
          SmartDashboard.putNumber("No Shoot Target", 999 );
      }  // end-if_target 
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    LimelightHelpers.setLEDMode_ForceOff("");
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
 