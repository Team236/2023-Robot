// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class AutoPIDDrive extends CommandBase {
  private Drive drive;
  private double driveDistance, gyrokP, error, LL, RR, adjustL, adjustR;
  private final PIDController leftPidController, rightPidController;

  /** Creates a new DriveStraight. */
  public AutoPIDDrive(Drive drive, double driveDistance) {
    this.drive = drive;
    this.driveDistance = driveDistance;
    this.leftPidController = new PIDController(DriveConstants.leftkPdrive, DriveConstants.leftkIdrive, DriveConstants.leftkDdrive);
    this.rightPidController = new PIDController(DriveConstants.rightkPdrive, DriveConstants.rightkIdrive, DriveConstants.rightkDdrive);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    leftPidController.setSetpoint(driveDistance);
    rightPidController.setSetpoint(driveDistance);
    //getRate returns rate of change of the gyro angle, hence it is "spin"
   error = 0; //drive.navX.getRate();
    gyrokP = 0;//0.03;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPidController.reset();
    rightPidController.reset();

    drive.resetLeftEncoder();
    drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LL = leftPidController.calculate(drive.getLeftDistance());
    RR = rightPidController.calculate(drive.getRightDistance());

    /*
    if (RR <= 0) {
      adjustL = kPgyro*error;
      adjustR = -kPgyro*error;
   } else {
      adjustL = -kPgyro*error;
      adjustR = kPgyro*error;
   }
   */
  adjustL = 0;
  adjustR = 0;

      drive.setLeftSpeed(LL + adjustL);
      drive.setRightSpeed(RR + adjustR);  
      
     // SmartDashboard.putNumber("error is", error);
     // SmartDashboard.putNumber("LL is", LL);
     // SmartDashboard.putNumber("RR is", RR);
     // SmartDashboard.putNumber("adjsut L is", adjustL);
     // SmartDashboard.putNumber("adjust R is", adjustR);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //stop when near target and commanded speed close to 0
  if ((drive.getLeftDistance() > 0.9*driveDistance)&& (Math.abs(LL + adjustL) < 0.02)) {
    SmartDashboard.putBoolean("AutoPIDDRive Finished?", true);
    return true;
  } else {
    SmartDashboard.putBoolean("AUTOPIDDRive Finished?", false);
    return false;
  }
  }
}
