// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
public class TDWG_No extends CommandBase {
  /** Creates a new TankDriveWithGyro. */
private double kP;
private double dist;
private double speed;
private Drive drive;

  public TDWG_No(Drive _drive, double _kP, double _dist, double _speed) {
    this.kP = _kP;
    this.dist = _dist;
    this.speed = _speed;
    this.drive = _drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   drive.navX.reset();
   drive.resetLeftEncoder();
   drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double L;
    double R; 
    double ang = drive.navX.getRate();

if (speed <= 0) {
  L = speed - kP*ang;
  R = speed + kP*ang;
} else {
  L = speed + kP*ang;
  R = speed - kP*ang;
}

drive.setLeftSpeed(L);
drive.setRightSpeed(R);
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
   /* boolean isLeftFinished = 
    Math.abs(drive.getLeftDistance() - dist) 
    < Constants.DriveConstants.MARGIN_GYRO_DRIVE;

    boolean isRightFinished = 
    Math.abs(drive.getRightDistance() - dist) 
    < Constants.DriveConstants.MARGIN_GYRO_DRIVE;

    return (isLeftFinished && isRightFinished);*/
  }
}
