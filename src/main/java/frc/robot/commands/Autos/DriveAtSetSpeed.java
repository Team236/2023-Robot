// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveAtSetSpeed extends CommandBase {

  private Drive drive;
  private double distance, speed;  

  /** Creates a new DriveToCS. */
  public DriveAtSetSpeed(Drive m_drive, double m_distance, double m_speed) {
    this.drive = m_drive;
    this.distance = m_distance;
    this.speed = m_speed;

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
    SmartDashboard.putNumber("Initial Roll in degrees", drive.getRoll());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setBothSpeeds(speed);
    SmartDashboard.putBoolean("isLevel", drive.isLevel());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drive.getAvgDistance() >= distance) {
      return true;
    } else {
      return false;
       }
}
}
