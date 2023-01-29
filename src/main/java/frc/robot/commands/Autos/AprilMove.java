// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PhotonModule;

public class AprilMove extends CommandBase {
  private Drive drive;
  private double driveDistance;
  private final PIDController leftController, rightController;
  private PhotonModule photonModule;

  public AprilMove(Drive drive, double driveDistance, PhotonModule photonModule) {
    this.drive = drive;
    this.photonModule = photonModule;
    leftController = new PIDController(DriveConstants.leftkPdrive, DriveConstants.leftkIdrive, DriveConstants.leftkDdrive);
    rightController = new PIDController(DriveConstants.rightkPdrive, DriveConstants.rightkIdrive, DriveConstants.rightkDdrive);  }
    @Override

  public void initialize() {
  leftController.reset();;
  rightController.reset();

  drive.resetLeftEncoder();
  drive.resetRightEncoder();
  }
  @Override
  
  public void execute() {
    double leftSpeed = leftController.calculate(drive.getLeftDistance());
    double rightSpeed = rightController.calculate(drive.getRightDistance());
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(rightSpeed);
  }
   
    @Override

  public void end(boolean interrupted) {
  drive.stop(0);
  }
  
  @Override

  public boolean isFinished() {
    return false;
  }
}
