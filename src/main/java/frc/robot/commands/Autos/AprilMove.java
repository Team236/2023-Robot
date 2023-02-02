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
  private double driveDistance = 0.5;  //meters
  private double kPdistance, kIdistance, kDdistance; 
  private double kPangle, kIangle, kDangle; 
  private final PIDController distanceController, turnController;
  private PhotonModule photonModule;

  public AprilMove(Drive m_drive, PhotonModule m_photonModule) {
    drive = m_drive;
    photonModule = m_photonModule;

    distanceController = new PIDController(kPdistance, kIdistance, kDdistance);
    turnController = new PIDController(kPangle, kIangle, kDangle);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    distanceController.reset();
    // goal is to maintain a set distance between the Apriltag and camera
    distanceController.setSetpoint(driveDistance);

    // goal is to center the robot on target
    turnController.setSetpoint(0);
    turnController.reset();

//  dont think we need these 
    // drive.resetLeftEncoder();
    // drive.resetRightEncoder();
  }

  @Override
  public void execute() {
    double leftSpeed = distanceController.calculate(photonModule.getX()); // + turnController.calculate(photonModule.getY());
    double rightSpeed = distanceController.calculate(photonModule.getX()); // - turnController.calculate(photonModule.getY());

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
