// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class GridToCenterPiece extends CommandBase {
  private Drive drive;
  private double driveDistance;
  private final PIDController leftPidController, rightPidController;

  /** Creates a new DriveStraight. */
  public GridToCenterPiece(Drive drive, double driveDistance) {
    this.drive = drive;
    this.leftPidController = new PIDController(DriveConstants.leftkPdrive, DriveConstants.leftkIdrive, DriveConstants.leftkDdrive);
    this.rightPidController = new PIDController(DriveConstants.rightkPdrive, DriveConstants.rightkIdrive, DriveConstants.rightkDdrive);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    leftPidController.setSetpoint(driveDistance);
    rightPidController.setSetpoint(driveDistance);
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
    double leftSpeed = leftPidController.calculate(drive.getLeftDistance());
    double rightSpeed = rightPidController.calculate(drive.getRightDistance());
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(rightSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
