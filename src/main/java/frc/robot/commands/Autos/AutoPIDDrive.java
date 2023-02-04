// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AutoPIDDrive extends CommandBase {
  private Drive drive;
  private double driveDistance, gyrokP, spin;
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
   spin = drive.navX.getRate();
    gyrokP = 0.03;
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
    //negative speed applicable????/
    double leftSpeed = (((leftPidController.calculate(drive.getLeftDistance())) - (spin*gyrokP)));
    double rightSpeed = ((rightPidController.calculate(drive.getRightDistance())) + (spin*gyrokP));
    //double leftSpeed = (leftPidController.calculate(drive.getLeftDistance()));
   // double rightSpeed = (rightPidController.calculate(drive.getRightDistance()));
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(rightSpeed);

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
