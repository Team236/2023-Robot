// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoTrapezoidalPID extends CommandBase {
  private Drive drive;
  private double trapDriveDistance, kPtrap, kItrap, kDtrap;
  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController pidController;
  /** Creates a new AutoTrapezoidalPID. */
  public AutoTrapezoidalPID(Drive _drive, double _trapDriveDistance, double kPtrap, double kItrap, double kDtrap) {
    this.drive = _drive;
    this.trapDriveDistance = _trapDriveDistance;
    this.kPtrap = kPtrap;
    this.kItrap = kItrap;
    this.kDtrap = kDtrap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
    pidController = new ProfiledPIDController(kPtrap, kItrap, kDtrap, constraints);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(trapDriveDistance);

    drive.leftFront.set(pidController.calculate(drive.getLeftDistance()));
    drive.rightFront.set(pidController.calculate(drive.getRightDistance()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
