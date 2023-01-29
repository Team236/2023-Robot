// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;

public class DoubleArcadeDrive extends CommandBase {
  private Drive drive;
  private Gripper gripper1;
  private Boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;
  /** Creates a new DoubleArcadeDrive. */
  public DoubleArcadeDrive(Drive drive, Gripper gripper1) {
    this.gripper1 = gripper1;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper1);
    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDeadzone = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setArcadeSpeed();
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
