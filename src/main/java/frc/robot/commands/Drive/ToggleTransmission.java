// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleTransmission extends CommandBase {
  private Drive drive;
  private boolean toggle;
  /** Creates a new SwitchGear. */
  public ToggleTransmission(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggle = false;
    
    if (drive.inLowGear()) {
      drive.highGear();
     toggle = true;
     } else if (!drive.inLowGear()) {
     drive.lowGear();
     toggle = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
