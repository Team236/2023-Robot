// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
/** An example command that uses an example subsystem. */
public class DriveWithJoysticks extends CommandBase {
 
  private Drive drive;
  private Joystick leftstick, rightStick;
  private Boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoysticks(Drive drive, Joystick leftStick, Joystick rightStick) {
    this.drive = drive;
    this.leftstick = leftStick;
    this.rightStick = rightStick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
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
