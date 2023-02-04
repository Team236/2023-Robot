// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
/** An example command that uses an example subsystem. */
public class DriveWithJoysticks extends CommandBase {
 private Gripper gripper2;
  private Drive drive;
  private Joystick leftstick, rightStick;
  private Boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoysticks(Drive drive, Gripper gripper2, Joystick leftStick, Joystick rightStick) {
    this.drive = drive;
    this.gripper2 = gripper2;
    this.leftstick = leftStick;
    this.rightStick = rightStick;
    addRequirements(this.drive);
    addRequirements(this.gripper2);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  this.isDeadzone = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.isDeadzone) {
      drive.setRightSpeedWithDeadzone(-rightStick.getY());
      drive.setLeftSpeedWithDeadzone(-leftstick.getY());
      gripper2.autoGrab();
    } else {
      drive.setLeftSpeed(-leftstick.getY());
      drive.setRightSpeed(-rightStick.getY());
      gripper2.autoGrab();
    }
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
