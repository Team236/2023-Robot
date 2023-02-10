// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
/** An example command that uses an example subsystem. */
public class DriveWithJoysticks extends CommandBase {
 private Gripper gripper2;
  private Drive drive;
  private XboxController xboxController;
  private Boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoysticks(Drive drive, Gripper gripper2, XboxController xboxController) {
    this.drive = drive;
    this.gripper2 = gripper2;
    this.xboxController = xboxController;
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
      drive.setRightSpeedWithDeadzone(-xboxController.getRightY());
      drive.setLeftSpeedWithDeadzone(-xboxController.getLeftY());
      gripper2.autoGrab();
    } else {
      drive.setLeftSpeed(-xboxController.getLeftY());
      drive.setRightSpeed(-xboxController.getRightY());
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
