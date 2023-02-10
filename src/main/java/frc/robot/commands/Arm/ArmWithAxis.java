// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ControllerConstants;
public class ArmWithAxis extends CommandBase {
  /** Creates a new ArmWithAxis. */
  private Arm arm;
  private XboxController controller;
  private double speed;

  public ArmWithAxis(Arm arm, XboxController controller) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -controller.getRawAxis(ControllerConstants.XboxController.AxesXbox.LY);
    arm.setArmSpeed(speed);
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armStop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
