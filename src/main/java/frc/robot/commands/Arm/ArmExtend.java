// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ArmExtend extends CommandBase {
  /** Creates a new ArmExtend. */
  private Arm arm;
  private XboxController controller;
  private double speed;

  public ArmExtend(Arm arm, double speed, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    /*if ((speed > 0.008) && arm.isAExtLimit()) {
      return true;
    } else if ((speed < 0) && arm.isARetLimit()) {
      arm.resetArmEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;
  }
}
