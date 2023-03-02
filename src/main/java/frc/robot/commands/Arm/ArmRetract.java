// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
public class ArmRetract extends CommandBase {
  /** Creates a new ArmRetract. */
  private Arm arm2;
  private double speed;
  public ArmRetract(Arm armretract, double speed) {
  this.arm2 = armretract;
  addRequirements(arm2);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm2.setArmSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm2.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   /*  if ((speed > 0.008) && arm.isAExtLimit()) {
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
