// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
public class ArmRetract extends CommandBase {
  /** Creates a new ArmRetract. */
  private Arm arm2;
  private double speed2;
  public ArmRetract(Arm armRet, double speedRet) {
  this.arm2 = armRet;
  this.speed2 = speedRet;
  addRequirements(arm2);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm2.setArmSpeed(-speed2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm2.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if ((speed2 > 0.008) && arm2.isAExtLimit()) {
      return true;
    } else if ((speed2 < 0) && arm2.isARetLimit()) {
      arm2.resetArmEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;
  }
  }