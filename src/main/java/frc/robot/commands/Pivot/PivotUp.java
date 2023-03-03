// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PivotUp extends CommandBase {
  private Arm arm;
  private double speed;
  /** Creates a new PivotCW. */
  public PivotUp(Arm pivotup, double speed5) {
    this.arm = pivotup;
    this.speed = speed5;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPivotSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      /*  if ((speed > 0.008) && pivot3.isPHighLimit()) {
      return true;
    } else if ((speed < 0) && turret.isPLowLimit()()) {
      turret.resetTurretEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;
  }
}
