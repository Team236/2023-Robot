// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PivotUp extends CommandBase {
  private Arm pivot3;
  private double speed3;
  /** Creates a new PivotCW. */
  public PivotUp(Arm pivotup, double speedup) {
    this.pivot3 = pivotup;
    this.speed3 = speedup;
    addRequirements(pivot3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot3.setPivotSpeed(speed3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot3.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pivot3.isPHighLimit()) {
      return true;
    } else {
    return false;}
  }
}