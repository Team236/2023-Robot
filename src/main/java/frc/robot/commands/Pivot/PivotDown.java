 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PivotConstants;
public class PivotDown extends CommandBase {
  private Arm pivot;
  private double speed;
  private XboxController controller;
  /** Creates a new PivotCCW. */
  public PivotDown(Arm pivot, double speed, XboxController controller) {
    this.pivot = pivot;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pivot.setPivotSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // pivot.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*  if ((speed > 0.008) && pivot.isPHighLimit()) {
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
