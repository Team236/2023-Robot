 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PivotConstants;
public class PivotDown extends CommandBase {
  private Arm pivot4;
  private double speed4;
  /** Creates a new PivotCCW. */
  public PivotDown(Arm pivotdown, double speeddown) {
    this.pivot4 = pivotdown;
    this.speed4 = speeddown;
    addRequirements(pivot4);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot4.setPivotSpeed(-speed4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   pivot4.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*  if ((speed4 > 0.008) && pivot4.isPHighLimit()) {
      return true;
    } else if ((speed4 < 0) && pivot4.isPLowLimit()()) {
      pivot4.resetPivotEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;
  }
}