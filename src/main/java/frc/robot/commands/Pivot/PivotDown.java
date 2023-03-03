 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PivotConstants;
public class PivotDown extends CommandBase {
  private Arm pivot1;
  private double speed;
  /** Creates a new PivotCCW. */
  public PivotDown(Arm pivotdown, double speed4) {
    this.pivot1 = pivotdown;
    this.speed = speed4;
    addRequirements(pivot1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot1.setPivotSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   pivot1.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*  if ((speed > 0.008) && pivot1.isPHighLimit()) {
      return true;
    } else if ((speed < 0) && pivot1.isPLowLimit()()) {
      pivot1.resetPivotEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;
  }
}
