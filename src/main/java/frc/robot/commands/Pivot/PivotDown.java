 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.PivotConstants;
public class PivotDown extends CommandBase {
  private Pivot pivot;
  private double speed;
  /** Creates a new PivotCCW. */
  public PivotDown(Pivot _pivotdown, double _speeddown) {
    this.pivot = _pivotdown;
    this.speed = _speeddown;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setPivotSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   pivot.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (pivot.isPLowLimit()) {
   pivot.resetPivotEncoder();
   
   return true; }
   else {
    return false;
   }

  }
}