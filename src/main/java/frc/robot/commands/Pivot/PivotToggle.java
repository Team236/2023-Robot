// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class PivotToggle extends CommandBase {

  private Pivot pivot;
  private boolean toggle;

  /** Creates a new PivotToggle. */
  public PivotToggle(Pivot pivot) {

    this.pivot=pivot;

    addRequirements(pivot);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (pivot.isextended()) {
      pivot.retract();
      toggle = true;
  
      } else if (!pivot.isextended()) {
      pivot.extend();
      toggle = true;
    }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;
  }
}
