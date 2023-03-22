// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class PivotDownPID extends CommandBase {
  private Pivot pivot4;
  private double pvtTarget4, pvtSpeed;
  private final PIDController pvtPidController;

  /** Creates a new PivotDownPID. */
  public PivotDownPID(Pivot m_pivotpid4, double m_pvtTarget4) {
    this.pivot4 = m_pivotpid4;
    addRequirements(pivot4);
    this.pvtTarget4 = m_pvtTarget4;
    this.pvtPidController = new PIDController(PivotConstants.kPpvtDown, PivotConstants.kIpvtDown, PivotConstants.kDpvtDown);
    pvtPidController.setSetpoint(pvtTarget4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pvtPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pvtSpeed = pvtPidController.calculate(pivot4.getPivotEncoder());
    pivot4.setPivotSpeed(pvtSpeed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot4.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (pivot4.getPivotEncoder() >= Math.abs(0.97*pvtTarget4)) {
      return true;
     } else {*/
      return false;
    // }
      
  }
}
