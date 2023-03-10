// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

public class PivotPID extends CommandBase {
  private Pivot pivot2;
  private double pvtTarget, pvtSpeed;
  private final PIDController pvtPidController;
  /** Creates a new PivotPID. */
  public PivotPID(Pivot m_pivotpid2, double m_pvtTarget) {
    this.pivot2 = m_pivotpid2;
    addRequirements(pivot2);
    this.pvtTarget = m_pvtTarget;
    this.pvtPidController = new PIDController(PivotConstants.kPpvt, PivotConstants.kIpvt, PivotConstants.kDpvt);
    pvtPidController.setSetpoint(pvtTarget);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pvtPidController.reset();
   //pivot2.resetPivotEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   pvtSpeed = pvtPidController.calculate(pivot2.getPivotEncoder());
  pivot2.setPivotSpeed(pvtSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   pivot2.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (pivot2.getPivotEncoder() >= Math.abs(0.97*pvtTarget)) {
    return true;
   } else {
    return false;
   }
    
  }
}