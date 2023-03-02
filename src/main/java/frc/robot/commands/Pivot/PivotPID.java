// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.PivotConstants;

public class PivotPID extends CommandBase {
  private Arm pivot2;
  private double pvtAngle;
  private final PIDController pvtPidController;
  /** Creates a new PivotPID. */
  public PivotPID(Arm pivotpid, double pvtAngle) {
    this.pivot2 = pivotpid;
    addRequirements(pivot2);
    this.pvtAngle = pvtAngle;
    this.pvtPidController = new PIDController(PivotConstants.kPpvt, PivotConstants.kIpvt, PivotConstants.kDpvt);
    pvtPidController.setSetpoint(pvtAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pvtPidController.reset();
   pivot2.resetPivotEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double pvtSpeed = pvtPidController.calculate(pivot2.getPivotAngle());
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
    return false;
    
  }
}
