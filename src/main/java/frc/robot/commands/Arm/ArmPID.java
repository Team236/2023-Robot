// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
public class ArmPID extends CommandBase {
  /** Creates a new ArmPID. */
  private Arm arm3;
  private double armDistance, armSpeed;
  private final PIDController armPidController;

  public ArmPID(Arm armpid, double armDistance) {
    this.arm3 = armpid;
    addRequirements(arm3);
    this.armDistance = armDistance;
    this.armPidController = new PIDController(ArmConstants.kParm, ArmConstants.kIarm, ArmConstants.kDarm);
    armPidController.setSetpoint(armDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPidController.reset();
    arm3.resetArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = armPidController.calculate(arm3.getArmDistance());
    arm3.setArmSpeed(armSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm3.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      //stop when near target and commanded speed close to 0
  if ((arm3.getArmDistance() > 0.9*armDistance)&& (Math.abs(armSpeed) < 0.02)) {
    SmartDashboard.putBoolean("ArmPID Finished?", true);
    return true;
  } else if (arm3.isARetLimit() && armSpeed < 0) {
    return true; }
    else if (arm3.isAExtLimit() && armSpeed> 0) {
      return true;
    } else {
    SmartDashboard.putBoolean("ArmPID Finished?", false);
    return false;
  }
  }
}