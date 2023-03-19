// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
public class ArmDownPID extends CommandBase {
  /** Creates a new ArmPID. */
  private Arm arm5;
  private Pivot pivot8;
  private double armDistance, armSpeed;
  private final PIDController armPidController;

  public ArmDownPID(Arm armDpid, double armDDistance) {
    this.arm5 = armDpid;
   // this.pivot7 = pivotpid1;
    addRequirements(arm5);
   // addRequirements(pivot7);

    this.armDistance = armDDistance;
   
    this.armPidController = new PIDController(ArmConstants.kParmDown, ArmConstants.kIarmDown, ArmConstants.kDarmDown);  //delete line below after inserting this line
    armPidController.setSetpoint(armDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = armPidController.calculate(arm5.getArmDistance());
    arm5.setArmSpeed(armSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm5.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      //stop when near target and commanded speed close to 0
  if ((arm5.getArmDistance() > 0.97*armDistance)&& (Math.abs(armSpeed) < 0.02)) {
    SmartDashboard.putBoolean("ArmPID Finished?", true);
    return true;
  } else if (arm5.isARetLimit() && armSpeed < 0) {
    return true; }
    else if (arm5.isAExtLimit() && armSpeed > 0) {
      return true;
    } else {
    SmartDashboard.putBoolean("ArmPID Finished?", false);
    return false;
  }
  }
}