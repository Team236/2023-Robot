// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class ArmPID extends CommandBase {
  /** Creates a new ArmPID. */
  private Arm arm;
  private double armDistance;
  private double armMargin;
  private double armError;
  private final PIDController armPidController;

  public ArmPID(Arm arm, double armDistance, double armMargin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(this.arm);

    this.armDistance = armDistance;
    this.armMargin = armMargin;
    this.armPidController = new PIDController(ArmConstants.kParm, ArmConstants.kIarm, ArmConstants.kDarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPidController.reset();
    arm.resetArmEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = armPidController.calculate(arm.getArmDistance());
    arm.setArmSpeed(armSpeed);
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((armDistance > 0) && arm.isAExtendLimit()) {
      return true;
    } else if ((armDistance < 0) && arm.isAReturnLimit()) {
      arm.resetArmEncoder();
      return true;
    } else {
      return false;
    }

  }
}
