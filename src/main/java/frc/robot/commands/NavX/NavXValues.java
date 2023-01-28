// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NavX;
import frc.robot.subsystems.NavX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class NavXValues extends CommandBase {
  private NavX navx = new NavX();
  /** Creates a new NavXCMD. */
  public NavXValues(NavX navx) {
    this.navx = navx;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("X", navx.navxgetx());
    SmartDashboard.putNumber("Y", navx.navxgety());
    SmartDashboard.putNumber("Z", navx.navxgetz());
    SmartDashboard.putNumber("XA", navx.navxgetxa());
    SmartDashboard.putNumber("YA", navx.navxgetya());
    SmartDashboard.putNumber("ZA", navx.navxgetza());
    SmartDashboard.putNumber("YAW", navx.navxgetyaw());
    SmartDashboard.putNumber("PITCH", navx.navxgetpitch());
    SmartDashboard.putNumber("ROLL", navx.navxgetroll());
    SmartDashboard.putNumber("Angle", navx.getangle());
    SmartDashboard.putNumber("Rate", navx.getrate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
