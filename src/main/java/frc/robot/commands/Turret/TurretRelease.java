// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretRelease extends CommandBase {

  private Turret turret6;

  /** Creates a new TurretBrake. */
  public TurretRelease(Turret releaseTurret){
      this.turret6 = releaseTurret;
      addRequirements(turret6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret6.turretRelease();
   SmartDashboard.putBoolean("turret brake is on:", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
