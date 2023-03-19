// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class TurretCCW extends CommandBase {
  private Turret turret;
  private double speed;
  /** Creates a new TurretClockwise. */
  public TurretCCW(Turret _turretccw, double _speedccw) {
    this.turret = _turretccw;
    this.speed = _speedccw;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretRelease();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setTurretSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretStop();
    turret.turretBrake();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}