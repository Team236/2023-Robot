// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
public class TurretPID extends CommandBase {
  private Turret turret3;
  private double turretAngle;
  private final PIDController turretPidController;
  /** Creates a new TurretPID. */
  public TurretPID(Turret turretpid, double turretAngle) {
    this.turret3 = turretpid;
    addRequirements(turret3);
    turretPidController = new PIDController(TurretConstants.kPturret, TurretConstants.kIturret, TurretConstants.kDturret);
    this.turretAngle = turretAngle;
    turretPidController.setSetpoint(turretAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret3.resetTurretEncoder();
    turretPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turretSpeed = turretPidController.calculate(turret3.getTurretAngle());
    turret3.setTurretSpeed(turretSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   turret3.turretStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}