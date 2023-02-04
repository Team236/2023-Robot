// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoTrapezoidalPID extends CommandBase {
  private Drive drive;
  private double trapDriveDistance, kPtrap, kItrap, kDtrap;
  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController pidController;
  public static double LL, RR, adjustL, adjustR, kPgyro, error;
  /** Creates a new AutoTrapezoidalPID. */
  public AutoTrapezoidalPID(Drive _drive, double _trapDriveDistance, double kPtrap, double kItrap, double kDtrap) {
    this.drive = _drive;
    this.trapDriveDistance = _trapDriveDistance;
    this.kPtrap = kPtrap;
    this.kItrap = kItrap;
    this.kDtrap = kDtrap;
     //this is from AutoPID drive - may need to change here depending on how fast robot is going 0.03
     kPgyro = 0.03; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
     //17.8 ft/sec or 13.3 ft/sec hi/lo gear max  (5.42 m/s or 4.05 m/s)
    constraints = new TrapezoidProfile.Constraints(1.5, 0.85);
    pidController = new ProfiledPIDController(kPtrap, kItrap, kDtrap, constraints);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRightEncoder();
    drive.resetLeftEncoder();
    pidController.reset(trapDriveDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidController.setGoal(trapDriveDistance);

    LL = pidController.calculate(drive.getLeftDistance());
    RR = pidController.calculate(drive.getRightDistance());


    error = drive.navX.getRate();

    if (RR <= 0) {
      adjustL = kPgyro*error;
      adjustR = -kPgyro*error;
   } else  {
     adjustL = -kPgyro*error;
     adjustR = kPgyro*error;
   }
   drive.leftFront.set(LL + adjustL);
   drive.rightFront.set(RR + adjustR);

    //drive.leftFront.set(pidController.calculate(drive.getLeftDistance()));
   // drive.rightFront.set(pidController.calculate(drive.getRightDistance()));
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
