// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StoweFromUP extends SequentialCommandGroup {

  /** Creates a new StoweFromUP. */
  public StoweFromUP(Arm armSFU, Pivot pivotSFU, Turret turretSFU) {

    SmartDashboard.putNumber("turret angle in SFU: ", turretSFU.getTurretAngle());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ArmRetract(armSFU, 0.85).withTimeout(2),  //arm retracts to limit at speed of 0.5
    parallel(new TurretPID(turretSFU, 0.5).withTimeout(3),  //moves Turret to 0.5 degrees
    new PivotDown(pivotSFU, 0.55).withTimeout(4))  //pivot down to limit as speed 0.25

    );
  
}
}