// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Gripper.Grab;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.commands.ScoringPositions.Pickup;
import frc.robot.commands.ScoringPositions.PickupToStow;
import frc.robot.commands.ScoringPositions.ScoreHigh;
import frc.robot.commands.ScoringPositions.StoweFromUP;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCCRed extends SequentialCommandGroup {
  /** Creates a new ScoreCubeCone. */
  public ScoreCCRed(Arm armCC, Pivot pvtCC, Turret trrtCC, Gripper gripCC, Drive drCC) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(new PivotPID(pvtCC, 10500), new TurretPID(trrtCC, 139)).withTimeout(1.25),
      new ArmPID(armCC, 29.2).withTimeout(1),
      new ReleasePiece(gripCC).asProxy().withTimeout(0.75),
      new StoweFromUP(armCC, pvtCC, trrtCC).withTimeout(2),
     parallel(new DriveAtSetSpeed(drCC, 180.5, 0.65), new Pickup(armCC, pvtCC, gripCC)).withTimeout(3),
     new WaitCommand(0.2),
     new Grab(gripCC).asProxy().withTimeout(1),
     //new PickupToStow(pvtCC, armCC).withTimeout(2),
     parallel(new DriveAtSetSpeed(drCC, 180.5, -0.65), new PivotPID(pvtCC, 9500),
     new TurretPID(trrtCC, 171), new ArmPID(armCC, 20.8)).withTimeout(3.9),
     new ReleasePiece(gripCC).asProxy().withTimeout(1)
    );
  }
}
