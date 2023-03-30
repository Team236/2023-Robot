// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Gripper.Grab;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.ScoringPositions.Pickup;
import frc.robot.commands.ScoringPositions.ScoreMid;
import frc.robot.commands.Turret.TurretCCW;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score2withSpin extends SequentialCommandGroup {
  /** Creates a new ScoreToCenter. */
  public Score2withSpin(Arm scoreSpin, Gripper gripSpin, Drive driveSpin, Pivot pvtSpin, Turret trrtSpin, XboxController driver) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //   new AutoScoreHigh(scoreSpin, pvtSpin, gripSpin).withTimeout(2),
    //  parallel(new DriveAtSetSpeed(driveSpin, 180, -0.5).withTimeout(4),
    //   new TurretPID(trrtSpin, 180).withTimeout(2),  
    //   new ArmRetract(scoreSpin, 0.7).withTimeout(1.5),
    //   new PivotDown(scoreSpin, 0.75).withTimeout(1.5)),
    //   new Pickup(scoreSpin, pvtSpin, gripSpin),
    //   new Grab(gripSpin).asProxy(),
    //   parallel(new TurretCCW(trrtSpin, 0.2).withTimeout(2), new DriveAtSetSpeed(driveSpin, 180, 0.5)).withTimeout(4),
    //   //new AutoScoreMid(pvtSpin, scoreSpin, gripSpin).withTimeout(2)

    );
  }
} 
