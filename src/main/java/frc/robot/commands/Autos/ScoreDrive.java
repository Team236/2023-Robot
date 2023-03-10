// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.ScoringPositions.StowPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreDrive extends SequentialCommandGroup {
  /** Creates a new ScoreDrive. */
  public ScoreDrive(Arm armDrive, Pivot pvtDrive, Drive driveDrive, Gripper gDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoScoreHigh(armDrive, pvtDrive, gDrive).withTimeout(3),
      new ArmRetract(armDrive, 0.5).withTimeout(2),
   // new StowPosition(backwardA, backwardP).withTimeout(4),
      new PivotDown(pvtDrive, 0.5).withTimeout(2),
      new AutoPIDDrive(driveDrive, -170).withTimeout(4),
      new TurnPID(driveDrive, 180)
    );
  }
}
