// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoringPositions.ScoreMid;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreToCenter extends SequentialCommandGroup {
  /** Creates a new ScoreToCenter. */
  public ScoreToCenter(Arm scoreCenter, Gripper no, Drive driveCenter, Pivot pvtCent, XboxController driver) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreMid(scoreCenter, pvtCent, no),
      new DriveAtSetSpeed(driveCenter, 180, -0.4)
    );
  }
}
