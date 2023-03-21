// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stowe extends SequentialCommandGroup {
  /** Creates a new StowPosition. */
  public Stowe(Arm armStow, Pivot pvtStow) {

addCommands(
  new ArmRetract(armStow, 0.5).withTimeout(2),
  new PivotDown(pvtStow, 0.25).withTimeout(6)


  );

  }

}

