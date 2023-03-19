// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotDownPID;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLow extends SequentialCommandGroup {

  
  /** Creates a new ScoreMiddleLevel. */
  public ScoreLow(Arm lowScore, Pivot pvtLow, Gripper gripScore) {

    if (pvtLow.getPivotEncoder() > Constants.PivotConstants.PVT_ENC_LOW_SCORE) {

      //From higher angle (getPivotEncoder > target): ArmPID then PivotDownPID
      addCommands(
      new ArmPID(lowScore,Constants.ArmConstants.ARM_LOW).withTimeout(1),
      new PivotDownPID(pvtLow, Constants.PivotConstants.PVT_ENC_LOW_SCORE).withTimeout(1),
      new ReleasePiece(gripScore).asProxy()
      );
      }
      else {

      //From lower angle (getPivotEncoder < target):  PivotPID (pivoting up) then ArmPID 
      addCommands(
      new PivotPID(pvtLow, Constants.PivotConstants.PVT_ENC_LOW_SCORE).withTimeout(1),
      new ArmPID(lowScore, Constants.ArmConstants.ARM_LOW).withTimeout(1),
      new ReleasePiece(gripScore).asProxy()
      );
  
      }
  }
}
