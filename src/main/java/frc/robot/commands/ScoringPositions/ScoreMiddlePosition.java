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
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMiddlePosition extends SequentialCommandGroup {

  
  /** Creates a new ScoreMiddleLevel. */
  public ScoreMiddlePosition(Arm midScore, Gripper gripScore2, Pivot pvtMid) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
   
     //new GrabReleaseToggle(gripScore1),
      new PivotPID(pvtMid, PivotConstants.PVT_ENC_MID_SCORE).withTimeout(1),
      new ArmPID(midScore, Constants.ArmConstants.ARM_MID)//,
      //new WaitCommand(0.5), new GrabReleaseToggle(gripScore2)
      );
      
  }
}
