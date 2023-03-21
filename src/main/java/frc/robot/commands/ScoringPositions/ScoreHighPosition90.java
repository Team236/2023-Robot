// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.Arm.ArmExtend;
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
public class ScoreHighPosition90 extends SequentialCommandGroup {

  
  /** Creates a new ScoreHighPosition when Turret at 90 or 270 degrees. */
  public ScoreHighPosition90(Arm hiScore90, Pivot pvtHi90, Gripper gripHigh90) {
   
    addCommands(
      new PivotPID(pvtHi90, PivotConstants.PVT_ENC_90_HIGH_SCORE),
      new ArmPID(hiScore90, Constants.ArmConstants.ARM_90_HIGH)//,
      //new ReleasePiece(gripHigh90).asProxy()
      );
      
  }
}
