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
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighN90 extends SequentialCommandGroup {

  
  /** Creates a new ScoreHighPosition when Turret at 90 or 270 degrees. */
  public ScoreHighN90(Arm hiScoreN90, Pivot pvtHiN90, Gripper gripHighN90, Turret turHighN90) {
   
    addCommands(
      new PivotPID(pvtHiN90, PivotConstants.PVT_ENC_90_HIGH_SCORE).withTimeout(1.5),
      parallel(new TurretPID(turHighN90, -90).withTimeout(1),
      new ArmPID(hiScoreN90, Constants.ArmConstants.ARM_90_HIGH).withTimeout(2))//,
      //new ReleasePiece(gripHigh90).asProxy()
      );
      
  }
}