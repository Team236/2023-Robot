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
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLowN90 extends SequentialCommandGroup {

  
  /** Creates a new ScoreLow for when Turret at 90 or 270 degrees */
  public ScoreLowN90(Arm lowScore90, Pivot pvtLow90, Gripper gripLow90, Turret turLow90) {

    // if (pvtLow90.getPivotEncoder() > Constants.PivotConstants.PVT_ENC_LOW_SCORE) {

    //   //From higher angle (getPivotEncoder > target): ArmPID then PivotDownPID
    //   addCommands(
    //   new ArmPID(lowScore90,Constants.ArmConstants.ARM_90_LOW).withTimeout(1),
    //   new PivotDownPID(pvtLow90, Constants.PivotConstants.PVT_ENC_90_LOW_SCORE).withTimeout(1)//,
    //   //new ReleasePiece(gripLow90).asProxy()
    //   );
    //   }
    //   else {

      //From lower angle (getPivotEncoder < target):  PivotPID (pivoting up) then ArmPID 
      addCommands(
      new PivotPID(pvtLow90, Constants.PivotConstants.PVT_ENC_90_LOW_SCORE).withTimeout(1),
      new TurretPID(turLow90, -90).withTimeout(1),
      new ArmPID(lowScore90, Constants.ArmConstants.ARM_90_LOW).withTimeout(1)//,
      //new ReleasePiece(gripLow90).asProxy()
      );
  
      //}
  }
}

