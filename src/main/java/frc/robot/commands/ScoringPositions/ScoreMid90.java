// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public class ScoreMid90 extends SequentialCommandGroup {

  /** Creates a new ScoreMiddleLevel for when Turret at 90 or 270 degrees */
  public ScoreMid90(Arm midScore90, Pivot pvtMid90, Gripper gripMid90, Turret turMid90) {   

  // if (pvtMid90.getPivotEncoder() > Constants.PivotConstants.PVT_ENC_MID_SCORE) {

  //   //From higher angle (getPivotEncoder > target): ArmPID then PivotDownPID
  //   addCommands(
  //  // new ArmPID(midScore90, Constants.ArmConstants.ARM_90_MID).withTimeout(1),
  //  // new PivotDownPID(pvtMid90, Constants.PivotConstants.PVT_ENC_90_MID_SCORE).withTimeout(1),
  //   new ReleasePiece(gripMid90).asProxy()
  //  // , new TurretPID(turMid90, 90).withTimeout(1)
  //   );
  //   }
  //   else {
  //     //SmartDashboard.putNumber("in ScoreMid90, desired turret angle is:", 90);

    //From lower angle (getPivotEncoder < target):  PivotPID (pivoting up) then ArmPID 
    addCommands(
    new PivotPID(pvtMid90, Constants.PivotConstants.PVT_ENC_90_MID_SCORE).withTimeout(1),
    
    new TurretPID(turMid90, 90).withTimeout(1),
    new ArmPID(midScore90, Constants.ArmConstants.ARM_90_MID).withTimeout(1)
    //,
    //new ReleasePiece(gripMid90).asProxy()
    );

    //}

 }
}


