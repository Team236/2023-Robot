// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.Pivot.PivotDownPID;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadStationPosition extends SequentialCommandGroup {

  /** Creates a new LoadStationPosition. */
  public LoadStationPosition(Arm loadStation, Pivot loadPivot, Gripper gripLoad) {
     
    if (loadPivot.getPivotEncoder() > Constants.PivotConstants.PVT_ENC_LOAD_STN) {

    //From higher angle (getPivotEncoder > target): ArmPID then PivotDownPID
    addCommands(
    new ArmPID(loadStation,Constants.ArmConstants.ARM_LOAD_STN).withTimeout(1),
    new PivotDownPID(loadPivot, Constants.PivotConstants.PVT_ENC_90).withTimeout(1),
    new ReleasePiece(gripLoad).asProxy()
    );
    }
    else {

    //From lower angle (getPivotEncoder < target):  PivotPID (pivoting up) then ArmPID 
    addCommands(
    new PivotPID(loadPivot, Constants.PivotConstants.PVT_ENC_90).withTimeout(1),
    new ArmPID(loadStation, Constants.ArmConstants.ARM_LOAD_STN).withTimeout(1)//,
   // new ReleasePiece(gripLoad).asProxy()
    );

    }

  }
}

