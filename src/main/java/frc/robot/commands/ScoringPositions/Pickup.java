// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.Arm.ArmDownPID;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pickup extends SequentialCommandGroup {

  /** Creates a new PickupPosition. */
  public Pickup (Arm Armpickup, Pivot pvtPickup, Gripper gripPickup) {
    addCommands( new PivotPID(pvtPickup, PivotConstants.PVT_ENC_PICKUP).withTimeout(1),
    new ArmDownPID(Armpickup, ArmConstants.ARM_PICKUP).withTimeout(1.5));

  /*if (pvtPickup.getPivotEncoder() > Constants.PivotConstants.PVT_ENC_PICKUP) {
    
    //From higher angle (getPivotEncoder > target): ArmPID then PivotDownPID
    addCommands(
    new ArmPID(Armpickup, Constants.ArmConstants.ARM_PICKUP).withTimeout(1),
    new PivotDownPID(pvtPickup, Constants.PivotConstants.PVT_ENC_PICKUP).withTimeout(1),
    new ReleasePiece(gripPickup).asProxy()
    );
    }
    else {

    //From lower angle (getPivotEncoder < target):  PivotPID (pivoting up) then ArmPID 
    addCommands(
    new PivotPID(pvtPickup, Constants.PivotConstants.PVT_ENC_PICKUP).withTimeout(1),
    new ArmPID(Armpickup, Constants.ArmConstants.ARM_PICKUP).withTimeout(1),
    new ReleasePiece(gripPickup).asProxy()
    );*/
    }

}
//}

  
