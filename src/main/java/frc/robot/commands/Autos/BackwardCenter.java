// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.commands.ScoringPositions.ScoreHighPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackwardCenter extends SequentialCommandGroup {
  /** Creates a new BackwardCenter. */
  public BackwardCenter(Arm backwardA, Gripper backwardG, Drive backwardD) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new PivotPID(backwardA, PivotConstants.PVT_ENC_HIGH_SCORE).withTimeout(1),
     // new ArmPID(backwardA, Constants.ArmConstants.ARM_HIGH).withTimeout(1),
      //new WaitCommand(1),
    // new ArmPID(backwardA, -Constants.ArmConstants.ARM_HIGH).withTimeout(1),
    //new PivotPID(backwardA, -10000).withTimeout(1),
    new AutoPIDDrive(backwardD, -30).withTimeout(3),
    new WaitCommand(1),
    new TurnPID(backwardD, 180)
    );
  }
}
