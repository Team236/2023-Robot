// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMiddleLevel extends SequentialCommandGroup {

  
  /** Creates a new ScoreMiddleLevel. */
  public ScoreMiddleLevel(Arm apScore1, Gripper gripScore1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
   
     //new GrabReleaseToggle(gripScore1),
      new PivotPID(apScore1, 10065).withTimeout(1),
      new ArmPID(apScore1, Constants.ArmConstants.ARM_MID)
      );
      
  }
}
