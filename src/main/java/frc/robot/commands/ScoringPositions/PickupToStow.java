// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupToStow extends SequentialCommandGroup {
  /** Creates a new PickupToStow. */
  public PickupToStow(Pivot pvtPs, Arm armPs) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PivotPID(pvtPs, 3000).withTimeout(1),
      new ArmPID(armPs, 0.1).withTimeout(2),
      new PivotDown(pvtPs, 0.75).withTimeout(5)
    );
  }
}
