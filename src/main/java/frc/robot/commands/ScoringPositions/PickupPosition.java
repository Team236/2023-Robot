// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupPosition extends SequentialCommandGroup {
  /** Creates a new PickupPosition. */
  public PickupPosition(Arm armPickup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmPID(armPickup, 0),
      new PivotPID(armPickup, 1105)
    );
  }
}
