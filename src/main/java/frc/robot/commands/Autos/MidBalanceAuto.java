// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidBalanceAuto extends SequentialCommandGroup {
  /** Creates a new BackwardCenter. */
  public MidBalanceAuto(Arm backwardA, Gripper backwardG, Drive backwardD, Pivot backwardP) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutoScoreMid(backwardP, backwardA, backwardG).withTimeout(3),
    new ArmRetract(backwardA, 0.7).withTimeout(1.5),
    new PivotDown(backwardP, 0.75).withTimeout(1.5),
    new DriveAtSetSpeed(backwardD, 148, -0.3),
    new DriveAtSetSpeed(backwardD, 70, 0.3)
    );
  }
}
