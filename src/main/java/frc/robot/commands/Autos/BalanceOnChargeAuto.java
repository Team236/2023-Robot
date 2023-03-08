// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import frc.robot.commands.Drive.AutoBalanceGyro;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceOnChargeAuto extends SequentialCommandGroup {
  /** Creates a new BalanceOnChargeAuto. */
  public BalanceOnChargeAuto(Drive driveauto,  XboxController driveController1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveAtSetSpeed(driveauto, 164, -0.4),
      new WaitCommand(0.5),
      new DriveAtSetSpeed(driveauto, 100, 0.4)
      //, new AutoBalanceGyro(driveauto, driveController1)
    );
  }
}
