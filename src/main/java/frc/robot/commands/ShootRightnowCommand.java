// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootRightnowCommand extends SequentialCommandGroup {
  /** Creates a new ShootRightnowCommand. */
  public ShootRightnowCommand(Shooter shooter, Amp amp, Limelight limelight, AngleShooter angleShooter, Drivetrain drivetrain, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drivetrain, limelight, angleShooter, shooter, intake, amp);
    addCommands(new WaitXSecondCommand(2));
     addCommands(new SetShooterStateCommand(shooter));
    addCommands(
        new ParallelRaceGroup(
            new WaitXSecondCommand(3),
            new SetShooterSpeedCommand(shooter, amp),
            new SetShooterAngleCommand(limelight, angleShooter))); // to speed up
    addCommands(new ParallelRaceGroup(new WaitXSecondCommand(2), new ShootNoteCommand(intake)));
    addCommands(new SetShooterStateCommand(shooter));
    addCommands(new AvancerXmCommand(drivetrain, -3.25));
    addCommands(new SetShooterSpeedCommand(shooter, amp));
    //addCommands(new AvancerXmCommand(drivetrain, -3.25)); // todo find the number of meter
  }
}
