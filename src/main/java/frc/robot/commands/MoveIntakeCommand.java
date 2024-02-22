// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.util.Range;

public class MoveIntakeCommand extends Command {
  private Intake intake;
  private CommandXboxController xboxController;
  private PIDController pid = new PIDController(0.00125, 0, 0);

  /** Creates a new MoveIntakeCommand. */
  public MoveIntakeCommand(Intake intake, CommandXboxController xboxController) {
    this.intake = intake;
    this.xboxController = xboxController;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.wantedInside()) {
      intake.moveIntake(Range.coerce(0, 1, pid.calculate(intake.getVelocity(), 200)));
    } else {
      intake.moveIntake(Range.coerce(-1, 0, -pid.calculate(Math.abs(intake.getVelocity()), 200)));
    }

    intake.spinWheel(xboxController.getLeftTriggerAxis() - xboxController.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
