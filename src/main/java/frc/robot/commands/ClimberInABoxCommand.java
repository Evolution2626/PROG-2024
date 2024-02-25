// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberInAnBox;

public class ClimberInABoxCommand extends Command {
  private ClimberInAnBox climberInAnBox;
  private CommandXboxController xboxController;


  /** Creates a new ClimberInABoxCommand. */
  public ClimberInABoxCommand(ClimberInAnBox climberInAbox, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberInAnBox = climberInAbox;
    this.xboxController = xboxController;
    addRequirements(climberInAbox);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getLeftY() != 0 || xboxController.getRightY() != 0){
      if(xboxController.x().getAsBoolean() && climberInAnBox.getclimberOut()){
        climberInAnBox.activateRatchet();
        climberInAnBox.climb(
        Math.abs(xboxController.getLeftY()), Math.abs(xboxController.getRightY()));
      }
      else if(!climberInAnBox.isRatchetActivated()){
        climberInAnBox.setClimberOut(true);
        climberInAnBox.climb(
       -Math.abs(xboxController.getLeftY()), Math.abs(xboxController.getRightY()));// TODO check orientation
      }
    }
    else{
      climberInAnBox.climb(0,0);
    }
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
