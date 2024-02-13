// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTestCommand extends SequentialCommandGroup {

  private static String testPath = "paths/testPath.wpilib.json";


  /** Creates a new GoberUnBallonCommand. */
  public AutoTestCommand(Drivetrain drivetrain) {
    drivetrain.resetEncoder();
    
    addRequirements(drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(new TrajectoryCommand(drivetrain, testPath));
  
    
  }
}