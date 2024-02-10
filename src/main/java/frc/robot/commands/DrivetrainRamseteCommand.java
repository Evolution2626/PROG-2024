// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.util.TrajectoryLoader;

public class DrivetrainRamseteCommand extends RamseteCommand {
  private boolean resetPosition;
  private Drivetrain drivetrain;
  private Trajectory trajectory;
  /** Creates a new DrivetrainRamseteCommand. */
  public DrivetrainRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) {
    super(trajectory, 
      drivetrain::getPose, 
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
      DriveConstants.kDriveKinematics, 
      drivetrain::getWheelSpeed, 
      new PIDController(DriveConstants.kPDriveVel, 0, 0), 
      new PIDController(DriveConstants.kPDriveVel, 0, 0), 
      drivetrain::tankDriveVolts, 
      drivetrain);

      this.drivetrain = drivetrain;
      this.trajectory = trajectory;
      this.resetPosition = true;
  }

  public DrivetrainRamseteCommand(Drivetrain drivetrain, String path){
    this(drivetrain, TrajectoryLoader.getTrajectory(path));
  }
  public DrivetrainRamseteCommand(Drivetrain drivetrain, String... paths){
    this(drivetrain , TrajectoryLoader.getTrajectory(paths));
  }
  public DrivetrainRamseteCommand robotRelative(){
    this.resetPosition = true;
    return this;
  }

  public DrivetrainRamseteCommand fieldRelative(){
    this.resetPosition = false;
    return this;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    if (resetPosition) {
      drivetrain.resetOdometry(trajectory.getInitialPose());
    }
  }
}

