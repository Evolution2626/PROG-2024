// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ClimberInAnBox extends SubsystemBase {
  private Talon climberDroit;
  private Talon climberGauche;

  /** Creates a new ClimberInAnBox. */
  public ClimberInAnBox() {
    OperatorConstants deviceNumber = new OperatorConstants();
    climberDroit = new Talon(deviceNumber.DeviceNumberClimberDroit);
    climberGauche = new Talon(deviceNumber.DeviceNumberClimberGauche);

    climberDroit.setInverted(false);
    climberGauche.setInverted(false);
  }

  public void climb(double activated) {
    climberDroit.set(activated);
    climberGauche.set(activated);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
