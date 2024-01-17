// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private Talon shooterGauche;

  private Talon shooterDroit;

  public Shooter() {
    OperatorConstants deviceNumber = new OperatorConstants();
    shooterGauche = new Talon(deviceNumber.DeviceNumberShooterGauche);
    shooterDroit = new Talon(deviceNumber.DeviceNumberShooterDroit);

    shooterGauche.setInverted(false);
    shooterDroit.setInverted(false);
  }

  public void shoot(double activated) {
    shooterGauche.set(activated);
    shooterDroit.set(activated);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
