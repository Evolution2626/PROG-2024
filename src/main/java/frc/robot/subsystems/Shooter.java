// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterGauche;

  private CANSparkMax shooterDroit;
  private CANSparkMax pusherGauche;
  private CANSparkMax pusherDroit;

  private RelativeEncoder shooterDroitEncoder;
  private RelativeEncoder shooterGaucheEncoder;

  public Shooter() {
    shooterGauche = new CANSparkMax(Constants.OperatorConstants.DeviceNumberShooterGauche, MotorType.kBrushless);
    shooterDroit = new CANSparkMax(Constants.OperatorConstants.DeviceNumberShooterDroit, MotorType.kBrushless);
    pusherGauche = new CANSparkMax(Constants.OperatorConstants.DeviceNumberPusherGauche, MotorType.kBrushless);
    pusherDroit = new CANSparkMax(Constants.OperatorConstants.DeviceNumberPusherDroit, MotorType.kBrushless);

    shooterGauche.setInverted(false);
    shooterDroit.setInverted(false);
    pusherGauche.setInverted(false);
    pusherDroit.setInverted(false);

    shooterDroitEncoder = shooterDroit.getAlternateEncoder(42);
    shooterGaucheEncoder = shooterGauche.getAlternateEncoder(42);
  }

  public void shooterPower(double powerDroit, double powerGauche) {
    shooterGauche.set(powerGauche);
    shooterDroit.set(powerDroit);
  }

  public double getVelocityDroit() {
    return shooterDroitEncoder.getVelocity();
  }

  public double getVelocityGauche() {
    return shooterGaucheEncoder.getVelocity();
  }

  public void pusherPower(double power) {
    pusherDroit.set(power);
    pusherGauche.set(power);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
