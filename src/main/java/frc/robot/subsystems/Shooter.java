// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterHaut;

  private CANSparkMax shooterBas;

  private CANSparkMax pusher;

  private RelativeEncoder pusherEncoder;
  private RelativeEncoder shooterBasEncoder;
  private RelativeEncoder shooterHautEncoder;

  public Shooter() {
    shooterHaut = new CANSparkMax(CAN.DeviceNumberShooterHaut, MotorType.kBrushless);
    shooterBas = new CANSparkMax(CAN.DeviceNumberShooterBas, MotorType.kBrushless);

    shooterHaut.setInverted(false);
    shooterBas.setInverted(false);

    shooterBasEncoder = shooterBas.getEncoder();
    shooterHautEncoder = shooterHaut.getEncoder();
    shooterBas.setIdleMode(IdleMode.kCoast);
    shooterHaut.setIdleMode(IdleMode.kCoast);
    shooterBas.setSmartCurrentLimit( 30);
    shooterHaut.setSmartCurrentLimit(30);
    shooterHaut.setIdleMode(IdleMode.kCoast);
    shooterBas.setIdleMode(IdleMode.kCoast);

    shooterBas.burnFlash();
    shooterHaut.burnFlash();

    pusher = new CANSparkMax(CAN.DeviceNumberPusher, MotorType.kBrushless);
    pusher.setInverted(false);
    pusherEncoder = pusher.getEncoder();
    pusher.setSmartCurrentLimit(30);

    pusher.burnFlash();
  }

  public void shooterPower(double powerDroit, double powerGauche) {
    shooterHaut.set(powerGauche);
    shooterBas.set(-powerDroit);
  }

  public void pusherPower(double power) {
    pusher.set(power);
  }

  public double getVelocityBas() {
    return shooterBasEncoder.getVelocity();
  }

  public double getVelocityHaut() {
    return shooterHautEncoder.getVelocity();
  }

  public double getEncoderPosPusher() {
    return pusherEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("velocityBas", getVelocityBas());
    SmartDashboard.putNumber("velocityHaut", getVelocityHaut());
    SmartDashboard.putNumber("powerBas", shooterBas.get());
    SmartDashboard.putNumber("powerHaut", shooterHaut.get());
    // This method will be called once per scheduler run
  }
}
