// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int currentR;
  private int currentG;
  private int currentB;

  public enum shooterPossibleState {
    OFF,
    SPEAKER,
    AMP
  }

  private shooterPossibleState shooterState = shooterPossibleState.OFF;

  public Shooter() {
    shooterHaut = new CANSparkMax(CAN.DeviceNumberShooterHaut, MotorType.kBrushless);
    shooterBas = new CANSparkMax(CAN.DeviceNumberShooterBas, MotorType.kBrushless);

    shooterHaut.setInverted(false);
    shooterBas.setInverted(false);

    shooterBasEncoder = shooterBas.getEncoder();
    shooterHautEncoder = shooterHaut.getEncoder();
    shooterBas.setIdleMode(IdleMode.kCoast);
    shooterHaut.setIdleMode(IdleMode.kCoast);
    shooterBas.setSmartCurrentLimit(30);
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
    m_led = new AddressableLED(0);

    // Reuse buffer
    m_ledBuffer = new AddressableLEDBuffer(42);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void setLED(int r, int g, int b) {
    if (r != currentR || g != currentG || b != currentB) {
      currentR = r;
      currentG = g;
      currentB = b;
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, r, g, b);
      }
    }

    m_led.setData(m_ledBuffer);
  }

  public void shooterPower(double powerDroit, double powerGauche) {
    shooterHaut.set(powerGauche);
    shooterBas.set(-powerDroit);
  }

  public void pusherPower(double power) {
    pusher.set(power);
  }

  public shooterPossibleState getShooterState() {
    return shooterState;
  }

  public void setShooterState(shooterPossibleState shooterState) {
    this.shooterState = shooterState;
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
    if (getVelocityBas() >= 4500 && getVelocityHaut() >= 4500) {
      setLED(0, 255, 0);
    } else {
      setLED(255, 0, 0);
    }
    SmartDashboard.putNumber("velocityBas", getVelocityBas());
    SmartDashboard.putNumber("velocityHaut", getVelocityHaut());

    // This method will be called once per scheduler run
  }
}
