// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterGauche;
  private CANSparkMax shooterDroit;
  private CANSparkMax pusherGauche;
  private CANSparkMax pusherDroit;
  private CANSparkMax shooterAngle;
  private RelativeEncoder shooterDroitEncoder;
  private RelativeEncoder shooterGaucheEncoder;
   private RelativeEncoder pusherDroitEncoder;
   private RelativeEncoder shooterAngleEncoder;
  private RelativeEncoder pusherGaucheEncoder;
   private PIDController pidControllerDroitRPM = new PIDController(0.1, 0.1, 0);
  private PIDController pidControllerGaucheRPM = new PIDController(0.1, 0.1, 0);
  private PIDController pidControllerShooterAngle = new PIDController(0.1, 0.1, 0);

  private double shooterAngleEncoderZero = 0.0;

  public Shooter() {
    OperatorConstants deviceNumber = new OperatorConstants();
    shooterGauche = new CANSparkMax(deviceNumber.DeviceNumberShooterGauche, MotorType.kBrushless);
    shooterDroit = new CANSparkMax(deviceNumber.DeviceNumberShooterDroit, MotorType.kBrushless);
    pusherGauche = new CANSparkMax(deviceNumber.DeviceNumberPusherGauche, MotorType.kBrushless);
    pusherDroit = new CANSparkMax(deviceNumber.DeviceNumberPusherDroit, MotorType.kBrushless);
    shooterAngle = new CANSparkMax(deviceNumber.DeviceNumberShooterAngle, MotorType.kBrushless);

    shooterGauche.setInverted(false);
    shooterDroit.setInverted(false);
    pusherGauche.setInverted(false);
    pusherDroit.setInverted(false);
    shooterAngle.setInverted(false);
    shooterDroitEncoder = shooterDroit.getAlternateEncoder(42);
    shooterGaucheEncoder = shooterGauche.getAlternateEncoder(42);
    pusherDroitEncoder = pusherDroit.getAlternateEncoder(42);
    pusherGaucheEncoder = pusherGauche.getAlternateEncoder(42);
    shooterAngleEncoder = shooterAngle.getAlternateEncoder(42);
    
  }

  public void shooterPower(double powerDroit, double powerGauche) {
    shooterGauche.set(powerGauche);
    shooterDroit.set(powerDroit);
    
  }
 
  public void goToAngle(double angle){
    double encoderPosition = shooterAngleEncoderZero + (angle*8.57);
    shooterAngle.set(pidControllerShooterAngle.calculate(shooterAngleEncoder.getPosition(), encoderPosition));
  }
  public void resetEncoder(int postion){
    if (postion == 0) {
    shooterAngleEncoderZero = shooterAngleEncoder.getPosition();
    }
    if (postion == 1) {
       shooterAngleEncoderZero = shooterAngleEncoder.getPosition() - 1234567890;//TODO change to right number
    }
  }
    public double getVelocityDroit() {
    return shooterDroitEncoder.getVelocity();
  }

  public double getVelocityGauche() {
    return shooterGaucheEncoder.getVelocity();
  }
 public void pusherPower(double power){
  pusherDroit.set(power);
  pusherGauche.set(power);
 }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
