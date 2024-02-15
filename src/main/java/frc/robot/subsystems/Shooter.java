// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterGauche;

  private CANSparkMax shooterDroit;
 

  private RelativeEncoder shooterDroitEncoder;
  private RelativeEncoder shooterGaucheEncoder;

  public Shooter() {
    OperatorConstants deviceNumber = new OperatorConstants();
    shooterGauche = new CANSparkMax(deviceNumber.DeviceNumberShooterGauche, MotorType.kBrushless);
    shooterDroit = new CANSparkMax(deviceNumber.DeviceNumberShooterDroit, MotorType.kBrushless);
    
    shooterGauche.setInverted(false);
    shooterDroit.setInverted(false);
   
    shooterDroitEncoder = shooterDroit.getEncoder();
    shooterGaucheEncoder = shooterGauche.getEncoder();
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

  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
