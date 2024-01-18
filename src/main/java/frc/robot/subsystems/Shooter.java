// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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


  }

  public void shoot(double power) {
    shooterGauche.set(power);
    shooterDroit.set(power);
  }
  public double[] getEncoder() {

    double[] encoderValue = {
      shooterGaucheEncoder.getPosition(),
      shooterDroitEncoder.getPosition()
     
    };
    return encoderValue;
  }
  public double getRPM(){
    double startposition = shooterGaucheEncoder.getPosition();
    while(shooterGaucheEncoder.getPosition()< startposition+0.058 || shooterGaucheEncoder.getPosition() > startposition+0.058){

      /* TODO: Regle cela (manque de temps)
      
      i = i + 1
      wait(10)
      i * 10 * 100 * 60
      
      */


    }

    //TODO find a way to get speed with rpm
    return 1.0;
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
