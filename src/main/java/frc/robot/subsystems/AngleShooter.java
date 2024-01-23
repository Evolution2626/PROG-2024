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

public class AngleShooter extends SubsystemBase {
  private CANSparkMax shooterAngle;
  private RelativeEncoder shooterAngleEncoder;
  private PIDController pidControllerShooterAngle = new PIDController(0.1, 0.1, 0);

  private double shooterAngleEncoderZero = 0.0;



  /** Creates a new AngleShooter. */
  public AngleShooter() {
        OperatorConstants deviceNumber = new OperatorConstants();

        shooterAngle = new CANSparkMax(deviceNumber.DeviceNumberShooterAngle, MotorType.kBrushless);

        shooterAngle.setInverted(false);

        shooterAngleEncoder = shooterAngle.getAlternateEncoder(42);


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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
