// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ClimberInAnBox extends SubsystemBase {
  private CANSparkMax climberDroit;
  private CANSparkMax climberGauche;

  /** Creates a new ClimberInAnBox. */
  public ClimberInAnBox() {
    OperatorConstants deviceNumber = new OperatorConstants();
    climberDroit = new CANSparkMax(deviceNumber.DeviceNumberClimberDroit, MotorType.kBrushless);
    climberGauche = new CANSparkMax(deviceNumber.DeviceNumberClimberGauche, MotorType.kBrushless);

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
