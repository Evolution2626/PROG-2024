// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PCM;

public class ClimberInAnBox extends SubsystemBase {
  private TalonSRX climberDroit;
  private TalonSRX climberGauche;
  private DoubleSolenoid piston1;
  private DoubleSolenoid piston2;

  /** Creates a new ClimberInAnBox. */
  public ClimberInAnBox() {
    OperatorConstants deviceNumber = new OperatorConstants();
    PCM pcm = new PCM();
    climberDroit = new TalonSRX(deviceNumber.DeviceNumberClimberDroit);
    climberGauche = new TalonSRX(deviceNumber.DeviceNumberClimberGauche);

    climberDroit.setInverted(false);
    climberGauche.setInverted(false);

    piston1 =
        new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            pcm.PISTON_CLIMBER_FORWARD_1,
            pcm.PISTON_CLIMBER_REVERSE_1);
    piston2 =
        new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            pcm.PISTON_CLIMBER_FORWARD_2,
            pcm.PISTON_CLIMBER_REVERSE_2);
    piston1.set(DoubleSolenoid.Value.kReverse);
    piston2.set(DoubleSolenoid.Value.kReverse);
  }

  public void climb(double droit, double gauche) {
    climberDroit.set(ControlMode.PercentOutput, droit);
    climberGauche.set(ControlMode.PercentOutput, gauche);
  }

  public void activateRatchet() {

    piston1.set(DoubleSolenoid.Value.kForward);
    piston2.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
