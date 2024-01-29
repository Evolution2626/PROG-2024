// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PCM;

public class ClimberInAnBox extends SubsystemBase {
  private Talon climberDroit;
  private Talon climberGauche;
  private DoubleSolenoid piston;

  /** Creates a new ClimberInAnBox. */
  public ClimberInAnBox() {
    OperatorConstants deviceNumber = new OperatorConstants();
    PCM pcm = new PCM();
    climberDroit = new Talon(deviceNumber.DeviceNumberClimberDroit);
    climberGauche = new Talon(deviceNumber.DeviceNumberClimberGauche);

    climberDroit.setInverted(false);
    climberGauche.setInverted(false);
    piston =
        new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            pcm.PISTON_CLIMBER_FORWARD,
            pcm.PISTON_CLIMBER_REVERSE);
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void climb(double activated) {
    climberDroit.set(activated);
    climberGauche.set(activated);
  }

  public void activateRatchet() {

    piston.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
