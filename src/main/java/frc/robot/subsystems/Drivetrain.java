// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PCM;

public class Drivetrain extends SubsystemBase {
  OperatorConstants deviceNumber = new OperatorConstants();
  PCM pcm = new PCM();

  private DoubleSolenoid piston;
  private CANSparkMax avantgauche;
  private CANSparkMax avantdroit;
  private CANSparkMax arrieregauche;
  private CANSparkMax arrieredroit;
  private RelativeEncoder avantGaucheEncoder;
  private RelativeEncoder avantDroitEncoder;
  private RelativeEncoder arriereGaucheEncoder;
  private RelativeEncoder arriereDroitEncoder;
  public boolean isTankDrive;
  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private MecanumDrive m_robotDrive;

  /** Creates a new TankDrivetrain. */
  public Drivetrain() {
    // ADIS16470_IMU gyro = new ADIS16470_IMU();

    piston =
        new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, pcm.PISTON_FORWARD, pcm.PISTON_REVERSE);

    avantgauche = new CANSparkMax(deviceNumber.DeviceNumberAvantGauche, MotorType.kBrushless);
    avantdroit = new CANSparkMax(deviceNumber.DeviceNumberAvantDroit, MotorType.kBrushless);
    arrieredroit = new CANSparkMax(deviceNumber.DeviceNumberArriereDroit, MotorType.kBrushless);
    arrieregauche = new CANSparkMax(deviceNumber.DeviceNumberArriereGauche, MotorType.kBrushless);

    avantdroit.setInverted(true);
    avantgauche.setInverted(false);
    arrieredroit.setInverted(true);
    arrieregauche.setInverted(false);
    m_robotDrive = new MecanumDrive(avantgauche, arrieregauche, avantdroit, arrieredroit);

    avantGaucheEncoder = avantgauche.getEncoder();
    avantDroitEncoder = avantdroit.getEncoder();
    arriereDroitEncoder = arrieredroit.getEncoder();
    arriereGaucheEncoder = arrieregauche.getEncoder();
    m_robotDrive.setSafetyEnabled(false);
  }

  public double getGyroAngle() {
    return Math.abs(gyro.getAngle(IMUAxis.kZ));
  }

  public double getGyroAngleRaw() {
    return gyro.getAngle(IMUAxis.kZ);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ));
  }

  public void resetGyroAngle() {
    gyro.reset();
  }

  public void ActivateDrivetank() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void ActivateMecanum() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public double[] getEncoder() {

    double[] encoderValue = {
      avantDroitEncoder.getPosition(),
      avantGaucheEncoder.getPosition(),
      arriereDroitEncoder.getPosition(),
      arriereGaucheEncoder.getPosition()
    };
    return encoderValue;
  }

  public void setDriveMode(boolean isTankDrive) {
    this.isTankDrive = isTankDrive;
  }

  public void drive(
      double rightX,
      double rightY,
      double leftX,
      double leftY,
      double leftTrigger,
      double rightTrigger) {
    if (isTankDrive == true) {
      driveTank(Math.pow(rightY, 3), Math.pow(leftY, 3));

      // m_robotDrive.driveCartesian(Math.pow(leftY, 3), 0, -Math.pow(rightX, 3));
    } else {
      if (leftTrigger > 0) {
        m_robotDrive.driveCartesian(0, -Math.pow(leftTrigger, 3) / 2, 0);
      } else if (rightTrigger > 0) {
        m_robotDrive.driveCartesian(0, Math.pow(rightTrigger, 3) / 2, 0);
      } else {
        m_robotDrive.driveCartesian(
            Math.pow(leftY, 3), -Math.pow(leftX, 3), -Math.pow(rightX, 3), getRotation2d());
      }
    }
  }

  public void driveOneMotor(int id, double speed) {
    switch (id) {
      case 3:
        arrieredroit.set(speed);
        break;
      case 4:
        arrieregauche.set(speed);
        break;
      case 0:
        avantdroit.set(speed);
        break;
      case 1:
        avantgauche.set(speed);
        break;
    }
  }

  public void driveTank(double joystickDroit, double joystickGauche) {
    avantdroit.set(joystickDroit);
    avantgauche.set(joystickGauche);
    arrieregauche.set(joystickGauche);
    arrieredroit.set(joystickDroit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    SmartDashboard.putNumber("Gyro", gyro.getAngle(IMUAxis.kZ));
    if (isTankDrive == true) {
      SmartDashboard.putString("Mode", "drivetank");
    } else {
      SmartDashboard.putString("Mode", "mecanum");
    }
  }
}
