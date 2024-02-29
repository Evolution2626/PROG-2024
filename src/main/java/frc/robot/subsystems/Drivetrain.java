// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM;

public class Drivetrain extends SubsystemBase {

  private DoubleSolenoid piston;
  private CANSparkMax avantgauche;
  private CANSparkMax avantdroit;
  private CANSparkMax arrieregauche;
  private CANSparkMax arrieredroit;

  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private MecanumDrive m_robotDrive;
   enum possibleDriveState {
    MECANUM,
    DRIVETANK
  }
  private possibleDriveState driveState = possibleDriveState.DRIVETANK;


  public Drivetrain() {
    piston =
        new DoubleSolenoid(49, PneumaticsModuleType.REVPH, PCM.PISTON_FORWARD, PCM.PISTON_REVERSE);

    avantgauche = new CANSparkMax(CAN.DeviceNumberAvantGauche, MotorType.kBrushless);
    avantdroit = new CANSparkMax(CAN.DeviceNumberAvantDroit, MotorType.kBrushless);
    arrieredroit = new CANSparkMax(CAN.DeviceNumberArriereDroit, MotorType.kBrushless);
    arrieregauche = new CANSparkMax(CAN.DeviceNumberArriereGauche, MotorType.kBrushless);

    avantgauche.setSmartCurrentLimit(40, 30);
    avantdroit.setSmartCurrentLimit(40, 30);
    arrieredroit.setSmartCurrentLimit(40, 30);
    arrieregauche.setSmartCurrentLimit(40, 30);

    avantgauche.setOpenLoopRampRate(0.05);
    avantdroit.setOpenLoopRampRate(0.05);
    arrieredroit.setOpenLoopRampRate(0.05);
    arrieregauche.setOpenLoopRampRate(0.05);

    avantdroit.setInverted(true);
    avantgauche.setInverted(false);
    arrieredroit.setInverted(true);
    arrieregauche.setInverted(false);

    avantdroit.setIdleMode(IdleMode.kBrake);
    avantgauche.setIdleMode(IdleMode.kBrake);
    arrieredroit.setIdleMode(IdleMode.kBrake);
    arrieregauche.setIdleMode(IdleMode.kBrake);

    m_robotDrive = new MecanumDrive(avantgauche, arrieregauche, avantdroit, arrieredroit);
    m_robotDrive.setSafetyEnabled(false);
    resetEncoder();
    avantdroit.burnFlash();
    avantgauche.burnFlash();
    arrieredroit.burnFlash();
    arrieregauche.burnFlash();
  }

  public double getGyroAngle() {
    return Math.abs(gyro.getAngle(IMUAxis.kZ));
  }

  public double getGyroAngleRaw() {
    return gyro.getAngle(IMUAxis.kZ);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getGyroAngleRaw());
  }

  public void resetGyroAngle() {
    gyro.reset();
  }


  public void switchMode(possibleDriveState driveState){
    this.driveState = driveState;
  }

  public double[] getEncoder() {

    double[] encoderValue = {
      avantdroit.getEncoder().getPosition(),
      avantgauche.getEncoder().getPosition(),
      arrieredroit.getEncoder().getPosition(),
      arrieregauche.getEncoder().getPosition()
    };
    return encoderValue;
  }

  

  public double[] getAverageEncoder() {
    double[] value = {
      ((avantdroit.getEncoder().getPosition() + arrieredroit.getEncoder().getPosition()) / 2),
      ((avantdroit.getEncoder().getPosition() + arrieredroit.getEncoder().getPosition()) / 2)
    };
    return value;
  }

  public possibleDriveState getCurrentDrivetrain() {
    return driveState;
  }

  public void drive(
      double rightX,
      double rightY,
      double leftX,
      double leftY,
      double leftTrigger,
      double rightTrigger) {
    if (driveState == possibleDriveState.DRIVETANK) {
      driveTank(Math.pow(rightY, 3), Math.pow(leftY, 3));

    } else {
      if (leftTrigger > 0) {
        m_robotDrive.driveCartesian(0, -Math.pow(leftTrigger, 3) / 2, 0);
      } else if (rightTrigger > 0) {
        m_robotDrive.driveCartesian(0, Math.pow(rightTrigger, 3) / 2, 0);
      } else {
        m_robotDrive.driveCartesian(
            Math.pow(leftY, 3), -Math.pow(leftX, 3), -Math.pow(rightX, 3) / 2);
      }
    }
  }

  public void driveAllMotor(double speed) {
    avantdroit.set(speed);
    avantgauche.set(speed);
    arrieredroit.set(speed);
    arrieregauche.set(speed);
  }

  public void resetEncoder() {
    avantdroit.getEncoder().setPosition(0);
    avantgauche.getEncoder().setPosition(0);
    arrieredroit.getEncoder().setPosition(0);
    arrieregauche.getEncoder().setPosition(0);
  }

  public void driveOneMotor(double name, double speed) {
    switch (id) {
      case "br":
        arrieredroit.set(speed);
        break;
      case "bl":
        arrieregauche.set(speed);
        break;
      case "fr":
        avantdroit.set(speed);
        break;
      case "fl":
        avantgauche.set(speed);
        break;
    }
  }

  public void driveRotation(double speed) {
    arrieredroit.set(-speed);

    arrieregauche.set(speed);

    avantdroit.set(-speed);

    avantgauche.set(speed);
  }

  public void driveTank(double joystickDroit, double joystickGauche) {
    avantdroit.set(joystickDroit);
    avantgauche.set(joystickGauche);
    arrieregauche.set(joystickGauche);
    arrieredroit.set(joystickDroit);
  }

  @Override
  public void periodic() {
    if(driveState == possibleDriveState.DRIVETANK){
      piston.set(DoubleSolenoid.Value.kReverse);
    }
    else if(driveState == possibleDriveState.MECANUM){
      piston.set(DoubleSolenoid.Value.kForward);
    }
    // This method will be called once per scheduler
    SmartDashboard.putNumber("Gyro", gyro.getAngle(IMUAxis.kZ));
    if (isTankDrive == true) {
      SmartDashboard.putString("Mode", "drivetank");
    } else {
      SmartDashboard.putString("Mode", "mecanum");
    }

    SmartDashboard.putNumber("FR", avantdroit.getEncoder().getPosition());
    SmartDashboard.putNumber("FL", avantgauche.getEncoder().getPosition());
    SmartDashboard.putNumber("BL", arrieredroit.getEncoder().getPosition());
    SmartDashboard.putNumber("BR", arrieregauche.getEncoder().getPosition());
  }
}
