// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private DoubleSolenoid piston;
  private CANSparkMax avantgauche;
  private CANSparkMax avantdroit;
  private CANSparkMax arrieregauche;
  private CANSparkMax arrieredroit;
  private RelativeEncoder avantGaucheEncoder;
  private RelativeEncoder avantDroitEncoder;
  private RelativeEncoder arriereGaucheEncoder;
  private RelativeEncoder arriereDroitEncoder;
  public boolean isTankDrive = true;
  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private MecanumDrive m_robotDrive;
  private DifferentialDriveOdometry odometry;

  /** Creates a new TankDrivetrain. */
  public Drivetrain() {
    // ADIS16470_IMU gyro = new ADIS16470_IMU();

    piston =
        new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.PCM.PISTON_FORWARD, Constants.PCM.PISTON_REVERSE);

    avantgauche = new CANSparkMax(Constants.OperatorConstants.DeviceNumberAvantGauche, MotorType.kBrushless);
    avantdroit = new CANSparkMax(Constants.OperatorConstants.DeviceNumberAvantDroit, MotorType.kBrushless);
    arrieredroit = new CANSparkMax(Constants.OperatorConstants.DeviceNumberArriereDroit, MotorType.kBrushless);
    arrieregauche = new CANSparkMax(Constants.OperatorConstants.DeviceNumberArriereGauche, MotorType.kBrushless);

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

    avantgauche.setSmartCurrentLimit(30);
    avantdroit.setSmartCurrentLimit(30);
    arrieredroit.setSmartCurrentLimit(30);
    arrieregauche.setSmartCurrentLimit(30);

    avantgauche.setClosedLoopRampRate(0.1);
    avantdroit.setClosedLoopRampRate(0.1);
    arrieredroit.setClosedLoopRampRate(0.1);
    arrieregauche.setClosedLoopRampRate(0.1);

    avantgauche.setOpenLoopRampRate(0.1);
    avantdroit.setOpenLoopRampRate(0.1);
    arrieredroit.setOpenLoopRampRate(0.1);
    arrieregauche.setOpenLoopRampRate(0.1);

    avantDroitEncoder.setPosition(0);
    arriereDroitEncoder.setPosition(0);
    arriereGaucheEncoder.setPosition(0);
    avantGaucheEncoder.setPosition(0);

    avantDroitEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    arriereDroitEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    avantGaucheEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);
    arriereGaucheEncoder.setVelocityConversionFactor(DriveConstants.velocityConversionFactor);

    avantDroitEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);
    arriereDroitEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);
    avantGaucheEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);
    arriereGaucheEncoder.setPositionConversionFactor(DriveConstants.positionConversionFactor);

    odometry =
        new DifferentialDriveOdometry(
            getRotation2d(), avantGaucheEncoder.getPosition(), avantDroitEncoder.getPosition());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoder();
    resetGyroAngle();
    odometry.resetPosition(
        getRotation2d(), avantGaucheEncoder.getPosition(), avantDroitEncoder.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeed() {
    double leftEncoder =
        (arriereGaucheEncoder.getVelocity() + avantGaucheEncoder.getVelocity()) / 2;
    double rightEncoder = (arriereDroitEncoder.getVelocity() + avantDroitEncoder.getVelocity()) / 2;

    return new DifferentialDriveWheelSpeeds(leftEncoder, rightEncoder);
  }
  
  public void resetEncoder(){
    avantDroitEncoder.setPosition(0);
    arriereDroitEncoder.setPosition(0);
    arriereGaucheEncoder.setPosition(0);
    avantGaucheEncoder.setPosition(0);
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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //avantdroit.setVoltage(rightVolts);
    arrieredroit.setVoltage(rightVolts);
    //avantgauche.setVoltage(leftVolts);
    arrieregauche.setVoltage(leftVolts);
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

    SmartDashboard.putNumber("flVel", avantGaucheEncoder.getVelocity());
    SmartDashboard.putNumber("frVel", avantDroitEncoder.getVelocity());
    SmartDashboard.putNumber("blVel", arriereGaucheEncoder.getVelocity());
    SmartDashboard.putNumber("brVel", arriereDroitEncoder.getVelocity());

    SmartDashboard.putNumber("flPos", avantGaucheEncoder.getPosition());
    SmartDashboard.putNumber("frPos", avantDroitEncoder.getPosition());
    SmartDashboard.putNumber("blPos", arriereGaucheEncoder.getPosition());
    SmartDashboard.putNumber("brPos", arriereDroitEncoder.getPosition());

    odometry.update(
        getRotation2d(), avantGaucheEncoder.getPosition(), avantDroitEncoder.getPosition());
  }
}
