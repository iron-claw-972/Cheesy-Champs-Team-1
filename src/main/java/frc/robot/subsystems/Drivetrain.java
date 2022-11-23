/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.util.MotorFactory;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;

  private final WPI_TalonFX m_rightMotor1;

  private final DifferentialDrive m_drive;

  private AHRS ahrs = new AHRS(SerialPort.Port.kMXP); 
 
  private final DifferentialDriveOdometry m_odometry;
  
  public Drivetrain() {
    m_leftMotor1 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor1);
  
    m_rightMotor1 = MotorFactory.createTalonFX(Constants.drive.kRightMotor1);

    m_drive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    //setting distance per pulse
    m_leftMotor1.configSelectedFeedbackCoefficient(1/2048*12/62*Math.PI*Units.inchesToMeters(4));
    m_rightMotor1.configSelectedFeedbackCoefficient(1/2048*12/62*Math.PI*Units.inchesToMeters(4));
  }

  /**
   * Drives the robot using tank drive controls
   * Tank drive is slightly easier to code but less intuitive to control, so this is here as an example for now
   * @param leftPower the commanded power to the left motors
   * @param rightPower the commanded power to the right motors
   */
  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(ControlMode.PercentOutput, leftPower);
    m_rightMotor1.set(ControlMode.PercentOutput, rightPower);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */
  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor1.set(ControlMode.PercentOutput, throttle + turn);
    m_rightMotor1.set(ControlMode.PercentOutput, throttle - turn);
}
  public double getLeftEncoderValues() {
  return m_leftMotor1.getSelectedSensorPosition();
}
  public double getRightEncoderValues() {
  return m_rightMotor1.getSelectedSensorPosition();
  } 
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        ahrs.getRotation2d(), m_leftMotor1.getSelectedSensorPosition(), m_rightMotor1.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.setVoltage(leftVolts);
    m_rightMotor1.setVoltage(rightVolts);
  }
}

