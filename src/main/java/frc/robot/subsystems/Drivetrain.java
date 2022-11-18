/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
  
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;
import frc.robot.constants.PIDConstants;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  public Drivetrain() {
    m_leftMotor = MotorFactory.createTalonFX(Constants.drive.kLeftMotor);
    m_rightMotor = MotorFactory.createTalonFX(Constants.drive.kRightMotor);
    m_rightMotor.setInverted(true);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */

  public PIDController m_pid = new PIDController(Constants.pid.kP, Constants.pid.kI, Constants.pid.kD);
  // TODO 4.1: Also add a double for the setpoint, and a boolean for if the PID is enabled.
  double setpoint;
  public boolean pidEnabled = false;

  @Override
  public void periodic() {
    // TODO 4.1: Periodic runs periodically, so we will update the PID here and set the motors. 
    if (pidEnabled) {
      System.out.println(getEncoderPosition(true) * Constants.drive.kSetpointToTicks);
      m_leftMotor.set(m_pid.calculate(getEncoderPosition(true) * Constants.drive.kSetpointToTicks, setpoint));
      m_rightMotor.set(m_pid.calculate(getEncoderPosition(true) * Constants.drive.kSetpointToTicks, setpoint));
    } else {
      m_leftMotor.stopMotor();
      m_rightMotor.stopMotor();
    }
    // If the pid is enabled (a boolean value declared above) then you should set the motors using the pid's calculate() function.
    // Otherwise, it should set the motor power to zero.
    // pid.calculate() takes two values: calculate(processVariable, setpoint). get the process var by getting the encoders,
    // and the setpoint is a variable declared above.

  }
  
  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor.set((throttle - turn) * 0.35);
    m_rightMotor.set((throttle + turn) * 0.35);
  }

  public double getEncoderPosition(boolean isLeft) {
    if (isLeft) return m_leftMotor.getSelectedSensorPosition();
    return m_rightMotor.getSelectedSensorPosition();
  }

  public void setEncoderPosition(double position) {
    m_leftMotor.setSelectedSensorPosition(position);
    m_rightMotor.setSelectedSensorPosition(position);
  }

  public void setSetpoint(double new_setpoint) {
    setpoint = new_setpoint;
  }
  public void enablePID() {
    pidEnabled = true;
  }
  public void disablePID() {
    pidEnabled = false;
  }
  public void resetPID() {
    m_pid.reset();
  }
  
}
