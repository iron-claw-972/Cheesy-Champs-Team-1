package lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Constants;

public class Motors {

  private static double kVoltageCompensation = Constants.kMaxVoltage; 

  /**
   * Create a configured TalonSRX
   * 
   * @param id the ID of the TalonSRX
   * @param continuousCurrentLimit the continuous current limit to set in amps (A)
   * @param peakCurrentLimit the peak current limit to set in amps (A)
   * @param peakCurrentDuration the peak current limit duration to set in milliseconds (ms)
   * 
   * @return a fully configured TalonSRX object
   */
  public static WPI_TalonSRX createTalonSRX(
    int id,
    NeutralMode neutralMode,
    boolean enableVoltageCompensation,
    boolean enableCurrentLimit,
    int continuousCurrentLimit,
    int peakCurrentLimit,
    int peakCurrentDuration
  ) {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = continuousCurrentLimit;
    config.peakCurrentLimit = peakCurrentLimit;
    config.peakCurrentDuration = peakCurrentDuration;
    config.voltageCompSaturation = kVoltageCompensation;

    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableCurrentLimit(enableCurrentLimit);
    talon.enableVoltageCompensation(enableVoltageCompensation);
    talon.setNeutralMode(neutralMode);

    return talon;
  }

  public static WPI_TalonSRX createTalonSRX(int id, NeutralMode neutralMode) {
    return createTalonSRX(id, NeutralMode.Brake, true, false, 38, 45, 125);
  }


  /**
   * Create a configured CANSparkMax
   * @param id the ID of the Spark MAX
   * @param motorType the type of motor the Spark MAX is connected to 
   * @param idleMode the idle mode of the motor (coast or brake)
   * @param stallLimit the current limit to set at stall
   * @return a fully configured CANSparkMAX
   */
  public static CANSparkMax createSparkMAX(int id, MotorType motorType, IdleMode idleMode, int stallLimit) {
    CANSparkMax sparkMAX = new CANSparkMax(id, motorType);
    sparkMAX.restoreFactoryDefaults();
    sparkMAX.enableVoltageCompensation(kVoltageCompensation);
    sparkMAX.setSmartCurrentLimit(stallLimit);
    sparkMAX.setIdleMode(idleMode);

    sparkMAX.burnFlash();
    return sparkMAX;
  }

  public static CANSparkMax createSparkMAX(int id, MotorType motorType, IdleMode idleMode) {
    return createSparkMAX(id, motorType, idleMode, 60);
  }

  public static CANSparkMax createSparkMAX(int id, MotorType motorType) {
    return createSparkMAX(id, motorType, IdleMode.kBrake);
  }


  /*
  * Talon Stator / Supply Limits explanation 
  * Supply current is current that’s being drawn at the input bus voltage. Stator current is current that’s being drawn by the motor.
  * Supply limiting (supported by Talon SRX and FX) is useful for preventing breakers from tripping in the PDP.
  * Stator limiting (supported by Talon FX) is useful for limiting acceleration/heat.
  */

  /**
  * Create a configured TalonFX.
  * @param id the ID of the motor
  * @param neutralMode Whether the motor is in coast or brake mode
  * @param enableVoltageCompensation Whether to enable Voltage Compensation
  * @param enableSupplyCurrentLimit whether to enable supply current limiting
  * @param supplyCurrentLimit the regular current to return to after the trigger
  * @param supplyTriggerThreshold The current at which the trigger will activate
  * @param supplyTriggerDuration The amount of time the current must be above the trigger current to reduce current
  * @return a fully configured TalonFX
  */
  public static WPI_TalonFX createTalonFX(
    int id,
    NeutralMode neutralMode,
    boolean enableVoltageCompensation,
    boolean enableSupplyCurrentLimit,
    double supplyCurrentLimit,
    double supplyTriggerThreshold,
    double supplyTriggerDuration
  ) {

    if (id == -1) return null;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 100, 100, 0);
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(enableSupplyCurrentLimit, supplyCurrentLimit, supplyTriggerThreshold, supplyTriggerDuration);
    config.voltageCompSaturation = kVoltageCompensation;

    WPI_TalonFX talon = new WPI_TalonFX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(enableVoltageCompensation);
    talon.setNeutralMode(neutralMode);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;
  }

  public static WPI_TalonFX createTalonFX(int id, NeutralMode neutralMode, boolean enableSupplyCurrentLimit, double supplyCurrentLimit, double supplyTriggerThreshold, double supplyTriggerDuration) {
    return createTalonFX(id, neutralMode, false, enableSupplyCurrentLimit, supplyCurrentLimit, supplyTriggerThreshold, supplyTriggerDuration);
  }

  public static WPI_TalonFX createTalonFX(int id, NeutralMode neutralMode) {
    return createTalonFX(id, neutralMode, false, 40, 55, 3);
  }

}