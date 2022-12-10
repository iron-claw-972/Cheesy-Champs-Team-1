package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import lib.Motors;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;

  private final TalonEncoder m_leftEncoder;
  private final TalonEncoder m_rightEncoder;

  private final PhoenixMotorControllerGroup m_leftMotors;
  private final PhoenixMotorControllerGroup m_rightMotors;

  private final DifferentialDrive m_dDrive;

  private final AHRS m_gyro;

  private final PIDController m_leftDrivePID = new PIDController(
    Constants.drive.kLeftDriveP,
    Constants.drive.kLeftDriveI,
    Constants.drive.kLeftDriveD
  );
  private final PIDController m_rightDrivePID = new PIDController(
    Constants.drive.kRightDriveP,
    Constants.drive.kRightDriveI,
    Constants.drive.kRightDriveD
  );

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.drive.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(Constants.drive.kSLinear, Constants.drive.kVLinear);
  private final RamseteController ramseteController = new RamseteController(Constants.auto.kRamseteB, Constants.auto.kRamseteZeta); 

  private final Field2d m_field = new Field2d();

 
  public Drivetrain() {
    this(
      Motors.createTalonFX(Constants.drive.kLeftMotorId, NeutralMode.Brake, true, 30, 40, 1),
      Motors.createTalonFX(Constants.drive.kRightMotorId, NeutralMode.Brake, true, 30, 40, 1)
    );
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, AHRS gyro) {
    this(leftMotor1, rightMotor1, gyro, new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, DifferentialDrive dDrive) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), dDrive);
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, AHRS gyro, DifferentialDrive dDrive) {
    // Motor setup
    m_leftMotor1 = leftMotor1;
    m_rightMotor1 = rightMotor1;

    m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1);
    m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1);


    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(false);

    // Encoder setup
    m_leftEncoder = new TalonEncoder(m_leftMotor1);
    m_rightEncoder = new TalonEncoder(m_rightMotor1);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);

    resetEncoders();

    // Gyro setup
    m_gyro = gyro;
    resetGyro();

    // Drivetrain setup
    m_dDrive = dDrive;
    SmartDashboard.putData(m_dDrive);

    // Odometry setup
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // Place field on Shuffleboard
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
  }

  public RamseteController getRamseteController(){
    return ramseteController; 
  }

  public SimpleMotorFeedforward getDriveFF() {
    return m_driveFF;
  }

  public PIDController getLeftDrivePID() {
    return m_leftDrivePID;
  }

  public PIDController getRightDrivePID() {
    return m_rightDrivePID;
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics; 
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculate FF
    final double leftFeedforward = m_driveFF.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_driveFF.calculate(speeds.rightMetersPerSecond);

    // Calculate PID
    final double leftOutput = m_leftDrivePID.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightDrivePID.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    // Output PID+FF
    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
    m_dDrive.feed(); 
  }

  public void feedForwardDrive(double throttle, double turn) {
    // Convert drivetrain throttle and turn to left and right wheel speeds
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(throttle, 0.0, turn));

    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry() {
    // Upate robot pose (x, y, theta)
    m_odometry.update(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public double getPoseX() {
    return getPose().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }

  public double getPoseRotation() {
    return getPose().getRotation().getDegrees();
  }

  public void arcadeDrive(double throttle, double turn){
    m_dDrive.arcadeDrive(throttle, turn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_dDrive.feed();
  }

  public double getDriveSpeed(){
    return (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2;
  }

  public double getGyroRotation(){
    return m_gyro.getRotation2d().getDegrees();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
}