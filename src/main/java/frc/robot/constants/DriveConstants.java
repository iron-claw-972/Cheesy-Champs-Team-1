package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {

  public final int kLeftMotor1 = -1;
  public final int kLeftMotor2 = -1;
  public final int kRightMotor1 = -1;
  public final int kRightMotor2 = -1;
//Pathweaver stuff below all set to 0 for now 
  public static final double ksVolts = 0;
  public static final double kvVoltSecondsPerMeter = 0;
  public static final double kaVoltSecondsSquaredPerMeter = 0;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 0;
  
  public static final double kTrackwidthMeters = 0.635;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

   public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
