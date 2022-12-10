package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(1);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(1);

  public static void configureControls() {
    driver.get(Button.B).whenPressed(new InstantCommand( () -> Robot.intake.setMotors(Constants.intakeOuttake.intakeMotorPower) ) );
    driver.get(Button.B).whenReleased(new InstantCommand( () -> Robot.intake.setMotors(0) ) );
    driver.get(Button.A).whenPressed(new InstantCommand( () -> Robot.outtake.setMotors(Constants.intakeOuttake.outtakeMotorPower) ) );
    driver.get(Button.A).whenReleased(new InstantCommand( () -> Robot.outtake.setMotors(0) ) );
 
    // to unJam...
    driver.get(Button.Y).whenPressed(new InstantCommand( () -> Robot.intake.setMotors(Constants.intakeOuttake.unJam) ) );
    driver.get(Button.Y).whenReleased(new InstantCommand( () -> Robot.intake.setMotors(0) ) );
    driver.get(Button.X).whenPressed(new InstantCommand( () -> Robot.outtake.setMotors(Constants.intakeOuttake.unJam) ) );
    driver.get(Button.X).whenReleased(new InstantCommand( () -> Robot.outtake.setMotors(0) ) );

  }

  public static double getRawThrottleValue() {
    return driver.get(Axis.LEFT_Y);

  }

  public static double getRawTurnValue() {
    return driver.get(Axis.RIGHT_X);
  }

  public static double getRawLeft() {
    return driver.get(Axis.LEFT_Y);
  }

  public static double getRawRight() {
    return driver.get(Axis.RIGHT_Y);
  }

  public static double getThrottle() {
    return - slewThrottle.calculate(Functions.deadband(0.05, getRawThrottleValue()*Constants.drive.kMaxDriveSpeed));
    
  }
  public static double getTurn() {
    return - slewTurn.calculate(Functions.deadband(0.05, getRawTurnValue()*Constants.drive.kMaxTurnSpeed));

  }
}
