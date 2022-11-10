package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(3);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(3);
  

  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {

    // TODO 3.1: Change the DoNothing() command to one of your commands
    driver.get(Button.A).whenPressed(new DoNothing());

    // TODO 3.3: Write some more triggers for your commands! Group your commands and functions using at least one of each of these: ParallelCommandGroup, SequentialCommandGroup, ConditionalCommand, PrintCommand, WaitUntilCommand
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

  public static double getThrottleValue() {
    
    return slewThrottle.calculate(Functions.deadband(0.05, getRawThrottleValue()));
    
  }
  public static double getTurnValue() {
    return slewTurn.calculate(Functions.deadband(0.05, getRawTurnValue()));

  }


}