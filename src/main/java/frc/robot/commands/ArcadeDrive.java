package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.controls.Driver;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  Drivetrain m_drive;

  /**
   * Creates a new ArcadeDrive command. It will continuously run arcade drive on the robot.
   *
   * @param drive The drivetrain subsystem used by this command.
   */
  public ArcadeDrive(Drivetrain drive) {

    // store the drivetrain in the m_drive variable to be accessed elsewhere
    m_drive = drive;

    // add the drivetrain as a requirement so the scheduler 
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO 2.1: Call the arcade drive function written in Drivetrain here. This function is called repeatedly so it will constantly run while the command is being executed 
    m_drive.arcadeDrive(Driver.getThrottleValue(), Driver.getTurnValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}