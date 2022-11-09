package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDCommand extends CommandBase {

  // TODO 4.2: replace ExampleSubsystem with your created ExtraSubsystem, or with Drivetrain. change the name to something better
  Drivetrain m_drive;

  // TODO 4.2: Add a parameter that asks for the setpoint
  public PIDCommand(Drivetrain drive, double setpoint) {
    m_drive = drive;
    m_drive.setSetpoint(setpoint);
    addRequirements(drive);
    // TODO 4.2: replace above ExampleSubsystem with your created ExtraSubsystem, or with Drivetrain.
  }

  public void initialize() {
    // TODO 4.2: zero encoders and reset the PID controller before starting
    m_drive.setEncoderPosition(0);
    m_drive.resetPID();
    m_drive.enablePID();
  }

  public void execute() {
    // TODO 4.2: Make the PID control loop
    m_drive.periodic();
  }

  public void end(boolean interrupted) {
    // TODO 4.2: when the command ends, the motors should stop spinning
    m_drive.disablePID();
  }

  public boolean isFinished() {
    // TODO 4.2: check if the PID is finished though the PID controler
    if (m_drive.m_pid.atSetpoint()) {
      return true;
    }
    return false;
  }
}