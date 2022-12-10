package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    //CANSparkMax motor = new CANSparkMax(Constants.intakeOuttake.intakeMotorID, MotorType.kBrushless);
    public Intake() {
       // motor.setIdleMode(IdleMode.kBrake);
    }
 public void setMotors(double power) {
    //motor.set(power);
    }
}