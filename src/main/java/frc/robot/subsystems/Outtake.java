package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Outtake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.intakeOuttake.outtakeMotorID, MotorType.kBrushless);
    public Outtake(){
        
    }
 public void setMotors(double power) {
    motor.set(power);
    }
}