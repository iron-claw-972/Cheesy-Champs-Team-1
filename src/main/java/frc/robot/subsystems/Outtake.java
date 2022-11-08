package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Outtake extends SubsystemBase {
    CANSparkMax da1motor = new CANSparkMax(-1, MotorType.kBrushless);
    
}
