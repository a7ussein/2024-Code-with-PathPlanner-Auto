package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.kMotorPort, MotorType.kBrushless);
    public void shoot(){
        shooterMotor.set(Constants.ShooterConstants.kSpeed);

    }

    public void stop(){
        shooterMotor.set(0);
    }
}
