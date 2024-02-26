package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
     // private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.kMotorPort, MotorType.kBrushless);
     public final DigitalInput m_sensor = new DigitalInput(Constants.IntakeConstants.kSensorPort);

     public IntakeSubsystem(){
        
     }

     public void startIntake() {
          System.out.println("starting intake");
          // intakeMotor.set(Constants.IntakeConstants.kSpeed);
     }

     public void stopIntake() {
          System.out.println("stopping intake");
          // intakeMotor.set(0);
     }

     @Override
     public void periodic(){
     }
}
