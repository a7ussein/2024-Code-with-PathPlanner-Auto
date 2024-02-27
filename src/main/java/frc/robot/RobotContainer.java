package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final AutoCommands autoCommands = new AutoCommands(swerveSubsystem, shooterSubsystem, intakeSubsystem);

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

    private final SendableChooser<String> auto_chooser = new SendableChooser<>();
    private static final String kdefaultAuto = "Nothing Auto";
    private static final String kMobility = "Mobility";
    private static final String kOneNotePlusMobility = "One Note + Mobility";
    private static final String kTwoNoteAuto = "Two Note Auto";

    public RobotContainer() {
        // intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
        // Auton stuff
        SmartDashboard.putData("Auto Chooser",auto_chooser);
        auto_chooser.setDefaultOption("Nothing Auto", kdefaultAuto);
        auto_chooser.addOption("Mobility", kMobility);
        auto_chooser.addOption("Mobility + One Note", kOneNotePlusMobility);
        auto_chooser.addOption("Two Note Auto", kTwoNoteAuto);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); //  the ! says "be field relative as long as the button is not pressed."

        configureButtonBindings();
        configureTriggers();

        intakeSubsystem.startIntake();
    }

    private void configureButtonBindings() { // why are these here and not in constants?
        // buttons are apparently numbered 1-9, not 0-8
        // from constants, kDriverFieldOrientedButtonIdx = 5; // lets try using the "L" button for this. 
        new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(()-> swerveSubsystem.zeroHeading()) ); // use the R button for this. this zero's the GYRO??
        
        new JoystickButton(operatorJoystick, OIConstants.kOperatorGreenButton)
            .onTrue(new InstantCommand(()-> shooterSubsystem.shoot()))
            .onFalse(new InstantCommand(()-> shooterSubsystem.stop()));
        // new JoystickButton(driverJoystick, 5).onTrue(new InstantCommand(()-> shooterSubsystem.shoot(0.5)))
        // .onFalse(new InstantCommand(()-> shooterSubsystem.shoot(0)));
        // We shouldn't need to reset the turning encoders anynmore. The turning encoders reflect the absolute value of the CANcoders. 
        // unless we skipped a belt or something.  shouldnt hurt though. 
        // new JoystickButton(driverJoystick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetTurningEncoders())); // this is the Y button
    }

    private void configureTriggers() {
        new Trigger(intakeSubsystem.m_sensor::get)
        .whileTrue(new InstantCommand(() -> intakeSubsystem.startIntake()))
        .whileFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
    }

    public Command getAutonomousCommand() {
        switch (auto_chooser.getSelected()) {
            case kMobility:
                return autoCommands.Mobility();
            case kOneNotePlusMobility:   
                return autoCommands.OneNotePlusMobility();
            case kTwoNoteAuto:
                return autoCommands.twoNoteAuto();
            default:
                return autoCommands.doNothing();
        }
    }
}