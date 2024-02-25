package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {


    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  
    // Auton stuff
    // private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); //  the ! says "be field relative as long as the button is not pressed."

    // Auton Stuff:
        // autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser",autoChooser);
        // Register Named Commands
       /* NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
        NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());*/ 

        configureButtonBindings();
    }

    private void configureButtonBindings() { // why are these here and not in constants?
        // buttons are apparently numbered 1-9, not 0-8
        // from constants, kDriverFieldOrientedButtonIdx = 5; // lets try using the "L" button for this. 
        new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(()-> swerveSubsystem.zeroHeading()) ); // use the R button for this. this zero's the GYRO??
        // We shouldn't need to reset the turning encoders anynmore. The turning encoders reflect the absolute value of the CANcoders. 
        // unless we skipped a belt or something.  shouldnt hurt though. 
        // new JoystickButton(driverJoystick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetTurningEncoders())); // this is the Y button
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
         // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Mobility");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
}