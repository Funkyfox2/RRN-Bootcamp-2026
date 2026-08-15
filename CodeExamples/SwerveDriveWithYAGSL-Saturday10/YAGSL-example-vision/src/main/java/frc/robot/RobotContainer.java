package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

    // Grabs the SwerveSubsystem to be used for controller actions from the subsystems folder.
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Sets the Joystick/Physical Driver Station ports, change port order in Driver Station to the numbers below.
    private final CommandXboxController driverJoystick = new CommandXboxController(0); // 0

    // Sends a dropdown for us to choose an auto in the Dashboard.
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
         */
        // Change the -1's to 1's to invert joystick directions!
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                        () -> driverJoystick.getLeftY() * -1,
                                                                        () -> driverJoystick.getLeftX() * -1)
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRightX())
                                                                    .deadband(0.05D)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        
        // Set the default command so that the scheduler knows that when nothing else is using the Swerve Subsystem, we want operator control.
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Maps commands to inputs on the controller.
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Sends a dropdown for us to choose an auto in the Dashboard.
        SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  

    private void configureButtonBindings() {
        // Used to set all Button Bindings as the name suggests, excluding moving the robot with the joystick.
        driverJoystick.a().onTrue(new ResetHeading(swerveSubsystem));
        driverJoystick.b().toggleOnTrue(new LockPose(swerveSubsystem));
        driverJoystick.x().whileTrue(null);
        driverJoystick.y().onChange(null);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
    
}
