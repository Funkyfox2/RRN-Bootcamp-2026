package frc.robot.examples;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.*;


import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
 
    /*
    "Imports" subsystems that you make in the subsystem folder to be used for controller actions.
    Make sure you actually import the subsystem in the same manner as we do with the SwerveSubsystem above.
    */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
    

    // Sets the Joystick/Physical Driver Station ports, change port order in Driver Station to the numbers below.
    private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort); // 0

    // Sends a dropdown for us to choose an auto in the Dashboard.
    private final SendableChooser<Command> autoChooser;
    
    // A command to be assigned later that will aim the robot to a point on the field.
    private Command driveAim;
    private Command driveToPose;
    
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
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        
        
        // The on field locations of the hub from the right side of the blue alliance.
        Pose2d redHubLocation = new Pose2d(11.920,4.021,Rotation2d.kZero);
        Pose2d blueHubLocation = new Pose2d(4.612,4.021,Rotation2d.kZero);
        
        // Supplies the following hubAim with the location that the robot needs to be facing. 
        Supplier<Pose2d> hubSupplier = () -> {
            // Picks the hub location based on what alliance you're on.
            Pose2d robotAim = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) ? blueHubLocation : redHubLocation;
            // Returns a pose that we can aim to.
            return robotAim;
        };

        // Takes our AngularVelocity drive path and overwrites the rotation component to always face towards the correct hub.
        // Also flips it 180 degrees to account for the side of the robot the shooter is on.  
        SwerveInputStream hubAim = driveAngularVelocity.copy().aim(hubSupplier).aimHeadingOffset(Rotation2d.fromDegrees(180)).aimHeadingOffset(true).aimWhile(true);
        // Gives us a command that we can bind to the controller.
        driveAim = swerveSubsystem.driveFieldOriented(hubAim);

        // The locations of the spots to climb on the field.
        Pose2d blueClimbLocation = new Pose2d(1.504,3.719,Rotation2d.kZero);
        Pose2d redClimbLocation = new Pose2d(15.020,3.719,Rotation2d.kZero);

        // Supplies the following driveToPose command with the location that the robot should go to.
        Supplier<Pose2d> climbSupplier = () -> {
            // Picks the hub location based on what alliance you're on.
            Pose2d climbLocation = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) ? blueClimbLocation : redClimbLocation;
            return climbLocation;
        };
        
        // Creates a command that we can bind to the controller.
        driveToPose = swerveSubsystem.driveToPose(climbSupplier.get());
        
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
        driverJoystick.x().whileTrue(driveToPose);
        driverJoystick.y().whileTrue(driveAim);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
    
}
