// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.TriggerEvent;

public class RobotContainer {

    // subsystems
    // private Elevator climber;
    private Intake intake;
    private Trigger trigger;
    private Turret turret;

    // drivetrain
    private static final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(field);

    public static Field2d getField() {
        return field;
    }
    
    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    private final SendableChooser<Command> autoChooser;


    private final double MaxSpeed = Constants.Drive.MAX_SPEED_MPS / Constants.Drive.SPEED_DIVISOR;
    private final double MaxAngularRate = Constants.Drive.MAX_ANGULAR_RATE_RPS;

    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DEADBAND_PERCENT).withRotationalDeadband(MaxAngularRate * Constants.Drive.DEADBAND_PERCENT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    // IO devices
    private final CommandXboxController Player1 = new CommandXboxController(0);
    private final CommandXboxController Player2 = new CommandXboxController(1);
    private final GenericHID Player3 = new GenericHID(3);
    private final boolean enableStadiaDrive = true;

    /*
     * Stadia Controller Map
     * a - 1
     * b - 2
     * x - 3
     * y - 4
     * left bumper - 5
     * right bumper - 6
     * left stick - 7
     * right stick - 8
     * select - 9
     * start - 10
     * stadia button - 11
     * right trigger - 12
     * left trigger - 13
     */

    JoystickButton buttonA = new JoystickButton(Player3, 1);
    JoystickButton buttonB = new JoystickButton(Player3, 2);
    JoystickButton buttonX = new JoystickButton(Player3, 3);
    JoystickButton buttonY = new JoystickButton(Player3, 4);
    JoystickButton buttonLeftBumper = new JoystickButton(Player3, 5);
    JoystickButton buttonRightBumper = new JoystickButton(Player3, 6);
    JoystickButton buttonLeftStick = new JoystickButton(Player3, 7);
    JoystickButton buttonRightStick = new JoystickButton(Player3, 8);
    JoystickButton buttonSelect = new JoystickButton(Player3, 9);
    JoystickButton buttonStart = new JoystickButton(Player3, 10);
    JoystickButton buttonStadia = new JoystickButton(Player3, 11);
    JoystickButton buttonRightTrigger = new JoystickButton(Player3, 12);
    JoystickButton buttonLeftTrigger = new JoystickButton(Player3, 13);

    

    public RobotContainer() {

      
        // climber = new Elevator();
        intake = new Intake();
        trigger = new Trigger();
        turret = new Turret(drivetrain);
        
        // NamedCommands must be registered before AutoBuilder loads autos
        mapEventsToCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Ribakov1");
        //autoChooser = AutoCommands.buildAutoChooser(this, drivetrain, drive, MaxSpeed, turret, trigger, intake);

        //autoChooser = new SendableChooser<>();

        SmartDashboard.putData("Field", field);

        new EventTrigger("intake").whileTrue(new RunIntake(intake));
        new EventTrigger("deploy intake").whileTrue(new ManualDeploy(intake, MaxSpeed).withTimeout(.08));
        new EventTrigger("kick up").whileTrue(new Shoot(turret, trigger));
        // Do all other initialization
        configureBindings();
        SmartDashboard.putNumber("velocity", 1.0);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void mapEventsToCommands()
    {
        // replace null with command instance
        // do this for all commands
        NamedCommands.registerCommand("intake", new RunIntake(intake).withTimeout(3.6));
        NamedCommands.registerCommand("shoot",  new SpinToDistanceSpeed(turret));
        NamedCommands.registerCommand("shoot distance", new SequentialCommandGroup(
            new SpinToSpeed(turret, Constants.Turret.SPEED_CLOSE_RPS),
            new ParallelCommandGroup(
                new DeployJumpCommand(intake),
                new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_CLOSE_RPS),
                new Shoot(turret, trigger)
            )
        ).withTimeout(7));
        NamedCommands.registerCommand("kickup", new Shoot(turret, trigger).withTimeout(2.0));
        NamedCommands.registerCommand("auto shoot", new AutoShoot(turret, trigger));
        NamedCommands.registerCommand("auto turret", new AutoTurret(turret, trigger, drivetrain).withTimeout(5.0));
        NamedCommands.registerCommand("stop intake",  new StopIntake(intake));
        NamedCommands.registerCommand("auto deploy", new ManualDeploy(intake, MaxSpeed).withTimeout(.08));
        
    }

    //field orient
    public void seedFieldOrient() {
        drivetrain.seedFieldCentric();
    }

    public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    private void configureDrivetrain() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void startTeleop()
    {
        turret.stopRotator();
        turret.stopShooter();
        intake.stopWheels();
        intake.stopDeploy();
        //seedFieldOrient();
    }

    //player 1 and 2 stick getters.

    public double getLeftY()
    {
        double player1 = Player1.getLeftY();
        double player2 = Player2.getLeftY();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        if (enableStadiaDrive) {
            double player3 = Player3.getRawAxis(1);
            if (Math.abs(player3) > Math.abs(max)) { max = player3; }
        }
        return max;
    }

    public double getLeftX()
    {
        double player1 = Player1.getLeftX();
        double player2 = Player2.getLeftX();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        if (enableStadiaDrive) {
            double player3 = Player3.getRawAxis(0);
            if (Math.abs(player3) > Math.abs(max)) { max = player3; }
        }
        return max;
    }

    public double getRightX()
    {
        double player1 = Player1.getRightX();
        double player2 = Player2.getRightX();

        double max = Math.abs(player1) > Math.abs(player2) ? player1 : player2;
        if (enableStadiaDrive) {
            double player3 = Player3.getRawAxis(3);
            if (Math.abs(player3) > Math.abs(max)) { max = player3; }
        }
        return max;
    }

    private void configureBindings() {
        configureDrivetrain();
 drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(getLeftY()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive forward with negative Y (forward)
                    .withVelocityY(-(getLeftX()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive left with negative X (left)
                    .withRotationalRate(-(getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    

        
        // drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        Player2.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Player1.y().whileTrue(new AutoTrackGoal(turret));
        //Player2.y().whileTrue(new AutoTrackGoal(turret));

        //Button Map

        Player1.x().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_CLOSE_RPS),
            new Shoot(turret, trigger)
        ));
        Player1.a().whileTrue(new ManualDeploy(intake, Constants.Intake.DEPLOY_MANUAL_SPEED)); // down
        Player1.b().whileTrue(new ManualDeploy(intake, -Constants.Intake.DEPLOY_MANUAL_SPEED)); // up
        Player1.leftBumper().whileTrue(new RunIntake(intake, true)); //true boolean means reverse
        

        Player1.rightTrigger().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_FAR_RPS),
            new Shoot(turret, trigger)
        )); // shoot and kick up, shooter first then kickup

        //Player1.povUp().onTrue(new StopTurretWheels(turret));
        Player1.povUp().whileTrue(new TurretTo(turret, 90));
        Player1.rightBumper().whileTrue(new ReverseShoot(trigger));

        

        //Manual Turret
        Player1.povRight().whileTrue(new ManualTurret(turret, () -> .3)); //left
        Player1.povLeft().whileTrue(new ManualTurret(turret, () -> -.3)); //left
        /*Player1.povRight().onTrue(new InstantCommand(() -> {
            turret.incVoltage();
        })); //left
        Player1.povLeft().onTrue(new InstantCommand(() -> {
            turret.stopRotator();
        })); //left*/

        Player1.leftTrigger().whileTrue(new RunIntake(intake)); // intake in

        //Player 2 controls
        Player2.x().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_CLOSE_RPS),
            new Shoot(turret, trigger)
        ));
        Player2.a().whileTrue(new ManualDeploy(intake, Constants.Intake.DEPLOY_MANUAL_SPEED)); // down
        Player2.b().whileTrue(new ManualDeploy(intake, -Constants.Intake.DEPLOY_MANUAL_SPEED)); // up
        Player2.leftBumper().whileTrue(new RunIntake(intake, true));        

        Player2.rightTrigger().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_FAR_RPS),
            new Shoot(turret, trigger)
        )); // shoot and kick up, shooter first then kickup

        //Player1.povUp().onTrue(new StopTurretWheels(turret));
        Player2.povUp().whileTrue(new TurretTo(turret, 90));
        Player2.rightBumper().whileTrue(new ReverseShoot(trigger));

        

        //Manual Turret
        Player2.povRight().whileTrue(new ManualTurret(turret, () -> .3)); //left
        Player2.povLeft().whileTrue(new ManualTurret(turret, () -> -.3)); //left

        Player2.leftTrigger().whileTrue(new RunIntake(intake)); // intake in



    }   

}
