package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.WantedIntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {

    RobotContainer robotContainer;
    Drivetrain drivetrain;
    Intake intake;
    Spindexer spindexer;
    Feeder feeder;
    Shooter shooter;

    public Superstructure(RobotContainer container) {
        // Use RobotContainer to gain subsystem access
        robotContainer = container;
        this.drivetrain = container.drivetrain;
        this.intake = container.intake;
        this.spindexer = container.spindexer;
        this.feeder = container.feeder;
        this.shooter = container.shooter;
    }

    public enum WantedSuperState {
        STOP,
        STATIC_SHOOT,
        DISTANCE_SHOOT,
        COLLECT,
        PASS
    }

    public enum CurrentSuperState {
        STOPPING,
        STATIC_SHOOTING,
        DISTANCE_SHOOTING,
        COLLECTING,
        PASSING
    }

    public WantedSuperState wantedSuperState = WantedSuperState.STOP;
    public CurrentSuperState currentSuperState = CurrentSuperState.STOPPING;

    StringPublisher wantedStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/Wanted Super State").publish();
    StringPublisher currentStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/Current Super State").publish();

    @Override
    public void periodic() {
        currentSuperState = handleStateTransition();

        wantedStatePublisher.set(wantedSuperState.toString());
        currentStatePublisher.set(currentSuperState.toString());

        applyState();
    }

    public WantedSuperState setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
        return wantedSuperState;
    }

    public Command setState(WantedSuperState state) {
        return runOnce(() -> setWantedSuperState(state));
    }

    public CurrentSuperState handleStateTransition() {
        switch (wantedSuperState) {
            case STATIC_SHOOT: return CurrentSuperState.STATIC_SHOOTING;
            case DISTANCE_SHOOT: return CurrentSuperState.DISTANCE_SHOOTING;
            case COLLECT: return CurrentSuperState.COLLECTING;
            case PASS: return CurrentSuperState.PASSING;
            default: return CurrentSuperState.STOPPING;
        }
    }

    public void applyState() {
        switch (currentSuperState) {
            case STATIC_SHOOTING: 
                staticShoot();
                break;
            case DISTANCE_SHOOTING: 
                distanceShoot();
                break;
            case COLLECTING:
                collect();
                break;
            case PASSING:
                pass();
                break;
            default:
                stop();
                break;
        }
    }

    public void staticShoot() {
        shooter.setShooterState(ShooterState.STATIC_SHOOT);
        intake.setWantedIntakeState(WantedIntakeState.STOP);
        // When shooter is up to speed, drive feeder and spindexer.
        if (shooter.isAtVelocity() || RobotBase.isSimulation()) {
            feeder.runFeeder(50.0);
            spindexer.runSpindexer(50.0);
        }
    }

    public void distanceShoot() {
        shooter.setShooterState(ShooterState.DISTANCE_SHOOT);
        intake.setWantedIntakeState(WantedIntakeState.STOP);
        // When shooter is up to speed, drive feeder and spindexer.
        if (shooter.isAtVelocity() || RobotBase.isSimulation()) {
            feeder.runFeeder(50.0);
            spindexer.runSpindexer(50.0);
        }
    }

    public void collect() {
        shooter.setShooterState(ShooterState.STOP);
        intake.setWantedIntakeState(WantedIntakeState.INTAKE);
        feeder.stopFeeder();
        spindexer.stopSpindexer();
    }

    public void pass() {
        shooter.setShooterState(ShooterState.PASS);
        intake.setWantedIntakeState(WantedIntakeState.INTAKE);
        // When shooter is up to speed, drive feeder and spindexer.
        if (shooter.isAtVelocity() || RobotBase.isSimulation()) {
            feeder.runFeeder(50.0);
            spindexer.runSpindexer(50.0);
        }
    }

    public void stop() {
        shooter.setShooterState(ShooterState.STOP);
        intake.setWantedIntakeState(WantedIntakeState.STOP);
        feeder.stopFeeder();
        spindexer.stopSpindexer();
    }
    
}
