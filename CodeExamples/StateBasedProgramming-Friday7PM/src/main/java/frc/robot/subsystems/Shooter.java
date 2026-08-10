package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.FieldConstants;

public class Shooter extends SubsystemBase {

    TalonFX shooterMotor = new TalonFX(CANConstants.shooterID, CANConstants.canivore);
    Drivetrain drivetrain;

    VelocityVoltage velocityControl = new VelocityVoltage(0);

    double targetShotSpeed = 0.0;
    double distanceToShoot = 0.0;
    double distanceToPass = 0.0;

    InterpolatingDoubleTreeMap shotInterpolator = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap passInterpolator = new InterpolatingDoubleTreeMap();
    
    public Shooter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Distance, RPS
        shotInterpolator.put(1.635, 11.35);
        shotInterpolator.put(2.135, 22.73);
        shotInterpolator.put(2.635, 28.68);
        shotInterpolator.put(3.135, 32.9);
        shotInterpolator.put(3.635, 40.346);
        shotInterpolator.put(4.135, 50.75);
        shotInterpolator.put(4.635, 55.0);
        shotInterpolator.put(5.135, 61.65);

        passInterpolator.put(3.0, 51.5);
        passInterpolator.put(14.0, 95.45);

        // Motor configuration and other things here
    }

    public enum ShooterState {
        PASS, // Use passing shot
        DISTANCE_SHOOT, // Shooting with dynamic position
        STATIC_SHOOT, // Shooting from repeatable position
        STOP // Stop shooter roller
    }

    public ShooterState shooterState = ShooterState.STOP;
    
    StringPublisher shooterStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Shooter/Shooter State").publish();
    DoublePublisher speedTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Shooter/Target Shooter RPS").publish();

    @Override
    public void periodic() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                distanceToShoot = FieldConstants.redHubPosition.getDistance(
                    drivetrain.getState().Pose.getTranslation());
                distanceToPass = FieldConstants.redPassingPosition - drivetrain.getState().Pose.getX();
            } else {
                distanceToShoot = FieldConstants.blueHubPosition.getDistance(
                    drivetrain.getState().Pose.getTranslation());
                distanceToPass = drivetrain.getState().Pose.getX() - FieldConstants.bluePassingPosition;
            }
        }
        shooterStatePublisher.set(shooterState.toString());

        applyState();
        speedTargetPublisher.set(targetShotSpeed);
    }

    public ShooterState setShooterState(ShooterState state) {
        this.shooterState = state;
        return state;
    }

    public void applyState() {
        switch (shooterState) {
            case PASS:
                runShooter(passInterpolator.get(distanceToPass));
                break;
            case DISTANCE_SHOOT:
                runShooter(shotInterpolator.get(distanceToShoot));
                break;
            case STATIC_SHOOT:
                runShooter(40.346);
                break;
            default:
                shooterMotor.stopMotor();
                break;
        }
    }

    public void runShooter(double rps) {
        shooterMotor.setControl(velocityControl.withVelocity(rps).withSlot(0));
        targetShotSpeed = rps;
    }

    public boolean isAtVelocity() {
        return MathUtil.isNear(targetShotSpeed, shooterMotor.getVelocity().getValueAsDouble(), 2.0);
    }

}
