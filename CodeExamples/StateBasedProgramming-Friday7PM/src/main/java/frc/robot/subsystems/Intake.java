package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

/*** In this subsystem, structure is copied almost exactly from Superstructure. */
public class Intake extends SubsystemBase {
    
    TalonFX intakeRollerMotor = new TalonFX(CANConstants.intakeRollerID, CANConstants.canivore);
    TalonFX intakeDeployMotor = new TalonFX(CANConstants.intakeDeployID, CANConstants.canivore);

    VelocityVoltage velocityControl = new VelocityVoltage(0);
    PositionVoltage positionControl = new PositionVoltage(0);

    double deployTargetPosition = 0;
    
    public Intake() {
        // Motor configuration and other things here
    }

    public enum WantedIntakeState {
        DEPLOY,
        INTAKE,
        RETRACT,
        STOP
    }

    public enum CurrentIntakeState {
        DEPLOYING,
        INTAKING,
        RETRACTING,
        STOPPING
    }

    public WantedIntakeState wantedIntakeState = WantedIntakeState.STOP;
    public CurrentIntakeState currentIntakeState = CurrentIntakeState.STOPPING;

    StringPublisher wantedStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Intake/Wanted Intake State").publish();
    StringPublisher currentStatePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Intake/Current Intake State").publish();

    @Override
    public void periodic() {
        currentIntakeState = handleStateTransition();

        wantedStatePublisher.set(wantedIntakeState.toString());
        currentStatePublisher.set(currentIntakeState.toString());

        applyState();
    }

    public WantedIntakeState setWantedIntakeState(WantedIntakeState state) {
        this.wantedIntakeState = state;
        return wantedIntakeState;
    }

    public CurrentIntakeState handleStateTransition() {
        switch (wantedIntakeState) {
            case DEPLOY:
                return CurrentIntakeState.DEPLOYING;
            case INTAKE:
                return CurrentIntakeState.INTAKING;
            case RETRACT:
                return CurrentIntakeState.RETRACTING;
            default:
                return CurrentIntakeState.STOPPING;
        }
    }

    public void applyState() {
        switch (currentIntakeState) {
            case DEPLOYING:
                moveToPosition(30.0);
                break;
            case INTAKING:
                moveToPosition(30.0);
                if (deployNearTarget()) runRoller(65.0);
                break;
            case RETRACTING:
                moveToPosition(0.0);
                runRoller(0.0);
                break;
            default:
                intakeRollerMotor.stopMotor();
                break;
        }
    }

    public void moveToPosition(double position) {
        deployTargetPosition = position;
        intakeDeployMotor.setControl(positionControl.withPosition(position).withSlot(0));
    }

    public void runRoller(double rps) {
        intakeRollerMotor.setControl(velocityControl.withVelocity(rps).withSlot(0));
    }

    public boolean deployNearTarget() {
        return MathUtil.isNear(deployTargetPosition, intakeDeployMotor.getPosition().getValueAsDouble(), 2.0);
    }

}
