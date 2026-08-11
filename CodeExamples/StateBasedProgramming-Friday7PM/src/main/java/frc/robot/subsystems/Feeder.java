package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

/*** In this subsystem example, the subsystem itself doesn't have a state machine. Instead, it contains methods to control the motors in the subsystem. */
public class Feeder extends SubsystemBase {

    TalonFX feederMotor = new TalonFX(CANConstants.feederID, CANConstants.canivore);

    VelocityVoltage velocityControl = new VelocityVoltage(0);

    double targetFeederSpeed;
    DoublePublisher speedTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Target Feeder RPS").publish();
    
    public Feeder() {
        // Motor configuration and other things here
    }

    public void runFeeder(double rps) {
        targetFeederSpeed = rps;
        feederMotor.setControl(velocityControl.withVelocity(rps).withSlot(0));
    }

    public void stopFeeder() {
        targetFeederSpeed = 0.0;
        feederMotor.stopMotor();
    }

    @Override
    public void periodic() {
        speedTargetPublisher.set(targetFeederSpeed);
    }

}
