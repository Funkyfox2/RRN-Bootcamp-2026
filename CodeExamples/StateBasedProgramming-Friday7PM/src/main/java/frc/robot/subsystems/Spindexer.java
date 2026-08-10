package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class Spindexer extends SubsystemBase {
    
    TalonFX spindexerMotor = new TalonFX(CANConstants.spindexerID, CANConstants.canivore);

    VelocityVoltage velocityControl = new VelocityVoltage(0);

    double targetSpindexerSpeed;
    DoublePublisher speedTargetPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Spindexer/Target Spindexer RPS").publish();
    
    public Spindexer() {
        // Motor configuration and other things here
    }

    public void runSpindexer(double rps) {
        targetSpindexerSpeed = rps;
        spindexerMotor.setControl(velocityControl.withVelocity(rps).withSlot(0));
    }

    public void stopSpindexer() {
        targetSpindexerSpeed = 0.0;
        spindexerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        speedTargetPublisher.set(targetSpindexerSpeed);
    }

}
