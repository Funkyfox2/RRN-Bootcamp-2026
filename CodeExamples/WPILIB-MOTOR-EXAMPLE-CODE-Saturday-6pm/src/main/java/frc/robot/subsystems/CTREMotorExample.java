// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CTREMotorExample extends SubsystemBase {
  TalonFX talon = new TalonFX(1, new CANBus("Hello"));
  TalonFXConfiguration configuration = new TalonFXConfiguration();

  StatusSignal<Angle> positionSignal;
  StatusSignal<Current> currentSignal;


  // You can adjust this to output more less voltage if you want;
  VoltageOut voltageOut = new VoltageOut(0.0);
  PositionVoltage positionVoltage = new PositionVoltage(Radians.zero());
  
  /** Creates a new CTREMotorExample. */
  public CTREMotorExample() {
    configuration.Slot0.kP = 100.0;
    configuration.Feedback.RotorToSensorRatio = 10.0;
    configuration.CurrentLimits.StatorCurrentLimit = 60;
    configuration.CurrentLimits.SupplyCurrentLimit = 40;
    talon.getConfigurator().apply(configuration);

    positionSignal = talon.getPosition();
    currentSignal = talon.getStatorCurrent();

    StatusSignal.setUpdateFrequencyForAll(50.0, positionSignal,currentSignal);
    talon.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Angle position = positionSignal.refresh().getValue();
    Current stator = currentSignal.refresh().getValue();  
  }
}
