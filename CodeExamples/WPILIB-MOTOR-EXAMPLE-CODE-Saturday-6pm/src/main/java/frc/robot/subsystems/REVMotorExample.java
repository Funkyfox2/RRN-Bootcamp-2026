// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class REVMotorExample extends SubsystemBase {
  SparkMax sparkmax = 
    new SparkMax(1, MotorType.kBrushless);
  SparkFlex sparkFlex = 
    new SparkFlex(1, MotorType.kBrushless);
  
  RelativeEncoder encoder = sparkmax.getEncoder();
  AbsoluteEncoder absolute = sparkmax.getAbsoluteEncoder();
  RelativeEncoder alternateEncoder = sparkmax.getAlternateEncoder();
  SparkAnalogSensor analogSensor = sparkmax.getAnalog();
  SparkClosedLoopController pid = 
  sparkmax.getClosedLoopController();
  
  /** Creates a new REVMotorExample. */
  public REVMotorExample() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.feedbackSensor(
      FeedbackSensor.kPrimaryEncoder);
    config.smartCurrentLimit(60);
    config.encoder.positionConversionFactor(1.0);
    config.closedLoop.p(0).i(0).d(0);
    config.closedLoop.feedForward.kV(0).kA(0).kS(0).kG(0);
    sparkmax.configure(
      config, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = encoder.getPosition();
    double velocity = encoder.getVelocity();

    double busVoltage = sparkmax.getBusVoltage();
  }

  public void setPosition(double position) {
    pid.setSetpoint(position, ControlType.kPosition);
  }
}
