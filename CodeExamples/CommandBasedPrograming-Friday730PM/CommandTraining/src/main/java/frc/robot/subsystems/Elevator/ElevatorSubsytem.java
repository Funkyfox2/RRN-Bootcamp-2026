// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsytem extends SubsystemBase {

  private SmartMotorControllerConfig smcMotorController =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Drum radius is required for elevators. Chain-driven: specify chain pitch and tooth
          // count.
          .withDrumRadius(Inches.of(0.25), 22)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(4, 0, 0)
          .withTrapezoidalProfile(MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          .withSimClosedLoopController(4, 0, 0)
          // Feedforward Constants
          .withFeedforward(new ElevatorFeedforward(0, 0, 0))
          .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withStartingPosition(Meter.of(0));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), smcMotorController);

  private ElevatorConfig elevconfig =
      new ElevatorConfig()
          .withHardLimits(Meter.of(0), Meter.of(3))
          .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
          .withCarriageWeight(Pound.of(2));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig, sparkSmartMotorController);

  /** Creates a new Elevator. */
  public ElevatorSubsytem() {}

  /**
   * Runs the elevator to the given height and does not end the command when reached.
   *
   * @param height Distance to go to.
   * @return a Command
   */
  public Command run(Distance height) {
    return elevator.run(height);
  }

  /**
   * Runs the elevator to the given height and ends the command when reached, but not the closed
   * loop controller.
   *
   * @param height Distance to go to.
   * @param tolerance Distance tolerance for completion.
   * @return A Command
   */
  public Command runTo(Distance height, Distance tolerance) {
    return elevator.runTo(height, tolerance);
  }

  public Distance getHeightIn() {
    return sparkSmartMotorController.getMeasurementPosition();
  }

  public boolean atPosition(Double DesiredHeight) {
    return MathUtil.isNear(DesiredHeight.doubleValue(), elevator.getHeight().in(Inches), 10);
  }

  /**
   * Set the elevators closed loop controller setpoint.
   *
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) {
    elevator.setMeasurementPositionSetpoint(height);
  }

  public Command setHeightSetpointCommand(Distance height) {
    return Commands.run(() -> elevator.setMeasurementPositionSetpoint(height), this);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
    SmartDashboard.putNumber("ElevatorHeight", getHeightIn().in(Inches));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
