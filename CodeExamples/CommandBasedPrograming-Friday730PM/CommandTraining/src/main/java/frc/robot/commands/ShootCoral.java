// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndeffectorSubsytem;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {

  private final DoubleSupplier RotationsPerSecound;
  private final DoubleSupplier DistanceInches;

  private double StartShooterPositionInches = 0;
  double EndShooterPositionInches = 0;

  EndeffectorSubsytem EndEffector;

  public ShootCoral(
      DoubleSupplier RotationsPerSecound,
      DoubleSupplier distanceInches,
      EndeffectorSubsytem endeffectorSubsytem) {
    this.RotationsPerSecound = RotationsPerSecound;
    DistanceInches = distanceInches;
    EndEffector = endeffectorSubsytem;
    addRequirements(EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // sets the End Condtion For Command
    StartShooterPositionInches = EndEffector.getEndEffectorLinearPosition();
    EndShooterPositionInches = StartShooterPositionInches + DistanceInches.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // run shooter

    // Convert Linear Velocity to Angluar Velocity
    EndEffector.setVelocity(RotationsPerSecond.of(RotationsPerSecound.getAsDouble())).execute();

    // Log
    SmartDashboard.putNumber("ShootCoral/ShooterLog/Start", StartShooterPositionInches);
    SmartDashboard.putNumber(
        "ShootCoral/ShooterLog/Current", EndEffector.getEndEffectorLinearPosition());
    SmartDashboard.putNumber("ShootCoral/ShooterLog/End", EndShooterPositionInches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    EndEffector.setVelocity(RotationsPerSecond.of(0)).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return EndShooterPositionInches < EndEffector.getEndEffectorLinearPosition();
  }
}
