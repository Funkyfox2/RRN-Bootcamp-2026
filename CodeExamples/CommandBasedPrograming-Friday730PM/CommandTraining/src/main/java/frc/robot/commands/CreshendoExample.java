// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class CreshendoExample {

  /*
   *
   * Deploy Intake Example Code,
   *
   * CODE WILL NOT WORK IN SIM
   * CODE WILL NOT WORK IN SIM
   * CODE WILL NOT WORK IN SIM
   */

  // the Example States
  public static enum CreshendoExampleStates {
    INTAKE,
    SPEAKER,
    AMP,
    CLIMBING
  }

  CreshendoExampleStates states = CreshendoExampleStates.INTAKE;
  CreshendoExampleStates PreviusScoringState = CreshendoExampleStates.SPEAKER;

  CommandXboxController xboxController = new CommandXboxController(0);

  IndexerSubsytem m_IndexerSubsytem = new IndexerSubsytem();
  ShooterSubSytem m_ShooterSubSytem = new ShooterSubSytem();
  IntakeSubsytem m_IntakeSubsytem = new IntakeSubsytem();
  DriveSubsytem m_DriveSubsytem = new DriveSubsytem();

  /* Example Configure Bindings for Two States: Intake and Speaker */
  public void CreshendoExampleConfigureBindings() {

    /* set up Controller Triggers */

    Trigger GetScoreReady = xboxController.leftTrigger();
    Trigger score = xboxController.rightTrigger();
    Trigger setIntakeState = xboxController.y();
    Trigger setSpeakerState = xboxController.b();

    /* set up State Triggers */

    // is State Triggers
    Trigger isSpeaker = new Trigger(() -> isState(CreshendoExampleStates.SPEAKER));
    Trigger isIntake = new Trigger(() -> isState(CreshendoExampleStates.INTAKE));

    // action Triggers
    Trigger IndexerDectsNote = new Trigger(() -> m_IndexerSubsytem.IndexerHasNote());

    /* set Default Commands  */

    // drive
    m_DriveSubsytem.setDefaultCommand(
        m_DriveSubsytem.drive(
            () -> xboxController.getLeftX(),
            () -> xboxController.getLeftY(),
            () -> xboxController.getRightX()));

    // set Defualt Intake Command as retract
    m_IntakeSubsytem.setDefaultCommand(m_IntakeSubsytem.UndeployIntake());

    // set Default Indexer Command as Stop
    m_IndexerSubsytem.setDefaultCommand(m_IndexerSubsytem.RunIndexer(() -> 0));

    /* set Commands that set Commands */

    setIntakeState.onTrue(setState(CreshendoExampleStates.INTAKE));
    setSpeakerState.onTrue(setState(CreshendoExampleStates.SPEAKER));

    /* Intake State */

    /*
     * Intake:
     * when we hit "Get Ready to Score Button" Run Intake Sequence,
     * "When Note is Detected by Indexer" Switch to Mode Past Scoring Mode
     */

    // deploy, Default Command Brings back
    isIntake
        .and(() -> GetScoreReady.getAsBoolean())
        .whileTrue(DeployIntake(m_IntakeSubsytem, m_IndexerSubsytem));

    // if our Intake is being Deployed and we dect Not Switch to Previus Scoring Mode
    isIntake
        .and(() -> (GetScoreReady.getAsBoolean() && IndexerDectsNote.getAsBoolean()))
        .onTrue(getPreviusScoringState());

    /* Shooter State */

    /*
     * Speaker: When Mode Speaker: Check if Note is in Shooter If not shuffle Note to shooter.
     * when we hit "Get Ready to Score Button" Spin Up Shooter, Aim Shooter, and auto align to the speaker,
     *  " Shoot Button" Check if Shooter is ready to Shoot, Fire Note,
     *  "When Note is Fired" Switch to Mode Intake
     */

    // if set to Speaker make sure note is in speaker
    isSpeaker.onTrue(shuffleNote(CreshendoExampleStates.SPEAKER));

    // if whe Get score ready Spin up and Auto Rotate
    isSpeaker
        .and(() -> GetScoreReady.getAsBoolean())
        .whileTrue(SpinUp(m_ShooterSubSytem, m_DriveSubsytem));

    // when Score Shoot and Contue to aling with speaker, after fire change to intake
    isSpeaker
        .and(() -> score.getAsBoolean())
        .whileTrue(
            Fire(m_ShooterSubSytem, m_DriveSubsytem)
                .andThen(setState(CreshendoExampleStates.INTAKE)));
  }

  /* spins up shooter while auto alinging */
  public Command SpinUp(ShooterSubSytem shooterSubSytem, DriveSubsytem drive) {
    return m_ShooterSubSytem
        .setSpinUpData(
            ShooterSubSytem.lookUpSpinUpData(() -> m_DriveSubsytem.getDistanceFromSpeaker()))
        .alongWith(
            AutoRotateSpeaker(
                m_DriveSubsytem, () -> xboxController.getLeftX(), () -> xboxController.getLeftY()));
  }

  /* Wait till shooter spun up then fire */
  public Command Fire(ShooterSubSytem shooterSubSytem, DriveSubsytem drive) {
    return Commands.deadline(
            Commands.waitUntil(() -> shooterSubSytem.hasSpunUp()), SpinUp(shooterSubSytem, drive))
        .andThen(shooterSubSytem.Fire(() -> 6000));
  }

  public boolean isState(CreshendoExampleStates creshendoExampleStates) {
    return states == creshendoExampleStates;
  }

  public Command setState(CreshendoExampleStates creshendoExampleStates) {
    return Commands.runOnce(
        () -> {
          states = creshendoExampleStates;
        });
  }

  public Command getPreviusScoringState() {
    return Commands.runOnce(
        () -> {
          states = PreviusScoringState;
        });
  }

  public Command setPreviusScoringState() {
    return Commands.runOnce(
        () -> {
          PreviusScoringState = states;
        });
  }

  public Command DeployIntake(IntakeSubsytem intakeSubsytem, IndexerSubsytem indexerSubsytem) {

    /*
     *
     * Deploy the angle of the Intake While Running the Indexer and intake motors
     *
     */

    return Commands.parallel(
        intakeSubsytem.DeployIntake(() -> Constants.DeployIntakeExConstants.IntakeRPMRunning),
        indexerSubsytem.RunIndexer(() -> Constants.DeployIntakeExConstants.IndexerIntakeSpeed));
  }

  /* Shuffle Note to Current State */
  public Command shuffleNote(CreshendoExampleStates state) {
    /* im to lazy to program this */
    return Commands.none();
  }

  /* AutoRotates to speaker */
  public Command AutoRotateSpeaker(DriveSubsytem drive, DoubleSupplier x, DoubleSupplier y) {
    /* im to lazy to program this */
    return Commands.none();
  }

  public Command RetractIntake(IntakeSubsytem intakeSubsytem, IndexerSubsytem indexerSubsytem) {

    /*
     *
     * Deploy the angle of the Intake While Running the Indexer and intake motors
     *
     */

    return Commands.parallel(intakeSubsytem.UndeployIntake(), indexerSubsytem.RunIndexer(() -> 0));
  }

  /** Creates a new DeployIntakeExample. */
  public Command DeployIntakeExample(
      IntakeSubsytem intakeSubsytem, IndexerSubsytem indexerSubsytem) {

    /*
     *
     * Goal Run Intake Until We Intake A Note
     *
     * 1. Group, Deploy Our Intake At Running Speed, Run Our Wheals
     * 2. Stop Group 1. when we decect a Note in Indexer, and start 3
     * 3. Group Retract Intake And Stop Indexer
     *
     */

    return Commands.sequence(

        // Group 1
        Commands.parallel(
                intakeSubsytem.DeployIntake(
                    () -> Constants.DeployIntakeExConstants.IntakeRPMRunning),
                indexerSubsytem.RunIndexer(
                    () -> Constants.DeployIntakeExConstants.IndexerIntakeSpeed))

            // Run Until we Dect Indexer Has Note Then Runn a little more
            .withDeadline(
                Commands.waitUntil(() -> indexerSubsytem.IndexerHasNote())
                    .andThen(Commands.waitTime(Milliseconds.of(100)))),

        // Group 3
        Commands.parallel(intakeSubsytem.UndeployIntake(), indexerSubsytem.RunIndexer(() -> 0))

        // Ends Command When Intake Is back In

        );
  }

  public class IntakeSubsytem extends SubsystemBase {

    /*
     * Deploys Intake At Running Angle
     *
     * Sets Intake Wheals to Intake Speed RPM
     */
    public Command DeployIntake(DoubleSupplier IntakeSpeedRPM) {
      return Commands.none();
    }

    public Command UndeployIntake() {
      return Commands.none();
    }
  }

  public class ShooterSubSytem extends SubsystemBase {

    public static class SpinUpData {

      Rotation2d AimAngle;
      double FlyWhealSpeed;
    }

    /*
     * Runs Angler and Flywheal
     */
    public Command setSpinUpData(Supplier<SpinUpData> SpinUpData) {
      return Commands.none();
    }

    /*
     *
     * Runs Flywheals at speed and runs mag rollers
     *
     */
    public Command Fire(DoubleSupplier magSpeed) {
      return Commands.none();
    }

    /*
     *
     * Chechs if Spin up data Has bean reached
     *
     */
    public boolean hasSpunUp() {
      return false;
    }

    /*
     *
     * Looks up from map the spin up data
     *
     */
    public static Supplier<SpinUpData> lookUpSpinUpData(DoubleSupplier DistanceFromSpeaker) {
      return () -> new SpinUpData();
    }
  }

  public class DriveSubsytem extends SubsystemBase {

    /*
     * Gets the Distance From Speaker
     */

    public double getDistanceFromSpeaker() {
      return 0;
    }

    /*
     *
     * auto Aling with drive cablitys
     *
     */

    public Command driveAutoAlingRotate(Rotation2d rotation, DoubleSupplier x, DoubleSupplier y) {
      return Commands.none();
    }

    /*
     *
     * drives with x, y
     *
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier Theta) {
      return Commands.none();
    }
  }

  public class IndexerSubsytem extends SubsystemBase {

    /*
     * Runs Indexer at Speed RPM
     */
    public Command RunIndexer(DoubleSupplier IntakerSpeedRPM) {
      return Commands.none();
    }

    /*
     *
     * Returns Status from Sensor In Intake in Indexer Has a Note
     *
     */
    public boolean IndexerHasNote() {
      return false;
    }
  }
}
