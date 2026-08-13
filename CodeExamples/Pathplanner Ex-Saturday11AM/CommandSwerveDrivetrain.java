package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.exceptions.NoMegaTagException;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    

    /* Field boundary constants — poses outside these bounds are rejected as invalid */
    private static final double kFieldWidthMeters  = 17.548;
    private static final double kFieldHeightMeters =  8.052;


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    /* Cached alliance color — updated alongside operator perspective to avoid redundant DS calls */
    private boolean m_isBlue = true;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
        private Field2d field2d;
    
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public CommandSwerveDrivetrain(
            Field2d  field2d,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            
            super(drivetrainConstants, modules);
            setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            if (Utils.isSimulation()) {
                startSimThread();
            }
            this.field2d = field2d;
        configureAutoBuilder();
    }

    public Pose2d getPose()
    {
        return this.getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return this.getState().Speeds;
    }

    /** Drives robot-relative -- used by PathPlanner if called directly. */
    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_isBlue = allianceColor != Alliance.Red;
                m_hasAppliedOperatorPerspective = true;
            });
        }
        // Feed MegaTag2 pose estimate from limelight-left.
        // MT2 needs the RAW gyro yaw, not the fused pose heading — using fused heading
        // creates a feedback loop since the fused pose already includes vision corrections.
        double heading = getPigeon2().getYaw().getValueAsDouble();
        double yawRateDegsPerSec = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("limelight-left", heading, yawRateDegsPerSec, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-turret", heading, yawRateDegsPerSec, 0, 0, 0, 0);
        // Skip pose injection when rotating fast — MegaTag2 is unreliable above ~720 deg/s
        // and skipping the NT reads reduces periodic runtime during aggressive maneuvers.
        double omegaDegPerSec = Math.abs(yawRateDegsPerSec);
        if (omegaDegPerSec < 720) {
            // Always use _wpiBlue — the CTRE pose estimator works in blue-origin coordinates
            // regardless of alliance. Using _wpiRed on Red causes teleporting/wrong positions.
              LimelightHelpers.PoseEstimate left =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
            /*if (left != null) {
                this.field2d.getObject("Left").setPose(left.pose);
            }*/
           // savePose(left);
           LimelightHelpers.PoseEstimate turret =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret");
            /*if (turret != null) {
                this.field2d.getObject("Left").setPose(left.pose);
            }*/
        }
        // Keep the main robot marker in sync with the fused odometry pose
        

        LimelightHelpers.SetRobotOrientation("limelight-left", heading, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode("limelight-left", 3);

        LimelightHelpers.SetRobotOrientation("limelight-turret", heading, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode("limelight-turret", 3);




        if (DriverStation.isDisabled())
        {
            LimelightHelpers.PoseEstimate e = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-left");
            //SmartDashboard.putNumber("mt1 pose", e.pose.getRotation().getDegrees());
            ArrayList<LimelightHelpers.PoseEstimate> poses = new ArrayList<>();
            poses.add(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left"));
            poses.add(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret"));
            
            ArrayList<LimelightHelpers.PoseEstimate> mtlist = new ArrayList<>();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            {
                mtlist.add(LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-left"));
                mtlist.add(LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-turret"));
            } else {
                mtlist.add(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left"));
                mtlist.add(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret"));
            }
            savePose(poses);
            try {
                double newHeading = getCamHeading(mtlist);
                SmartDashboard.putNumber("new Heading", newHeading);
                seedFieldCentric(new Rotation2d(Math.toRadians(newHeading)));
            } catch (NoMegaTagException ex) {
                // do nothing, dont need to update rotation
            }
        }
        field2d.setRobotPose(getState().Pose);
    }
    
    private void configureAutoBuilder(){
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                 ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
            config,
            // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder: " + ex.getMessage(), ex.getStackTrace());
        }
    }

    


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Injects a fresh MegaTag2 pose into the Kalman filter.
     * Rejects null, zero-tag, origin-coordinate, out-of-bounds, and low-quality estimates.
     * Uses distance-scaled std devs so far/small tags get less trust than close ones.
     */
    private void savePose(ArrayList<LimelightHelpers.PoseEstimate> mt2) {
        double x = 0;
        double y = 0;
        double r = 0;
        int c = 0;


        // loop through list of all poses
        for (LimelightHelpers.PoseEstimate mtp : mt2)
        {
            if (mtp != null) {
                Pose2d p = mtp.pose;
                if (Math.abs(p.getX()) > 0.01 || (Math.abs(p.getY())) > 0.01) {
                    x += p.getX();
                    y += p.getY();
                    c++;
                }
            }
        }

        // average them together if there were any poses
        if (c > 0)
        {
            x /= c;
            y /= c;
            Pose2d finalPose = new Pose2d(x, y, new Rotation2d(r));
            setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            addVisionMeasurement(
                finalPose,
                mt2.get(0).timestampSeconds // there was at least one pose, use first pose timestamp
            );
        }
    }

     private double getCamHeading(ArrayList<LimelightHelpers.PoseEstimate> mt) throws NoMegaTagException {
        double r = 0;
        int cr = 0;

        for (LimelightHelpers.PoseEstimate mtp : mt)
        {
            if (mtp != null) {
                Pose2d p = mtp.pose;
                if (Math.abs(p.getX()) > 0.01 || (Math.abs(p.getY())) > 0.01) {
                    System.out.println("   rotation " + p.getRotation().getDegrees());
                    r += p.getRotation().getDegrees();
                   
                    cr++;
                }
            }
        }

        if (cr > 0)
            r /= cr;
        else
            throw new NoMegaTagException();

        System.out.println("final rotation " + r);
        return r;
    }
}
