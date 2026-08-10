package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    
    public static final class CANConstants {
        public static final CANBus canivore = new CANBus("omega");

        public static final int intakeRollerID = 9;
        public static final int intakeDeployID = 10;

        public static final int spindexerID = 11;
        public static final int feederID = 12;
        public static final int shooterID = 13;
    }

    public static final class FieldConstants {
        public static final double bluePassingPosition = 1.95;
        public static final double redPassingPosition = 14.45;

        public static final Translation2d blueHubPosition = new Translation2d(4.635, 4.035);
        public static final Translation2d redHubPosition = new Translation2d(11.9, 4.035);
    }

}
