package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public final class Constants {
    public static final class ROBOT {

    }
    public static final class DRIVETRAIN {
        public static final double kTrackWidth = 0.474;
        public static final double kWheelBase = 0.474;
        public static final double kWheelTrackDiameter = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2));
        public static final double kRobotRadiansPerSec = 12.9;
        public static final SwerveDriveKinematics kDrivetrainKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        public static double kp_RobotAngle = 0.01;
        public static double ki_RobotAngle = 0.005;
        public static double kd_RobotAngle = 0.0;
        public static double kp_RobotDrive = 0.01;
        public static double kd_RobotDrive = 0.005;
        public static double ki_RobotDrive = 0.0;
        public static double kp_RobotDriveRotate = 0.01;
        public static double kd_RobotDriveRotate = 0.005;
        public static double ki_RobotDriveRotate = 0.0;
        
        
    }
    public static final class OI{
        public static final double kCMax = 0.9;
    }
    public static final class SWERVE {
        public static final double kMaxDriveEncVel_UnitsPer100ms = 21777.0; // Measured from Encoder in native units
        public static final double kMaxVel_MPS = 4.324; // Calculated from gear ratios and wheel circumference
        public static final double kDriveEncVelRatio = kMaxVel_MPS / (kMaxDriveEncVel_UnitsPer100ms * 10);

        public static final double kSteerMotEncoderCountsPerRev = 2048.0;
        public static final double kSteerRatio = 15.43;
        public static final double kSteerMotCntsPerWheelDeg = (kSteerMotEncoderCountsPerRev * kSteerRatio) / 360;
        public static final double kSteerMotCountsPerWheelRadian = (kSteerMotEncoderCountsPerRev / (2 * Math.PI)) * kSteerRatio;
        
       // public static Map<String, SwerveData> SwerveModules = new HashMap<>();
        public static SwerveData LFData = new SwerveData("LF", 15, InvertType.None, 11, InvertType.InvertMotorOutput, 4, 64.0);
        public static SwerveData RFData = new SwerveData("RF", 14, InvertType.InvertMotorOutput, 10, InvertType.InvertMotorOutput, 3, 305.7);
        public static SwerveData LBData = new SwerveData("LB", 17, InvertType.None, 13, InvertType.InvertMotorOutput, 6, 111.0);
        public static SwerveData RBData = new SwerveData("RB", 16, InvertType.InvertMotorOutput, 12, InvertType.InvertMotorOutput, 5, 323);

        public static final double kDriveMotEncoderCountsPerRev = 2048.0;
        public static final double kDriveRatio = 7.85;
        public static final double kWheelDiameter = 4.0;
        public static final double kDriveCntsPerInch = kDriveMotEncoderCountsPerRev * kDriveRatio / (Math.PI * kWheelDiameter);
    }

}
