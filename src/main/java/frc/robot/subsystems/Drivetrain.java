
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVE;
import frc.robot.lib.Util;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule m_LF = new SwerveModule(SWERVE.LFData);
  private final SwerveModule m_RF = new SwerveModule(SWERVE.RFData);
  private final SwerveModule m_LB = new SwerveModule(SWERVE.LBData);
  private final SwerveModule m_RB = new SwerveModule(SWERVE.RBData);

  private final AHRS m_gyro = new AHRS(Port.kMXP);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DRIVETRAIN.kDrivetrainKinematics, m_gyro.getRotation2d());

  public Drivetrain() {

    SmartDashboard.putNumber("DesiredRobotAngle", 0.0);
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    Pose2d pose = m_odometry.update(m_gyro.getRotation2d(), m_LF.getState(), m_LB.getState(), m_RF.getState(), m_RB.getState());

    // SmartDashboard.putNumber("LFAng", m_LF.getSteerEncAngle_Deg());
    // SmartDashboard.putNumber("RFAng", m_RF.getSteerEncAngle_Deg());
    // SmartDashboard.putNumber("LBAng", m_LB.getSteerEncAngle_Deg());
    // SmartDashboard.putNumber("RBAng", m_RB.getSteerEncAngle_Deg());
    SmartDashboard.putNumber("RobotAngle", getRobotAngle());
  }

  public void drive(double _xSpeed, double _ySpeed, double _rot) {
    _xSpeed = Util.deadband(_xSpeed, 0.1);
    _ySpeed = Util.deadband(_ySpeed, 0.1);

    double xSpeed = _ySpeed * SWERVE.kMaxVel_MPS;
    double ySpeed = -_xSpeed * SWERVE.kMaxVel_MPS;
    double rot = _rot * DRIVETRAIN.kRobotRadiansPerSec;

    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kDrivetrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE.kMaxVel_MPS);

    m_LF.setDesiredState(swerveModuleStates[0]);
    m_RF.setDesiredState(swerveModuleStates[1]);
    m_LB.setDesiredState(swerveModuleStates[2]);
    m_RB.setDesiredState(swerveModuleStates[3]);

  }

  public void autoRotateWheels(double _xSpeed, double _ySpeed) {
    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kDrivetrainKinematics.toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, 0));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE.kMaxVel_MPS);
    m_LF.setDesiredState(swerveModuleStates[0], true);
    m_RF.setDesiredState(swerveModuleStates[1], true);
    m_LB.setDesiredState(swerveModuleStates[2], true);
    m_RB.setDesiredState(swerveModuleStates[3], true);
  }

  public double getRobotAngle() {
    return -m_gyro.getAngle();
  }

  public double getRobotDistanceInch() {
    double dis = (m_LF.getDriveMotorCnts() + m_RF.getDriveMotorCnts() + m_LB.getDriveMotorCnts()
        + m_RB.getDriveMotorCnts()) / 4.0;
    return dis / SWERVE.kDriveCntsPerInch;
  }

  public void resetDriveMotorEncoders() {
    m_LF.resetDriveMotorEncoderCnts();
    m_LB.resetDriveMotorEncoderCnts();
    m_RF.resetDriveMotorEncoderCnts();
    m_RB.resetDriveMotorEncoderCnts();
  }

  @Override
  public void simulationPeriodic() {

  }
}
