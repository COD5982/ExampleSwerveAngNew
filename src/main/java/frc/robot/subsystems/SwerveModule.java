// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveData;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVE;

/** Add your docs here. */
public class SwerveModule {
    private WPI_TalonFX m_steerMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_steerEncoder;
    private double m_steerEncoderAbsoluteOffset;
    SwerveData m_data;

    public SwerveModule(SwerveData _data){

        m_data = _data;
        
        m_driveMotor = new WPI_TalonFX(m_data.driveCANID);
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setInverted(m_data.driveInvert);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configOpenloopRamp(0.1);

        m_steerMotor = new WPI_TalonFX(m_data.steerCANID);
        m_steerMotor.setInverted(m_data.steerInvert);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.config_kP(0, .105);
        m_steerMotor.configClosedloopRamp(0.1);
        m_steerEncoderAbsoluteOffset = m_data.angleOffset_Deg;

        m_steerEncoder = new CANCoder(m_data.canCoderCANID);
        double steerAng = m_steerEncoder.getAbsolutePosition();
        double steerDiff =  (steerAng - m_steerEncoderAbsoluteOffset) * SWERVE.kSteerMotCntsPerWheelDeg;
        m_steerMotor.setSelectedSensorPosition(steerDiff);
    }
    public double getDriveMotorCnts(){
        return m_driveMotor.getSelectedSensorPosition();
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity_MPS(), new Rotation2d(getSteerEncAngle_Rad()));
    }
    private double getSteerEncAngle_Rad() {
        return m_steerEncoder.getPosition() * Math.PI / 180;
    }
    public double getSteerEncAngle_Deg() {
        return m_steerEncoder.getAbsolutePosition();
    }
    public double getDriveVelocity_MPS(){
        // get the sensor velocity in units/100ms. Multiply by 10 to get seconds of units
        return m_driveMotor.getSelectedSensorVelocity() * 10 * SWERVE.kDriveEncVelRatio;
    }
    public void setDesiredState(SwerveModuleState _desiredState) {
        //calcNewAngle(_desiredState);
      //SwerveModuleState swerveModuleState = _desiredState;
      SwerveModuleState swerveModuleState = SwerveModuleState.optimize(_desiredState, getRotation2d());
      
      double steerAng = swerveModuleState.angle.getRadians();
      double steerCnts = steerAng * SWERVE.kSteerMotCountsPerWheelRadian;
      m_steerMotor.set(ControlMode.Position, steerCnts);
      
      double driveSpeed = swerveModuleState.speedMetersPerSecond;
      driveSpeed = driveSpeed / SWERVE.kMaxVel_MPS;
      m_driveMotor.set(ControlMode.PercentOutput, driveSpeed);
     }
     public void setDesiredState(SwerveModuleState _desiredState, boolean disableDrive){
        //SwerveModuleState swerveModuleState = _desiredState;
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(_desiredState, getRotation2d());
        double steerAng = swerveModuleState.angle.getRadians();
        double steerCnts = steerAng * SWERVE.kSteerMotCountsPerWheelRadian;
        m_steerMotor.set(ControlMode.Position, steerCnts);

        if(!disableDrive){
            double driveSpeed = swerveModuleState.speedMetersPerSecond / SWERVE.kMaxVel_MPS;
            m_driveMotor.set(ControlMode.PercentOutput, driveSpeed);
        }else {
            m_driveMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }
    public double getSteerMotAngInRad() {
        double motCnts =  m_steerMotor.getSelectedSensorPosition();
        motCnts = motCnts % 2047;
        return motCnts / SWERVE.kSteerMotCountsPerWheelRadian;
    }
    public Rotation2d getRotation2d() {
        return new Rotation2d(getSteerMotAngInRad());
    }
    public void resetDriveMotorEncoderCnts(){
        m_driveMotor.setSelectedSensorPosition(0);
    }
}
