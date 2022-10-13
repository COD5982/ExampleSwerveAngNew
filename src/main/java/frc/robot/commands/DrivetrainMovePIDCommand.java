/* Drive to a distance at a particular angle with a max speed.
-- Parameters -- 
Distance in meters
Drive Angle to drive at in Degrees relative to current robot angle assuming zero 
Robot Angle to rotate to in degrees relative to the current robot angle assuming zero
Max speed in MPS to drive at.
Timout to stop the routine as an emergency only time

-- Init --  
Calculate x and y Speed from angle and max speed.
Rotate wheels to that angle to limit initial drive and rotate error.

-- Periodic -- 
Calculate rotation PID speeds
Drive at X and Y speeds and rotation speed PID 
Stop when distance and robot angle are reached.

Possible issues or complications:
Distance calculated is off since rotation happens while driving
Robot does not drive straight:
  Friction changes on swerve modules will make the robot drive off center 
  Possible corrections are::
     Set the wheels in velocity mode. This is difficult and still has offsets associated with it.
     Change the X and Y values based on the Gyro.
        This is difficult since the gyro heading is moving during the rotation time of the robot.
        But we are in field relative mode so we should be able to get the actual needed reference angle.


How would a curve drive work?
The angle of driving would need to be changed over time and/or distance.
A table of piece wise linear would smooth out corners.
The more in the table the smoother it would be.
Or 
Or make the rotation calculate a curve that is NOT the center of the robot.

*/
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.lib.Util;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVE;
import edu.wpi.first.wpilibj.Timer;

public class DrivetrainMovePIDCommand extends CommandBase {

  double m_driveAngle; // Degrees
  double m_robotAngle; // Degrees
  double m_distance; // Inches
  double m_maxSpeed; // MPS
  double m_timeOut; // Seconds

  double m_vX; // MPS
  double m_vY; // MPS

  PIDController m_drivePIDController;
  PIDController m_rotatePIDController;
  Timer wheelRotateTimer;
  Timer commmandTimer;
  boolean m_isFinished = false;

  /**
   * 
   * @param _driveAngle Angle in degrees to drive to
   * @param _robotAngle Angle the front of the robot should face
   * @param _distance   Distance in inches to drive to
   * @param _maxSpeed   Speed to drive to from +/- 1.0
   * @param _timeOut    Time to cancel this command if angle and distance are not reached
   */
  public DrivetrainMovePIDCommand(double _driveAngle, double _robotAngle, double _distance, double _maxSpeed, double _timeOut) {
    addRequirements(RobotContainer.m_drivetrain);
    m_driveAngle = _driveAngle;
    m_robotAngle = _robotAngle;
    m_distance = _distance;
    m_maxSpeed = _maxSpeed;
    m_timeOut = _timeOut;

    m_drivePIDController = new PIDController(DRIVETRAIN.kp_RobotDrive, DRIVETRAIN.ki_RobotDrive,  DRIVETRAIN.kd_RobotDrive);
    m_rotatePIDController = new PIDController(DRIVETRAIN.kp_RobotAngle, DRIVETRAIN.ki_RobotAngle, DRIVETRAIN.kd_RobotAngle);

    m_drivePIDController.setIntegratorRange(-1.0, 1.0);
    m_drivePIDController.setTolerance(5.0);
    m_drivePIDController.disableContinuousInput();

    m_rotatePIDController.disableContinuousInput();
    m_rotatePIDController.setIntegratorRange(-1.0, 1.0);
    m_rotatePIDController.setTolerance(5.0);

  }

  /**
   * Given the angle and max speed get the vX and vY components that are stored in a ChassisSpeeds class
   * The speed is the Hypontenus length, the angle is the anlge. Find the X and Y component
   * @param _angle Field relative angle
   * @param _speed Max Speed to drive at
   * @return
   */
  public ChassisSpeeds getChassisSpeeds(double _angle, double _speed) {
    double x = _speed * Math.sin(Math.toRadians(_angle));
    //x *= SWERVE.kMaxVel_MPS;
    double y = _speed * Math.sin(Math.toRadians(90 -_angle));
    //y *= SWERVE.kMaxVel_MPS;
    return new ChassisSpeeds(x, y, 0.0);
  }

  /**
   * This will run once to make the wheels rotate to the direction needed.
   * 
   */
  @Override
  public void initialize() {
    SmartDashboard.putData(m_drivePIDController);
    SmartDashboard.putData(m_rotatePIDController);

    wheelRotateTimer = new Timer();
    wheelRotateTimer.start();

    commmandTimer = new Timer();
    commmandTimer.start();
  }

  /**
   * 
   * 1. Calculate rotatePIDController with robot angle and gyro
   * 2. Calculate drivePIDController with Drivetrain position and desired
   * m_distance
   * 5. Get the vX and vY from getChassisSpeeds(angle,speed).
   * 6. Drive(vX, vY, Rotation)
   * 7. isFinihed is when drivePIDController AND rotatePIDController are onTarget.
   * 
   * rotatePIDController stays active till drivePIDController is finished.
   * 
   * 
   */
  @Override
  public void execute() {
    // Get the rotation value output from the PID
    double rot = m_rotatePIDController.calculate(RobotContainer.m_drivetrain.getRobotAngle(), m_robotAngle);
    // Calculate the drive PID for distance.
    double driveSpeed = m_drivePIDController.calculate(RobotContainer.m_drivetrain.getRobotDistanceInch(), m_distance);
    // Limit driveSpeed to maxSpeed
    driveSpeed = Util.limit(driveSpeed, -m_maxSpeed, m_maxSpeed);
    // Get chassis speeds vX & vY from angle and max requested speed
    ChassisSpeeds speeds = getChassisSpeeds(m_driveAngle, driveSpeed);
    // First rotate the wheels to the angle the robot will drive at.
    if (!wheelRotateTimer.hasElapsed(0.5)) {
      RobotContainer.m_drivetrain.autoRotateWheels(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      RobotContainer.m_drivetrain.resetDriveMotorEncoders();
    } else {
      // Drive the robot with the vX, vY and Rotation speed
      RobotContainer.m_drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, Util.limit(rot, -1.0, 1.0));
    }
    // Stop if rotation AND distance are atSetpoint() OR the time out occured
    if (m_rotatePIDController.atSetpoint() && m_drivePIDController.atSetpoint() || commmandTimer.hasElapsed(m_timeOut)) {
      m_isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
