// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.lib.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetrainDefaultPIDCommand extends PIDCommand {
 

  /** Creates a new DrivetrainDefaultPIDCommand. */
  public DrivetrainDefaultPIDCommand(Subsystem _subsystem) {
    super(
        // The controller that the command will use
        new PIDController(DRIVETRAIN.kp_RobotAngle, DRIVETRAIN.ki_RobotAngle, DRIVETRAIN.kd_RobotAngle),
        // This should return the measurement
        () -> (RobotContainer.m_drivetrain.getRobotAngle()),
        // This should return the setpoint (can also be a constant)
        () -> //SmartDashboard.getNumber("DesiredRobotAngle", 0.0), 
        Util.getAngle(RobotContainer.stick.getRawAxis(4), -RobotContainer.stick.getRawAxis(5)),
        // This uses the output
        output -> {
          double out = Util.limit(output, -1.0, 1.0);
          RobotContainer.m_drivetrain.drive(RobotContainer.stick.getRawAxis(0), -RobotContainer.stick.getRawAxis(1), out);
          SmartDashboard.putNumber("Rotation Out", out);
        }
        );
        addRequirements(_subsystem);
        getController().disableContinuousInput();
        //getController().enableContinuousInput(-360, 360);
        getController().setTolerance(1);
        getController().setIntegratorRange(-1, 1);

        
  }

  @Override
  public void execute(){
    super.execute();
    // getController().setP(DRIVETRAIN.kp_RobotAngle);
    // getController().setI(DRIVETRAIN.ki_RobotAngle);
    // getController().setD(DRIVETRAIN.kd_RobotAngle);
    SmartDashboard.putBoolean("kCMax", Util.hyp(RobotContainer.stick.getRawAxis(4), -RobotContainer.stick.getRawAxis(5)));
    if(Util.hyp(RobotContainer.stick.getRawAxis(4), -RobotContainer.stick.getRawAxis(5))){
      SmartDashboard.putNumber("ThumbAng", Util.getAngle(RobotContainer.stick.getRawAxis(4), -RobotContainer.stick.getRawAxis(5)));
    }
    
    
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
