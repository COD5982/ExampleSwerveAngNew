// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivetrainDefaultCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param _subsystem The subsystem used by this command.
   */
  public DrivetrainDefaultCommand(Drivetrain _subsystem) {
    m_subsystem = _subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = RobotContainer.stick.getRawAxis(0);
    double y = -RobotContainer.stick.getRawAxis(1);
    double rot = RobotContainer.stick.getRawAxis(4);
    double desiredRobotAngle = SmartDashboard.getNumber("DesiredRobotAngle", 0.0);

    RobotContainer.m_drivetrain.drive(x, y, rot);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
