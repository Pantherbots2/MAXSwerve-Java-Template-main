// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Climber;

public class SetClimberSpeed extends Command {

  private Climber m_climber;
  private double m_speed = 0;
  /** Creates a new SetElevatorSpeed. */
  public SetClimberSpeed(Climber climber, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    m_climber = climber;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
