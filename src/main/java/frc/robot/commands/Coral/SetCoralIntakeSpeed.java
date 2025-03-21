// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralIntakeSpeed extends Command {
  /** Creates a new SetCoralIntakeSpeed. */
  private double Speed;
  private final CoralIntake m_CoralIntake = RobotContainer.coralIntake;
  public SetCoralIntakeSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Speed = speed;
    addRequirements(m_CoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralIntake.RunCoralIntake(() -> Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralIntake.StopCoralIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
