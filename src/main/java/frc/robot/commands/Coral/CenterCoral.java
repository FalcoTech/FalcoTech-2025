// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterCoral extends Command {
  private final CoralIntake coralIntakeSubsystem = RobotContainer.coralIntake;
    private double startPosition;
    /** Creates a new CenterCoral. */
    public CenterCoral() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(coralIntakeSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      startPosition = coralIntakeSubsystem.GetCoralIntakePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntakeSubsystem.setCoralIntakePosition(startPosition+0.25);
    Commands.waitUntil(() -> Math.abs(coralIntakeSubsystem.GetCoralIntakePosition() - startPosition) < 0.1);
    coralIntakeSubsystem.setCoralIntakePosition(startPosition-0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntakeSubsystem.StopCoralIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
