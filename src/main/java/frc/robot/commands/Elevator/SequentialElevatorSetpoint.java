// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SequentialElevatorSetpoint extends Command {
  private final Elevator m_Elevator = RobotContainer.elevator;
  private double Position; 
  /** Creates a new SequentialElevatorSetpoint. */
  public SequentialElevatorSetpoint(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Position = position;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Position < m_Elevator.GetLeftElevatorPosition()){ 
      m_Elevator.MoveElevator(() -> ElevatorConstants.LOWER_ELEVATOR_SPEED);
    } else {
      m_Elevator.MoveElevatorToPosition(Position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.StopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Position - m_Elevator.GetLeftElevatorPosition()) < .1 || Math.abs(RobotContainer.Copilot.getRightY()) > .2;
  }
}
