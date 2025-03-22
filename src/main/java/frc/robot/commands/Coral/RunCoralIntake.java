// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralIntake extends Command {
  //initialize Coral Intake subsystem from container
  private final CoralIntake m_CoralIntake = RobotContainer.coralIntake;
  //declare a variable for speed
  private Supplier<Double> Speed;

  public RunCoralIntake(Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_CoralIntake);
    //set "global" speed using "this" keyword
    this.Speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set coral speed based on your "global" speed variable
    m_CoralIntake.RunCoralIntake(Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop coral intake 
    m_CoralIntake.StopCoralIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
