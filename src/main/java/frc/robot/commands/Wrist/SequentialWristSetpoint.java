// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SequentialWristSetpoint extends Command {
  private double Position;
  private Wrist m_wrist;
  
  /** Creates a new SequentialWristSetpoint. */
  public SequentialWristSetpoint(double position) {
    Position = position;
    m_wrist = RobotContainer.wrist;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.MoveWristToPosition(Position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Maybe make this condiitional and have is Finished hit true if the copilot moves the joystick 
    // if (interrupted){
      m_wrist.StopWrist();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Position - m_wrist.GetWristEncoderPosition()) < WristConstants.POSITION_THRESHOLD || 
           Math.abs(RobotContainer.Copilot.getLeftY()) > WristConstants.OVERRIDE_THRESHOLD;
  }
}
