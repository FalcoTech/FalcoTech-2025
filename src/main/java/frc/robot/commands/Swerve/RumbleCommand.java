// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleCommand extends Command {
  CommandXboxController mController;
  private Timer timer = new Timer();
  private double rumbleTime;
  /** Creates a new RumbleCommand. */
  public RumbleCommand(CommandXboxController controller, double rumbleTime) {
    this.mController = controller;
    this.rumbleTime = rumbleTime;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mController.setRumble(RumbleType.kBothRumble, 1); // Set the rumble to full
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mController.setRumble(RumbleType.kBothRumble, 0); // Turn off the rumble
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(rumbleTime);
  }
}
