// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  
  private boolean enableMegaTag2;
  
    public Robot() {
      m_robotContainer = new RobotContainer();
    }
  
    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
  
      /*
       * This example of adding Limelight is very simple and may not be sufficient for on-field use.
       * Users typically need to provide a standard deviation that scales with the distance to target
       * and changes with number of tags available.
       *
       * This example is sufficient to show that vision integration is possible, though exact implementation
       * of how to use vision should be tuned per-robot and to the team's specification.
       */
      if (kUseLimelight && !enableMegaTag2) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // LimelightHelpers.SetRobotOrientation("", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
    enableMegaTag2 = SmartDashboard.getBoolean("Enable MegaTag2", enableMegaTag2);
    // SmartDashboard.getBoolean("Enable MegaTag2", enableMegaTag2);
    
    if (enableMegaTag2){
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds,
          VecBuilder.fill(.5, .5, 9999999));
        // RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
