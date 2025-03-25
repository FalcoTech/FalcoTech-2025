// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToNearestTagWithOffset extends Command {
  private final CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  private static final PathConstraints m_pathConstraints = RobotContainer.pathFindConstraints;
  private boolean offsetRight;
  private AprilTag targetTag;
  private double offsetDistance;
  private Pose2d currentPose;
  private double offsetDistanceX;
  private double offsetDistanceY;
    // AprilTagFieldLayout fieldLayout;
    /** Creates a new AlignToNearestTagWithOffset. */
    public AlignToNearestTagWithOffset(AprilTagFieldLayout fieldLayout, boolean offsetRight) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.offsetRight = offsetRight;
      addRequirements(m_drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // get the current pose from the odometry
      currentPose = m_drivetrain.getState().Pose;
      // get the nearest tag
  
      // inverse offset when button pressed default is left
  
      // if(offsetRight){
      //   offsetDistance = -1*offsetDistance;
  
      // }
      targetTag = findNearestTag();
      
      AutoBuilder.pathfindToPose(currentPose, m_pathConstraints, null)
  
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
    private Pose2d getTargetPose(){
      // get the target pose
      
   
  
        offsetDistance = 0.1524;  // 6 inches in meters
        

        // Apply left/right direction relative to the current tag
        if (!offsetLeft) {
            offsetDirection = new Transform2d(0, 1, new Rotation2d(0)).getTranslation();
        } else {
            offsetDirection = new Transform2d(0, -1, new Rotation2d(0)).getTranslation();
        }
        
        // Calculate final target pose
        targetPose = new Pose2d(
            targetTag.pose.toPose2d().getX() + offsetDirection.getX() * offsetDistance,
            targetTag.pose.toPose2d().getY() + offsetDirection.getY() * offsetDistance,
            targetTag.pose.getRotation()  // Face same direction as tag
        );

    return targetPose;

  }

  private AprilTag findNearestTag(){
    // get the nearest tag
    AprilTag nearestTag = null; // fix this
    double nearestDistance = Double.MAX_VALUE;
    //Add loop to check list of tags
    for(AprilTag tag : ){ ){
      double distance = currentPose.getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
      if (distance < nearestDistance){
        nearestDistance = distance;
        nearestTag = tag;
      }
      
    }
    return nearestTag;
  }
}
