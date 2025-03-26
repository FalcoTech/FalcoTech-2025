// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  // private final CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  private CommandSwerveDrivetrain m_drivetrain;
  private final PathConstraints m_pathConstraints = RobotContainer.pathFindConstraints;
  private boolean offsetRight;
  private AprilTag targetTag;
  private Pose2d currentPose = new Pose2d();
  private Pose2d targetPose;
  private Command pathfindingCommand;
  double forwardDistance = 0.3048;  // 12 inches in meters
  double lateralDistance = 0.1524;  // 6 inches in meters for left/right offset
  private List<AprilTag> reefTags;
  AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  /** Creates a new Vision. */
  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Shouldn't this be here?
    //not really because once I run the command it should update to where the robot is I'll try it tho
    // this.currentPose = m_drivetrain.getState().Pose; 
  }

  public Command pathfindToNearestAprilTag(boolean offsetRight){
    //reducing the number of tags to loop through
    reefTags = new ArrayList<AprilTag>();
    reefTags.addAll(field.getTags().subList(6,11));
    reefTags.addAll(field.getTags().subList(17,22));
    this.offsetRight = offsetRight;

    currentPose = getCurrentPose();

    targetTag = getNearestTag();
    targetPose = getTargetPose(targetTag);


    return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }

  private Pose2d getTargetPose(AprilTag targetTag){

    // use tag relative offsets to get the target pose
    // 1. Get the tag's pose in field coordinates
    Pose2d tagPose = targetTag.pose.toPose2d();
    
    // 2. Get the tag's position and rotation
    Translation2d tagPosition = tagPose.getTranslation();
    Rotation2d tagRotation = tagPose.getRotation();
    
    // 3. Calculate the normal vector (direction the tag is facing)
    double normalX = tagRotation.getCos();
    double normalY = tagRotation.getSin();
    
    // 4. Calculate the perpendicular vector for left/right offset
    // When looking at the tag, right is 90° clockwise, left is 90° counterclockwise
    double perpX, perpY;
    if (offsetRight) {
      // Right perpendicular (90° clockwise from normal)
      perpX = -normalY;
      perpY = normalX;
    } else {
      // Left perpendicular (90° counterclockwise from normal)
      perpX = normalY;
      perpY = -normalX;
    }
    
    // 5. Calculate the target position by offsetting OPPOSITE to the normal
    // and to the left or right according to offsetRight
    Translation2d targetPosition = tagPosition
        .minus(new Translation2d(normalX * forwardDistance, normalY * forwardDistance))
        .plus(new Translation2d(perpX * lateralDistance, perpY * lateralDistance));
    
    // 6. Set robot rotation to face the tag (opposite of tag's rotation)
    Rotation2d targetRotation = tagRotation.plus(new Rotation2d(Math.PI));
    
    // 7. Create and return the target pose
    targetPose = new Pose2d(targetPosition, targetRotation);
    
    return targetPose;

  }

  private Pose2d getCurrentPose(){
    // Get the current pose of the robot
    Pose2d currentPose = m_drivetrain.getState().Pose;
    // print out the current pose for debugging
    return currentPose;
  }

  private AprilTag getNearestTag(){
    AprilTag nearestTag = null;
    double nearestDistance = Double.MAX_VALUE;
    //loop through all reef tags in the field by reducing the number of tags to loop through
    for(AprilTag tag : reefTags){ 
      double distance = currentPose.getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
      SmartDashboard.putNumber("Tag " + tag.ID + " Distance", distance);
      if (distance < nearestDistance){
        nearestDistance = distance;
        nearestTag = tag;
      }
    }
    // print out the distance to the tag for debugging to make sure this works
    if (nearestTag != null) {
      // System.out.println("Distance to tag " + nearestTag.ID + " is " + nearestDistance);
      SmartDashboard.putNumber("Distance to nearest tag " + nearestTag.ID, nearestDistance);
    } else {
      // System.out.println("No valid tags found");
      SmartDashboard.putString("No valid tags found", "");
    }
    return nearestTag;
    
  }
}
