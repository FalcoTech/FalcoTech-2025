// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlignmentSystem extends SubsystemBase {
  // private final CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  private CommandSwerveDrivetrain m_drivetrain;
  private final PathConstraints m_pathConstraints = RobotContainer.pathFindConstraints;
  private boolean offsetRight;
  private AprilTag targetTag;
  private Pose2d targetPose;

  double CoralForwardDistance = -0.381;  // 15 inches in meters away from the tag
  // double forwardDistance = -1;  // 1 meter
  // double lateralDistance = 2;  // 2 meters
  double CoralLateralDistance = 0.1524;  // 6 inches in meters for left/right offset
  private List<AprilTag> reefTags;
  private List<AprilTag> algaeProcTags;
  private List<AprilTag> coralStationTags;
  private List<AprilTag> bargeTags;
  AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
  /** Creates a new AlignmentSystem. */
  public AlignmentSystem(CommandSwerveDrivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    reefTags = new ArrayList<AprilTag>();
    reefTags.addAll(field.getTags().subList(5,11)); // Red Reef Tags (need to write 5 so 6 is included)
    reefTags.addAll(field.getTags().subList(16,22)); // Blue Reef Tags (need to write 16 so 17 is included)
    coralStationTags =new ArrayList<AprilTag>();
    coralStationTags.add(field.getTags().get(1));
    coralStationTags.add(field.getTags().get(2));
    coralStationTags.add(field.getTags().get(12));
    coralStationTags.add(field.getTags().get(13));
    bargeTags = new ArrayList<AprilTag>();
    bargeTags.add(field.getTags().get(4));
    bargeTags.add(field.getTags().get(5));
    bargeTags.add(field.getTags().get(14));
    bargeTags.add(field.getTags().get(15));
    algaeProcTags = new ArrayList<AprilTag>();
    algaeProcTags.add(field.getTags().get(15)); //Arrays start at 0 :Zany: o.O
    algaeProcTags.add(field.getTags().get(2));
  }

  @Override
  public void periodic() {
    // this.currentPose = m_drivetrain.getState().Pose; 
    // SmartDashboard.putString("currentPose", currentPose.toString());

    // SmartDashboard.putNumber("Nearest Tag ID", getNearestTag().ID);

  }

  public Command pathfindToNearestAprilTagOld(boolean offsetRight){
    this.offsetRight = offsetRight;

    targetTag = getNearestCoralReefTag();
    targetPose = getTargetCoralReefPose(targetTag);

    // SmartDashboard.putString("AprilTag Target Pose", targetPose.toString());

    return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }

  public Pose2d getTargetPoseToNearestAprilTag(boolean offsetRight){
    this.offsetRight = offsetRight;

    targetTag = getNearestCoralReefTag();
    targetPose = getTargetCoralReefPose(targetTag);

    return targetPose;
  }

  private Pose2d getTargetCoralReefPose(AprilTag targetTag){
    SmartDashboard.putString("Target Tag", targetTag.toString());

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
        .minus(new Translation2d(normalX * CoralForwardDistance, normalY * CoralForwardDistance))
        .plus(new Translation2d(perpX * CoralLateralDistance, perpY * CoralLateralDistance));
    
    // 6. Set robot rotation to face the tag (opposite of tag's rotation)
    Rotation2d targetRotation = tagRotation.plus(new Rotation2d(Math.PI));
    
    // 7. Create and return the target pose
    targetPose = new Pose2d(targetPosition, targetRotation);
    
    return targetPose;

  }

  private Pose2d getTargetPose(AprilTag targetTag, double offsetForward, double offsetLateral){
    SmartDashboard.putString("Target Tag", targetTag.toString());

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
        .minus(new Translation2d(normalX * offsetForward, normalY * offsetForward))
        .plus(new Translation2d(perpX * offsetLateral, perpY * offsetLateral));
    
    // 6. Set robot rotation to face the tag (opposite of tag's rotation)
    Rotation2d targetRotation = tagRotation.plus(new Rotation2d(Math.PI));
    
    // 7. Create and return the target pose
    targetPose = new Pose2d(targetPosition, targetRotation);
    
    return targetPose;

  }


  private Pose2d getCurrentPose(){
    return m_drivetrain.getState().Pose;
  }

  public Command pathfindToNearestCoralReefAprilTag(boolean offsetRight) {
    // Use a ProxyCommand to defer creation until execution time
    return Commands.defer(
      () -> {
        // This lambda runs when the command is actually scheduled (button pressed)
        this.offsetRight = offsetRight;
        targetTag = getNearestCoralReefTag();
        targetPose = getTargetCoralReefPose(targetTag);
        
        // Return the actual command to be run
        return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
    }, Set.of(this));
  
}

public Command pathfindToNearestCoralStationAprilTag(boolean offsetRight) {
  // Use a ProxyCommand to defer creation until execution time
  return Commands.defer(
    () -> {
      // This lambda runs when the command is actually scheduled (button pressed)
      this.offsetRight = offsetRight;
      targetTag = getNearestTag(coralStationTags);
      targetPose = getTargetPose(targetTag,CoralForwardDistance,.610); // 2ft
      
      // Return the actual command to be run
      return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }, Set.of(this));

}


public Command pathfindToNearestAlgaeReefAprilTag() {
  // Use a ProxyCommand to defer creation until execution time
  return Commands.defer(
    () -> {
      // This lambda runs when the command is actually scheduled (button pressed)
      targetTag = getNearestTag(reefTags);
      targetPose = getTargetPose(targetTag,CoralForwardDistance,0);
      
      // Return the actual command to be run
      return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }, Set.of(this));

}
public Command pathfindToNearestAlgaeProcAprilTag() {
  // Use a ProxyCommand to defer creation until execution time
  return Commands.defer(
    () -> {
      // This lambda runs when the command is actually scheduled (button pressed)
      AprilTag targetTag = getNearestTag(algaeProcTags);
      targetPose = getTargetPose(targetTag,-.508,0); // 1 ft
      
      // Return the actual command to be run
      return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }, Set.of(this));

}
public Command pathfindToNearestCoralStationAprilTag() {
  // Use a ProxyCommand to defer creation until execution time
  return Commands.defer(
    () -> {
      // This lambda runs when the command is actually scheduled (button pressed)
      AprilTag targetTag = getNearestTag(coralStationTags);
      targetPose = getTargetPose(targetTag,CoralForwardDistance,.305);
      
      // Return the actual command to be run
      return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }, Set.of(this));

}
public Command pathfindToNearestBargeAprilTag() {
  // Use a ProxyCommand to defer creation until execution time
  return Commands.defer(
    () -> {
      // This lambda runs when the command is actually scheduled (button pressed)
      AprilTag targetTag = getNearestTag(bargeTags);
      targetPose = getTargetPose(targetTag,-1,0); // 1 meter off
      // Return the actual command to be run
      return AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
  }, Set.of(this));

}


  private AprilTag getNearestCoralReefTag(){
    AprilTag nearestTag = null;
    double nearestDistance = Double.MAX_VALUE;
    //loop through all reef tags in the field by reducing the number of tags to loop through
    for(AprilTag tag : reefTags){ 
      double distance = getCurrentPose().getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
      SmartDashboard.putNumber("Tag " + tag.ID + " Distance", distance);
      if (distance < nearestDistance){
        nearestDistance = distance;
        nearestTag = tag;
      }
    }
    // print out the distance to the tag for debugging to make sure this works
    if (nearestTag != null) {
      // System.out.println("Distance to tag " + nearestTag.ID + " is " + nearestDistance);
      // SmartDashboard.putNumber("Distance to nearest tag " + nearestTag.ID, nearestDistance);
    } else {
      // System.out.println("No valid tags found");
      // SmartDashboard.putString("No valid tags found", "");
    }
    return nearestTag;
    
  }

  private AprilTag getNearestTag(List<AprilTag> tagList){
    AprilTag nearestTag = null;
    double nearestDistance = Double.MAX_VALUE;
    //loop through all reef tags in the field by reducing the number of tags to loop through
    for(AprilTag tag : tagList){ 
      double distance = getCurrentPose().getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
      SmartDashboard.putNumber("Tag " + tag.ID + " Distance", distance);
      if (distance < nearestDistance){
        nearestDistance = distance;
        nearestTag = tag;
      }
    }
    // print out the distance to the tag for debugging to make sure this works
    if (nearestTag != null) {
      // System.out.println("Distance to tag " + nearestTag.ID + " is " + nearestDistance);
      // SmartDashboard.putNumber("Distance to nearest tag " + nearestTag.ID, nearestDistance);
    } else {
      // System.out.println("No valid tags found");
      // SmartDashboard.putString("No valid tags found", "");
    }
    return nearestTag;
    
  }

}
