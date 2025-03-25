// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToNearestTagWithOffset extends Command {
  private final CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  private final PathConstraints m_pathConstraints = RobotContainer.pathFindConstraints;
  private boolean offsetRight;
  private AprilTag targetTag;
  private Pose2d currentPose;
  private Pose2d targetPose;
  private Command pathfindingCommand;
  double forwardDistance = 0.3048;  // 12 inches in meters
  double lateralDistance = 0.1524;  // 6 inches in meters for left/right offset
  private List<AprilTag> reefTags;
  AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    /** Creates a new AlignToNearestTagWithOffset. */
  
  
  public AlignToNearestTagWithOffset(boolean offsetRight) {
      //reducing the number of tags to loop through
      reefTags = new ArrayList<AprilTag>();
      reefTags.addAll(field.getTags().subList(6,11));
      reefTags.addAll(field.getTags().subList(17,22));
      // Use addRequirements() here to declare subsystem dependencies.
      this.offsetRight = offsetRight;
      addRequirements(m_drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // get the current pose from the odometry
      currentPose = m_drivetrain.getState().Pose;
      targetTag = getNearestTag();
      targetPose = getTargetPose(targetTag);
      
      pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, m_pathConstraints, 0);
      pathfindingCommand.schedule();  
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
      // if robot pose is within 2 inches of target pose stop and rumble controller?
      if(m_drivetrain.getState().Pose.getTranslation().getDistance(targetPose.getTranslation()) < 0.05 ){ // 2 inches in meters
        // new RumbleCommand(RobotContainer.pilot, 0.5).schedule();
        return true;
      }
      return false;
    }

    // get the target pose
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

  // get the nearest tag
  private AprilTag getNearestTag(){
    AprilTag nearestTag = null;
    double nearestDistance = Double.MAX_VALUE;
    //loop through all reef tags in the field by reducing the number of tags to loop through
    for(AprilTag tag : reefTags){ 
      double distance = currentPose.getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
      if (distance < nearestDistance){
        nearestDistance = distance;
        nearestTag = tag;
      }
    }
    // print out the distance to the tag for debugging to make sure this works
    if (nearestTag != null) {
      System.out.println("Distance to tag " + nearestTag.ID + " is " + nearestDistance);
    } else {
      System.out.println("No valid tags found");
    }
    return nearestTag;
  }
}
