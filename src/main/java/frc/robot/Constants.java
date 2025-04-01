package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class ControllerConstants {
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.2;
  }
  
  public static final class DrivetrainConstants {
    // These could replace some values in TunerConstants or provide fallbacks
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.96;
    public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND = 0.75 * 2 * Math.PI; // 3/4 rotation per second
  }
  
  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final boolean USE_LIMELIGHT = true;
    public static final double VISION_OMEGA_CUTOFF_RPS = 2.0; // Don't use vision measurements when rotating faster than this
  }
  
  public static final class ElevatorConstants {
    // SetPoint constants from SetElevatorToPosition.java
    public static final double HOME_POSITION = 0.0;
    public static final double FLOOR_LOAD_POSITION = 1.0;
    public static final double L2_SCORE_POSITION = 8.0;
    public static final double CORAL_STATION_POSITION = 13.5;
    public static final double L3_SCORE_POSITION = 14.8;
    public static final double ALGAE_PROCESSOR_POSITION = 5.0;
    public static final double L4_SCORE_POSITION = 26.0;
    
    // Renamed constants to be more specific
    public static final double L2_ALGAE_POSITION = 12.0;  // For Copilot.a().and(x())
    public static final double L3_ALGAE_POSITION = 17.0;  // For Copilot.a().and(y())
    public static final double BARGE_POSITION = 24.7;     // For Copilot.a().and(b())
    
    // PID constants and other elevator-related values
    public static final double ELEVATOR_CONTROL_SCALE = 0.5; // Scale for Copilot.getRightY()
    public static final double LOWER_ELEVATOR_SPEED = -0.01; //Speed to drop elevator
    
    // Safety thresholds for home position logic
    public static final double MIN_ELEVATOR_SAFETY_THRESHOLD = 1.0;
    public static final double MAX_ELEVATOR_SAFETY_THRESHOLD = 4.5;
  }
  
  public static final class WristConstants {
    public static final double HOME_POSITION = 0.0;
    public static final double FLOOR_POSITION = 12.0;
    public static final double L2_SCORE_POSITION = 5.8;
    public static final double L3_SCORE_POSITION = 5.8;
    public static final double L4_SCORE_POSITION = 5.8;
    public static final double CORAL_STATION_POSITION = 20.1;
    public static final double ALGAE_PROCESSOR_POSITION = 12.0;
    
    // Renamed constants to be more specific
    public static final double L2_ALGAE_POSITION = 12.5;  // For Copilot.a().and(x())
    public static final double L3_ALGAE_POSITION = 10.0;  // For Copilot.a().and(y())
    public static final double BARGE_POSITION = 3.0;      // For Copilot.a().and(b())
    
    // From SetWristToPosition.java line 42
    public static final double HOME_THRESHOLD = 0.05;
    public static final double OVERRIDE_THRESHOLD = 0.2;
    public static final double POSITION_THRESHOLD = 0.1;
    
    // Safety threshold for home position logic
    public static final double WRIST_SAFETY_THRESHOLD = 18.0;
  }
  
  public static final class IntakeConstants {
    public static final double ALGAE_INTAKE_SPEED = -0.5;
    public static final double ALGAE_OUTTAKE_SPEED = 1.0;
    
    public static final double CORAL_INTAKE_SPEED = 0.3;
    public static final double CORAL_OUTTAKE_SPEED = -0.2;
  }
  
  public static final class ClimbConstants {
    public static final double CLIMB_SPEED = 0.25;
  }
  
  public static final class PathPlanningConstants {
    // From RobotContainer.java
    public static final double MAX_PATH_SPEED = 3.0; // m/s
    public static final double MAX_PATH_ACCELERATION = 4.0; // m/s²
    public static final double MAX_ANGULAR_SPEED = 540.0; // deg/s
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540); // rad/s²
    
    // Field positions
    public static final Pose2d LEFT_FEEDER_POSE = new Pose2d(1.14, 6.93, Rotation2d.fromDegrees(127.16));
    public static final Pose2d ALGAE_SCORE_POSE = new Pose2d(5.98, .58, Rotation2d.fromDegrees(-90));
  }
}
