// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Algae.RunAlgaeIntake;
import frc.robot.commands.Coral.RunCoralIntake;
import frc.robot.commands.Elevator.RunElevator;
import frc.robot.commands.Elevator.SequentialElevatorSetpoint;
import frc.robot.commands.Elevator.SetElevatorToPosition;
import frc.robot.commands.Swerve.TeleOpDrive;
import frc.robot.commands.Wrist.RunWrist;
import frc.robot.commands.Wrist.SequentialWristSetpoint;
import frc.robot.commands.Wrist.SetWristToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlignmentSystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

import static frc.robot.Constants.*;

public class RobotContainer {
    /* Initialize Game Controllers */
    public static final CommandXboxController pilot = new CommandXboxController(ControllerConstants.PILOT_CONTROLLER_PORT);
    public static final CommandXboxController Copilot = new CommandXboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);

    /* Set max speeds */
    public static double MaxSpeed = DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
    public static double MaxAngularRate = DrivetrainConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND;
    /* Setting up control commands of the swerve drive */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * ControllerConstants.DEADBAND)
            .withRotationalDeadband(MaxAngularRate * ControllerConstants.DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * ControllerConstants.DEADBAND).withRotationalDeadband(MaxAngularRate * ControllerConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //PATHFINDING TEST
    private final Pose2d leftFeederTargetPose = PathPlanningConstants.LEFT_FEEDER_POSE;
    private final Pose2d algaeScoreTargetPose = PathPlanningConstants.ALGAE_SCORE_POSE;

    public static final PathConstraints pathFindConstraints = new PathConstraints(
        PathPlanningConstants.MAX_PATH_SPEED, 
        PathPlanningConstants.MAX_PATH_ACCELERATION, 
        PathPlanningConstants.MAX_ANGULAR_SPEED * (Math.PI/180), 
        PathPlanningConstants.MAX_ANGULAR_ACCELERATION);
    private final Command leftFeederPathfind = AutoBuilder.pathfindToPose(leftFeederTargetPose, pathFindConstraints, 1);
    private final Command algaeScorePathfind = AutoBuilder.pathfindToPose(algaeScoreTargetPose, pathFindConstraints, 1);
    
    private final SendableChooser<Command> autoChooser;

    //Initialize subsystems
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Wrist wrist = new Wrist();
    public static final AlgaeIntake algaeIntake = new AlgaeIntake();
    public static final CoralIntake coralIntake = new CoralIntake();
    public static final Climb climb = new Climb();
    public static final AlignmentSystem tagAlign = new AlignmentSystem(drivetrain);

    public RobotContainer() {
        /* Put autonomous chooser on dashboard */
        RegisterNamedCommands();
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // SmartDashboard.putData("Align to Tag", new AlignToNearestTagWithOffset(false));
        // SmartDashboard.putData("Algae Intake Pathfind", algaeScorePathfind);
        SmartDashboard.putData("Pathfind to Nearest AprilTag", new InstantCommand(() -> tagAlign.pathfindToNearestAprilTagOld(false).schedule()));

        configureBindings();

        SmartDashboard.putBoolean("Enable MegaTag2", false);
    }

    private void configureBindings() {
        /* Swerve Drive */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new TeleOpDrive());

        // reset the field-centric heading on start button press
        pilot.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        pilot.b().whileTrue(tagAlign.pathfindToNearestCoralReefAprilTag(true));
        pilot.x().whileTrue(tagAlign.pathfindToNearestCoralReefAprilTag(false));
        pilot.y().whileTrue(tagAlign.pathfindToNearestAlgaeReefAprilTag());
        pilot.a().whileTrue(tagAlign.pathfindToNearestAlgaeProcAprilTag());
        pilot.povUp().whileTrue(tagAlign.pathfindToNearestCoralStationAprilTag());
        pilot.povDown().whileTrue(tagAlign.pathfindToNearestBargeAprilTag());
        
        drivetrain.registerTelemetry(logger::telemeterize);
          
        // pilot.a().whileTrue(leftFeederPathfind);
        // pilot.b().whileTrue(algaeScoreCommand);
        // pilot.back().and(pilot.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); // pilot.back().and(pilot.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); // pilot.start().and(pilot.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // pilot.start().and(pilot.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //TODO
        // 1 Rotation at top of elevator = 5.5 in of linear movement or 11.03 in of movement at claw
        // Max travel is 29in 

        //ELEVATOR
        elevator.setDefaultCommand(new RunElevator(() -> Math.abs(Copilot.getRightY() * ElevatorConstants.ELEVATOR_CONTROL_SCALE)));
        Copilot.start().onTrue(new InstantCommand(() -> elevator.ResetElevatorEncoders()));
        
        
        //ALGAE INTAKE
        Copilot.leftBumper().whileTrue(new RunAlgaeIntake(() -> IntakeConstants.ALGAE_INTAKE_SPEED));
        Copilot.rightBumper().whileTrue(new RunAlgaeIntake(() -> IntakeConstants.ALGAE_OUTTAKE_SPEED)).onFalse(new RunAlgaeIntake(() -> IntakeConstants.ALGAE_INTAKE_SPEED).withTimeout(1));

        Copilot.rightBumper().toggleOnTrue(new RunAlgaeIntake(() -> -.5));
        Copilot.rightBumper().toggleOnFalse(new RunAlgaeIntake(() -> 1.0).withTimeout(1));

        //CORAL INTAKE
        coralIntake.setDefaultCommand(new RunCoralIntake(()-> Copilot.getRightTriggerAxis()-Copilot.getLeftTriggerAxis()));
        // Copilot.povRight().whileTrue(new CenterCoral());

        //WRIST
        wrist.setDefaultCommand(new RunWrist(() -> Copilot.getLeftY()));
        //Climb
        pilot.povLeft().whileTrue(climb.RunClimbCommand(() -> ClimbConstants.CLIMB_SPEED));
        pilot.povRight().whileTrue(climb.RunClimbCommand(() -> -ClimbConstants.CLIMB_SPEED));
        

        //ELEVATOR SETPOINTS
        //
        Copilot.povDown().onTrue( //Home position (does work)
            new SequentialCommandGroup(
                elevator.GetLeftElevatorPosition() > ElevatorConstants.MIN_ELEVATOR_SAFETY_THRESHOLD && 
                elevator.GetLeftElevatorPosition() < ElevatorConstants.MAX_ELEVATOR_SAFETY_THRESHOLD && 
                wrist.GetWristEncoderPosition() > WristConstants.WRIST_SAFETY_THRESHOLD ? 
                   new SequentialCommandGroup(
                       new SequentialElevatorSetpoint(ElevatorConstants.ALGAE_PROCESSOR_POSITION), 
                       new ParallelDeadlineGroup(new SetWristToPosition(WristConstants.HOME_POSITION), new SetElevatorToPosition(ElevatorConstants.ALGAE_PROCESSOR_POSITION)),
                       new InstantCommand(() -> elevator.StopElevator())
                   ) : 
                   new ParallelDeadlineGroup(
                       new SetWristToPosition(WristConstants.HOME_POSITION),
                       new SequentialElevatorSetpoint(ElevatorConstants.HOME_POSITION)
                   )
            )
        );
        
        Copilot.x().onTrue(new SequentialCommandGroup( //L2 CORAL SCORING
            new SequentialElevatorSetpoint(ElevatorConstants.L2_SCORE_POSITION),
            new ParallelCommandGroup(
                new SetElevatorToPosition(ElevatorConstants.L2_SCORE_POSITION),
                new SetWristToPosition(WristConstants.L2_SCORE_POSITION)
            )
        ));

        Copilot.y().onTrue(new SequentialCommandGroup( //L3 SCORING
            new SequentialElevatorSetpoint(ElevatorConstants.L3_SCORE_POSITION),
            new ParallelCommandGroup(
                new SetElevatorToPosition(ElevatorConstants.L3_SCORE_POSITION),
                new SetWristToPosition(WristConstants.L3_SCORE_POSITION)
            )
        ));

        Copilot.b().onTrue(new SequentialCommandGroup( //L4 SCORING
             new SequentialElevatorSetpoint(ElevatorConstants.L4_SCORE_POSITION),
            new ParallelCommandGroup(
                new SetElevatorToPosition(ElevatorConstants.L4_SCORE_POSITION),
                new SetWristToPosition(WristConstants.L4_SCORE_POSITION)
            )
        ));

        Copilot.a().and(Copilot.x()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(ElevatorConstants.L2_ALGAE_POSITION),
            new SetWristToPosition(WristConstants.L2_ALGAE_POSITION) //L2 
        ));
        Copilot.a().and(Copilot.y()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(ElevatorConstants.L3_ALGAE_POSITION),
            new SetWristToPosition(WristConstants.L3_ALGAE_POSITION) //L3
        ));
        Copilot.a().and(Copilot.b()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(ElevatorConstants.BARGE_POSITION),
            new SetWristToPosition(WristConstants.BARGE_POSITION) //Barge
        ));
        Copilot.a().and(Copilot.povUp()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(ElevatorConstants.ALGAE_PROCESSOR_POSITION),
            new SetWristToPosition(WristConstants.ALGAE_PROCESSOR_POSITION) //Algae Processor
        ));
        Copilot.a().and(Copilot.povDown()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(ElevatorConstants.FLOOR_LOAD_POSITION),
            new SetWristToPosition(WristConstants.FLOOR_POSITION) //Algae Floor Load
        ));
    

        // Copilot.povRight().onTrue(new SequentialCommandGroup( //L4 SCORING VALUES
        // new SequentialElevatorSetpoint(ElevatorConstants.L4_SCORE_POSITION),
        // new ParallelCommandGroup(
        //     new SetElevatorToPosition(ElevatorConstants.L4_SCORE_POSITION),
        //     new SetWristToPosition(WristConstants.L4_SCORE_POSITION) 
        // )
        // ));

        Copilot.povUp().and(Copilot.a().negate()).onTrue(new SequentialCommandGroup( //CORAL STATION VALUES
            new SequentialElevatorSetpoint(ElevatorConstants.CORAL_STATION_POSITION),
            new ParallelCommandGroup(
                new SetElevatorToPosition(ElevatorConstants.CORAL_STATION_POSITION),
                new SetWristToPosition(WristConstants.CORAL_STATION_POSITION) 
            )
        ));

        Copilot.back().onTrue(new SequentialCommandGroup(new RunCoralIntake(() -> IntakeConstants.CORAL_OUTTAKE_SPEED).withTimeout(.15), new RunCoralIntake(() -> IntakeConstants.CORAL_INTAKE_SPEED).withTimeout(.15), new RunCoralIntake(() -> IntakeConstants.CORAL_OUTTAKE_SPEED).withTimeout(.15), new RunCoralIntake(() -> IntakeConstants.CORAL_INTAKE_SPEED).withTimeout(.15), new RunCoralIntake(() -> 0.0).withTimeout(0.15)));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private void RegisterNamedCommands(){
        /* Pathplanner named commands for the pathplanner app. TODO: make this a function */
        NamedCommands.registerCommand("TestCommand", algaeScorePathfind);

        NamedCommands.registerCommand("Elevator L3 Score", ElevatorL3Score());

        NamedCommands.registerCommand("Elevator L4 Score", ElevatorL4Score());
        NamedCommands.registerCommand("Drive to Nearest Right Reef", tagAlign.pathfindToNearestCoralReefAprilTag(true));
    }

    public Command ElevatorL3Score(){
        return new SequentialCommandGroup(
            new SequentialElevatorSetpoint(ElevatorConstants.L3_SCORE_POSITION),
            new ParallelRaceGroup(
                new SetElevatorToPosition(ElevatorConstants.L3_SCORE_POSITION),
                new SequentialWristSetpoint(WristConstants.L3_SCORE_POSITION)
            ),
            new ParallelRaceGroup(
                new SetElevatorToPosition(ElevatorConstants.L3_SCORE_POSITION),
                new SetWristToPosition(WristConstants.L3_SCORE_POSITION),
                new RunCoralIntake(() -> IntakeConstants.CORAL_INTAKE_SPEED).withTimeout(2)
            ),
            new SequentialWristSetpoint(WristConstants.HOME_POSITION)
        );
    }

    public Command ElevatorL4Score(){
        return new SequentialCommandGroup(
            new SequentialElevatorSetpoint(ElevatorConstants.L4_SCORE_POSITION),
            new ParallelRaceGroup(
                new SetElevatorToPosition(ElevatorConstants.L4_SCORE_POSITION),
                new SequentialWristSetpoint(WristConstants.L4_SCORE_POSITION)
            ),
            new ParallelRaceGroup(
                new SetElevatorToPosition(ElevatorConstants.L4_SCORE_POSITION),
                new SetWristToPosition(WristConstants.L4_SCORE_POSITION),
                new RunCoralIntake(() -> IntakeConstants.CORAL_INTAKE_SPEED).withTimeout(2)
            ),
            new SequentialWristSetpoint(WristConstants.HOME_POSITION)
        );
    }


    // public Pose2d to2dPose(Pose3d pose3d) {
    //     // Extract x and y components from the Translation3d
    //     Translation2d translation2d = new Translation2d(
    //         pose3d.getX(),
    //         pose3d.getY()
    //     );
        
    //     // Convert the rotation - we'll use the rotation around the Z axis
    //     Rotation2d rotation2d = new Rotation2d(pose3d.getRotation().getZ());
        
    //     // Create and return a new Pose2d
    //     return new Pose2d(translation2d, rotation2d);
    // }
}