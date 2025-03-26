// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Algae.RunAlgaeIntake;
import frc.robot.commands.Coral.CenterCoral;
import frc.robot.commands.Coral.RunCoralIntake;
import frc.robot.commands.Elevator.RunElevator;
import frc.robot.commands.Elevator.SequentialElevatorSetpoint;
import frc.robot.commands.Elevator.SetElevatorToPosition;
import frc.robot.commands.Swerve.RumbleCommand;
import frc.robot.commands.Swerve.TeleOpDrive;
import frc.robot.commands.Wrist.RunWrist;
import frc.robot.commands.Wrist.SequentialWristSetpoint;
import frc.robot.commands.Wrist.SetWristToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlignmentSystem;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    /* Initialize Game Controllers */
    public static final CommandXboxController pilot = new CommandXboxController(0);
    public static final CommandXboxController Copilot = new CommandXboxController(1);

    /* Set max speeds */
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);// kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up control commands of the swerve drive */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //PATHFINDING TEST
    private final Pose2d leftFeederTargetPose = new Pose2d(1.14, 6.93, Rotation2d.fromDegrees(127.16));
    private final Pose2d algaeScoreTargerPose = new Pose2d(5.98, .58, Rotation2d.fromDegrees(-90));

    public static final PathConstraints pathFindConstraints = new PathConstraints(MaxSpeed, 4, MaxAngularRate, Units.degreesToRadians(540));
    private final Command leftFeederPathfind = AutoBuilder.pathfindToPose(leftFeederTargetPose, pathFindConstraints, 1);
    private final Command algaeScorePathfind = AutoBuilder.pathfindToPose(algaeScoreTargerPose, pathFindConstraints, 1);
    
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
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // SmartDashboard.putData("Align to Tag", new AlignToNearestTagWithOffset(false));
        // SmartDashboard.putData("Algae Intake Pathfind", algaeScorePathfind);
        SmartDashboard.putData("Pathfind to Nearest AprilTag", new InstantCommand(() -> tagAlign.pathfindToNearestAprilTagOld(false).schedule()));

        configureBindings();
        RegisterNamedCommands();
    }

    private void configureBindings() {
        /* Swerve Drive */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new TeleOpDrive());

        // reset the field-centric heading on start button press
        pilot.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        pilot.povRight().whileTrue(tagAlign.pathfindToNearestAprilTag(true));
        pilot.povLeft().whileTrue(tagAlign.pathfindToNearestAprilTag(false));
        
        drivetrain.registerTelemetry(logger::telemeterize);
          
        // pilot.a().whileTrue(leftFeederPathfind);
        // pilot.b().whileTrue(algaeScoreCommand);
        // pilot.back().and(pilot.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); // pilot.back().and(pilot.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); // pilot.start().and(pilot.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // pilot.start().and(pilot.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //TODO
        // 1 Rotation at top of elevator = 5.5 in of linear movement or 11.03 in of movement at claw
        // Max travel is 29in 

        //ELEVATOR
        elevator.setDefaultCommand(new RunElevator(() -> Math.abs(Copilot.getRightY() * .5)));
        Copilot.start().onTrue(new InstantCommand(() -> elevator.ResetElevatorEncoders()));
        
        
        //ALGAE INTAKE
        Copilot.leftBumper().whileTrue(new RunAlgaeIntake(() -> 1.0));
        Copilot.rightBumper().whileTrue(new RunAlgaeIntake(() -> -.2)).onFalse(new RunAlgaeIntake(() -> 1.0).withTimeout(1));


        //CORAL INTAKE
        coralIntake.setDefaultCommand(new RunCoralIntake(()-> Copilot.getLeftTriggerAxis()-Copilot.getRightTriggerAxis()));
        // Copilot.povRight().whileTrue(new CenterCoral());

        //WRIST
        wrist.setDefaultCommand(new RunWrist(() -> Copilot.getLeftY()));
        //Climb
        Copilot.povLeft().whileTrue(climb.RunClimbCommand(() -> 0.25));
        Copilot.povRight().whileTrue(climb.RunClimbCommand(() -> -0.25));
        

        //ELEVATOR SETPOINTS
        //
        Copilot.povDown().onTrue(
            new SequentialCommandGroup(
                elevator.GetLeftElevatorPosition() > 1 && elevator.GetLeftElevatorPosition() < 4.5 && wrist.GetWristEncoderPosition() > 18 ? 
                   new SequentialCommandGroup(
                       new SequentialElevatorSetpoint(6), 
                       new ParallelDeadlineGroup(new SetWristToPosition(0), new SetElevatorToPosition(6)),
                       new InstantCommand(() -> elevator.StopElevator())
                   ) : 
                   new ParallelDeadlineGroup(
                       new SetWristToPosition(0),
                       new SequentialElevatorSetpoint(0)
                   )
            )
        );
        
        Copilot.x().onTrue(new SequentialCommandGroup( //L2 SCORING
            new SequentialElevatorSetpoint(8),
            new ParallelCommandGroup(
                new SetElevatorToPosition(8),
                new SetWristToPosition(5.8)
            )
        ));

        Copilot.y().onTrue(new SequentialCommandGroup( //L3 SCORING
            new SequentialElevatorSetpoint(14.8),
            new ParallelCommandGroup(
                new SetElevatorToPosition(14.8),
                new SetWristToPosition(5.8)
            )
        ));

        Copilot.a().and(Copilot.x()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(12),
            new SetWristToPosition(12.5) //check this value
        ));
        Copilot.a().and(Copilot.y()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(14.6),
            new SetWristToPosition(10) //check this value
        ));
        Copilot.a().and(Copilot.b()).onTrue(new ParallelCommandGroup(
            new SetElevatorToPosition(24.7),
            new SetWristToPosition(3) //check this value
        ));

        // Copilot.povRight().onTrue(new SequentialCommandGroup( //L4 SCORING VALUES
        // new SequentialElevatorSetpoint(20),
        // new ParallelCommandGroup(
        //     new SetElevatorToPosition(20),
        //     new SetWristToPosition(5.8) 
        // )
        // ));

        Copilot.povUp().onTrue(new SequentialCommandGroup( //CORAL STATION VALUES
            new SequentialElevatorSetpoint(12.8),
            new ParallelCommandGroup(
                new SetElevatorToPosition(12.8),
                new SetWristToPosition(20.1) 
            )
        ));

        Copilot.back().onTrue(new SequentialCommandGroup(new RunCoralIntake(() -> -.2).withTimeout(.15), new RunCoralIntake(() -> .2).withTimeout(.2), new RunCoralIntake(() -> -.2).withTimeout(.15), new RunCoralIntake(() -> .2).withTimeout(.2), new RunCoralIntake(() -> 0.0).withTimeout(0.15)));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private void RegisterNamedCommands(){
        /* Pathplanner named commands for the pathplanner app. TODO: make this a function */
        NamedCommands.registerCommand("TestCommand", algaeScorePathfind);

        NamedCommands.registerCommand("Elevator L3 Score", new SequentialCommandGroup(
            new SequentialElevatorSetpoint(14.8),
            new ParallelRaceGroup(
                new SetElevatorToPosition(14.8),
                new SequentialWristSetpoint(5.8)
            ),
            new ParallelRaceGroup(
                new SetElevatorToPosition(14.8),
                new SetWristToPosition(5.8),
                new RunCoralIntake(() -> 0.3).withTimeout(2)
            ),
            new SequentialWristSetpoint(0)
        ));
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