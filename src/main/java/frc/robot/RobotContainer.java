// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RunElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    /* Initialize Game Controllers */
    // private final CommandPS4Controller joystick = new CommandPS4Controller(0);
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController Copilot = new CommandXboxController(1);

    /* Set max speeds */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);// kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up control commands of the swerve drive */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //PATHFINDING TEST
    private final Pose2d leftFeederTargetPose = new Pose2d(1.14, 6.93, Rotation2d.fromDegrees(127.16));
    private final Pose2d algaeScoreTargerPose = new Pose2d(5.98, .58, Rotation2d.fromDegrees(-90));

    private final PathConstraints pathFindConstraints = new PathConstraints(MaxSpeed, 4, MaxAngularRate, Units.degreesToRadians(540));
    private final Command leftFeederPathfind = AutoBuilder.pathfindToPose(leftFeederTargetPose, pathFindConstraints, 1);
    private final Command algaeScoreCommand = AutoBuilder.pathfindToPose(algaeScoreTargerPose, pathFindConstraints, 1);
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public static final Elevator elevator = new Elevator();

    public RobotContainer() {
        /* Put autonomous chooser on dashboard */
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        /* Pathplanner named commands for the pathplanner app. TODO: make this a function */
        NamedCommands.registerCommand("TestCommand", algaeScoreCommand);
        configureBindings();
    }

    private void configureBindings() {
        /* Swerve Drive */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getLeftY()) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX()) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX()) // Drive counterclockwise with negative X (left)
            )
        );

        pilot.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
            driveRobotCentric.withVelocityX(-pilot.getLeftY())
                .withVelocityY(-pilot.getLeftX())
                .withRotationalRate(-pilot.getRightX())
        ));
  
        // pilot.a().whileTrue(leftFeederPathfind);
        // pilot.b().whileTrue(algaeScoreCommand);

        // reset the field-centric heading on left bumper press
        pilot.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        pilot.back().and(pilot.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        pilot.back().and(pilot.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        pilot.start().and(pilot.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        pilot.start().and(pilot.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        //TODO
        // Add CoralIntake, AlgaeIntake, Claw
        // 1 Rotation at top of elevator = 5.5 in of linear movement or 11.03 in of movement at claw
        // Max travel is 29in 
        


        //ELEVATOR
        elevator.setDefaultCommand(new RunElevator(() -> -Copilot.getRightY() * .5));
        Copilot.start().onTrue(new InstantCommand(() -> elevator.ResetEncoder()));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
