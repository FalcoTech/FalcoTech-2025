// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Measure.*;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
  private final SparkMax LeftElevatorMotor = new SparkMax(20, MotorType.kBrushless);
  private final SparkMax RightElevatorMotor = new SparkMax(21, MotorType.kBrushless);

  private SparkMaxConfig LeftElevatorConfig = new SparkMaxConfig();
  private SparkMaxConfig RightElevatorConfig = new SparkMaxConfig();

  private final RelativeEncoder LeftElevatorEncoder = LeftElevatorMotor.getEncoder();
  private final RelativeEncoder RightElevatorEncoder = RightElevatorMotor.getEncoder();
  
  private final PIDController ElevatorPID = new PIDController(.2, 0, 0); //kP = .1
  private final ElevatorFeedforward ElevatorFF = new ElevatorFeedforward(0, .06, 0, 0); //kG = .1

  private SparkClosedLoopController ElevatorClosedLoopController = LeftElevatorMotor.getClosedLoopController();
  public double currentElevatorTarget = 0; // TODO: Set height to home position

  // private MutVoltage appliedVolts = Volts.mutable(0);
  // private Distance measu_pos = Inches.mutable(0);
  // private MutLinearVelocity measu_vel = InchesPerSecond.mutable(0);
  // // SysId objects
  // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(),
  //   new SysIdRoutine.Mechanism(
  //     // Apply voltage to motors
  //     (appliedVolts) -> LeftElevatorMotor.setVoltage(appliedVolts),
  //     // Log data
  //     log -> {
  //       log.motor("elevator-left")
  //         .voltage(appliedVolts)
  //         .linearPosition(GetLeftElevatorDistance())
  //         .linearVelocity(GetLeftElevatorVelocity());
  //     },
  //     this
  //   )
  // );


  /** Creates a new Elevator. */
  public Elevator() {
    // LeftElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // RightElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    LeftElevatorConfig.idleMode(IdleMode.kBrake);
    RightElevatorConfig.idleMode(IdleMode.kBrake);

    RightElevatorConfig.follow(20, true);

    LeftElevatorConfig.encoder.positionConversionFactor(2.206); // Converts 1 rotation of the encoder to 2.206 inches of elevator height
    LeftElevatorConfig.closedLoop
    .maxOutput(0.3)
    .minOutput(-0.3)
    .p(0)
    .i(0)
    .d(0)
      .maxMotion
      .maxVelocity(1) // TODO: Set to max velocity of elevator
      .maxAcceleration(10) //TODO: Set max accel of elevator
      .allowedClosedLoopError(0.25); //TODO: Set to allowed tolerance of elevator
   
    LeftElevatorMotor.configure(LeftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightElevatorMotor.configure(RightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    ResetElevatorEncoders();  

    SmartDashboard.putData("Reset Elevator Encoder", new InstantCommand(() -> ResetElevatorEncoders()).ignoringDisable(true));
    SmartDashboard.putBoolean("Hold Elevator Position", false);
    SmartDashboard.putNumber("ElevatorMaxMotion P Gain", 0);
    SmartDashboard.putNumber("Elevator Target", currentElevatorTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Elevator Encoder", GetLeftElevatorPosition());
    SmartDashboard.putNumber("Right Elevator Encoder", GetRightElevatorPosition());
    SmartDashboard.putNumber("Elevator Motor Output", LeftElevatorMotor.getAppliedOutput());

    // Smart Dashboard button to toggle holding at the setpoint using the MoveElevatorToMaxMotion method
    if (SmartDashboard.getBoolean("Hold Elevator Position", false)) {
      MoveElevatorToPositionMaxMotion(currentElevatorTarget);
    }
    else{
      StopElevator();
    }
    currentElevatorTarget = SmartDashboard.getNumber("Elevator Target", currentElevatorTarget);
    // SmartDashboard for editing the P value of the sparkmax pid controller
    LeftElevatorConfig.closedLoop.p(SmartDashboard.getNumber("ElevatorMaxMotion P Gain", 0));
    LeftElevatorMotor.configure(LeftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void MoveElevator(Supplier<Double> speed){
    LeftElevatorMotor.set(speed.get());
  }

  public void MoveElevatorToPositionMaxMotion(double position){
    currentElevatorTarget = position;
    ElevatorClosedLoopController.setReference(currentElevatorTarget, ControlType.kMAXMotionPositionControl);
  }

  public void MoveElevatorToPosition(double position){
    double PIDOutput = ElevatorPID.calculate(GetLeftElevatorPosition(), position) + ElevatorFF.calculate(position);
    double CommandedOutput = Math.copySign(Math.min(Math.abs(PIDOutput), .2), PIDOutput);
    LeftElevatorMotor.set(CommandedOutput);
    // SmartDashboard.putNumber("Elevator Motor Output", ElevatorPID.calculate(GetLeftElevatorPosition(), position) + ElevatorFF.calculate(1, 1));
  }

  public void StopElevator(){
    LeftElevatorMotor.set(0);
  }

  public double GetLeftElevatorPosition(){
    return LeftElevatorEncoder.getPosition();
  }
  public Distance GetLeftElevatorDistance(){
    // convert position from encoder to mutDistance
    return Inches.mutable(LeftElevatorEncoder.getPosition());
  }
  public double GetRightElevatorPosition(){
    return RightElevatorEncoder.getPosition();
  }
  public void ResetElevatorEncoders(){
    LeftElevatorEncoder.setPosition(0);
    RightElevatorEncoder.setPosition(0);
  }
  public void SetElevatorTarget(double targetHeight){
    currentElevatorTarget = targetHeight;
  }

  // /**
  //  * Sets the voltage to both elevator motors.
  //  * Used for SysId testing.
  //  * @param volts The voltage to set
  //  */
  // public void setVoltage(double volts) {
  //   appliedVolts = Measure.of(volts, Units.Volts);
  //   LeftElevatorMotor.setVoltage(volts);
  //   // Right motor follows left with inversion, so we don't need to set it directly
  // }

  // /**
  //  * Gets the velocity of the left elevator motor.
  //  * @return Velocity in inches per second
  //  */
  // public LinearVelocity GetLeftElevatorVelocity() {
  //   return LeftElevatorEncoder.getVelocity() / 60.0; // Convert RPM to RPS (and then in/s due to conversion factor)
  // }

  // /**
  //  * Command to run a SysId quasistatic test (gradually ramping voltage)
  //  * @param direction Direction of the test (true = forward, false = backward)
  //  * @return The SysId quasistatic test command
  //  */
  // public Command sysIdQuasistatic(boolean direction) {
  //   return sysIdRoutine.quasistatic(direction ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
  // }

  // /**
  //  * Command to run a SysId dynamic test (applying constant voltage)
  //  * @param direction Direction of the test (true = forward, false = backward)
  //  * @return The SysId dynamic test command
  //  */
  // public Command sysIdDynamic(boolean direction) {
  //   return sysIdRoutine.dynamic(direction ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
  // }
}