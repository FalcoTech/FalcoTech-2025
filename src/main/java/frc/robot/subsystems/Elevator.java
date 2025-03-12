// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax LeftElevatorMotor = new SparkMax(20, MotorType.kBrushless);
  private final SparkMax RightElevatorMotor = new SparkMax(21, MotorType.kBrushless);

  private SparkMaxConfig LeftElevatorConfig = new SparkMaxConfig();
  private SparkMaxConfig RightElevatorConfig = new SparkMaxConfig();

  private final RelativeEncoder LeftElevatorEncoder = LeftElevatorMotor.getEncoder();
  private final RelativeEncoder RightElevatorEncoder = RightElevatorMotor.getEncoder();
  
  private final PIDController ElevatorPID = new PIDController(0, 0, 0);
  private final ElevatorFeedforward ElevatorFF = new ElevatorFeedforward(0, 0, 0, 0);
  /** Creates a new Elevator. */
  public Elevator() {
    // LeftElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // RightElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    LeftElevatorConfig.idleMode(IdleMode.kBrake);
    RightElevatorConfig.idleMode(IdleMode.kBrake);

    RightElevatorConfig.follow(20, true);

    // RightElevatorConfig.follow(LeftElevatorMotor, true);    
    LeftElevatorMotor.configure(LeftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightElevatorMotor.configure(RightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Elevator Encoder", GetLeftElevatorPosition());
    SmartDashboard.putNumber("Right Elevator Encoder", GetRightElevatorPosition());
  }

  public void MoveElevator(Supplier<Double> speed){
    LeftElevatorMotor.set(speed.get());
  }

  public double GetLeftElevatorPosition(){
    return LeftElevatorEncoder.getPosition();
  }
  public double GetRightElevatorPosition(){
    return RightElevatorEncoder.getPosition();
  }
  public void ResetEncoder(){
    LeftElevatorEncoder.setPosition(0);
    RightElevatorEncoder.setPosition(0);
  }
  
}
