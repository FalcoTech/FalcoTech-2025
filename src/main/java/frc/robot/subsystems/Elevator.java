// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX LeftElevatorMotor = new TalonFX(20);
  private final TalonFX RightElevatorMotor = new TalonFX(21);
  private TalonFXConfiguration LeftElevatorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration RightElevatorConfig = new TalonFXConfiguration();
  private final PIDController ElevatorPID = new PIDController(0, 0, 0);
  /** Creates a new Elevator. */
  public Elevator() {
    LeftElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    RightElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    RightElevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    LeftElevatorMotor.getConfigurator().apply(LeftElevatorConfig);
    RightElevatorMotor.getConfigurator().apply(RightElevatorConfig);

    RightElevatorMotor.setControl(new Follower(LeftElevatorMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveElevator(double speed){
    LeftElevatorMotor.set(speed);
  }
  
}
