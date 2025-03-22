// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final TalonFX WristMotor = new TalonFX(30);
  private TalonFXConfiguration WristMotorConfig = new TalonFXConfiguration();
  private TalonFXConfigurator WristMotorConfigurator = WristMotor.getConfigurator();

  private final DutyCycleEncoder WristEncoder = new DutyCycleEncoder(0, 40, 0);

  private final PIDController m_PIDController = new PIDController(.11, 0, 0);
  /** Creates a new Wrist. */
  public Wrist() {
    WristMotorConfigurator.apply(WristMotorConfig);
    WristMotor.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.putData("Reset Wrist Encoder", new InstantCommand(() -> ResetWristEncoder()).ignoringDisable(true));
    // SmartDashboard.putBoolean("Wrist Connected?", WristEncoder.isConnected());

    WristEncoder.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Wrist Motor Output", WristMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Wrist Encoder Value", GetWristEncoderPosition());
    SmartDashboard.putBoolean("Wrist Encoder Connected?", WristEncoder.isConnected());
  }
  
  public void MoveWrist(Supplier<Double> speed){
    WristMotor.set(speed.get() * .2);
  }
  public void MoveWristToPosition(double position){
    double PIDOutput = m_PIDController.calculate(GetWristEncoderPosition(), position);
    double CommandedOutput = Math.copySign(Math.min(Math.abs(PIDOutput), .2), PIDOutput);
    WristMotor.set(CommandedOutput);
    // SmartDashboard.putNumber("Wrist Motor Output", m_PIDController.calculate(GetWristEncoderPosition(), position));
  }

  public void StopWrist(){
    WristMotor.set(0);
  }

  public double GetWristEncoderPosition(){
    // return WristMotor.getPosition().getValueAsDouble();
    return WristEncoder.get() - .94;
  }

  public void ResetWristEncoder(){
    // WristMotor.setPosition(0);
  }
}
