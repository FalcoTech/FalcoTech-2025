// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final SparkMax LeftClimbMotor = new SparkMax(50, MotorType.kBrushless);
  private final SparkMax RightClimbMotor = new SparkMax(51, MotorType.kBrushless);

  private SparkMaxConfig LeftClimbMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig RightClimbMotorConfig = new SparkMaxConfig();



  /** Creates a new Climb. */
  public Climb() {
	LeftClimbMotorConfig.idleMode(IdleMode.kBrake);
	RightClimbMotorConfig.idleMode(IdleMode.kBrake);

	RightClimbMotorConfig.follow(50, true);

	LeftClimbMotor.configure(LeftClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	RightClimbMotor.configure(RightClimbMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command RunClimbCommand(Supplier<Double> speed){
	return this.startEnd(() -> LeftClimbMotor.set(speed.get()), () -> LeftClimbMotor.set(0));
  }

}
