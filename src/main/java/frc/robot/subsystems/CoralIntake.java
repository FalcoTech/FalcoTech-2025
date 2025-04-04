// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  //talonFX motor with ID 42
  private final TalonFX CoralIntakeMotor = new TalonFX(42);
  public CoralIntake() {
    //set to brake mode
    CoralIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Intake Speed", this.getSpeed());

  }

  public void setCoralIntakePosition(double rotations){
    CoralIntakeMotor.setPosition(rotations);
  }

  public double GetCoralIntakePosition(){
    return CoralIntakeMotor.getPosition().getValueAsDouble();
  }

  //function to run coral intake
  public void RunCoralIntake(Supplier<Double> speed){
    CoralIntakeMotor.set(speed.get()*1);
  }
  //function to stop coral intake 
  public void StopCoralIntake(){
    CoralIntakeMotor.set(0);
  }

  public double getSpeed() {
    return CoralIntakeMotor.getVelocity().getValueAsDouble();
    
  }

}
