// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private final VictorSPX leftAlgaeIntake = new VictorSPX(40);
  private final VictorSPX RightAlgaeIntake = new VictorSPX(41);
  /** Creates a new AlgaeSubsystem. */
  public AlgaeIntake() {
    RightAlgaeIntake.setInverted(true);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  public void RunAlgaeIntake(double speed){
      leftAlgaeIntake.set(VictorSPXControlMode.PercentOutput, speed * .8);
      RightAlgaeIntake.set(VictorSPXControlMode.PercentOutput, speed * .8);
  }

  public void StopAlgaeIntake(){
     leftAlgaeIntake.set(VictorSPXControlMode.PercentOutput, 0);
      RightAlgaeIntake.set(VictorSPXControlMode.PercentOutput, 0);
  }
}