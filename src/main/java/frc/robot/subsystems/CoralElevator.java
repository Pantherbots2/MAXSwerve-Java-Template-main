// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralElevator extends SubsystemBase  {

  // Initialize the motor (Flex/MAX are setup the same way)
  private SparkMax Elevator  = new SparkMax(13, MotorType.kBrushed);
  double reference = 0;

  /** Creates a new Elevator. */
  public CoralElevator() {           
  }

  public void setSpeed(double speed){
    Elevator.set(speed);
  }
  
  // new methods from PSU
  public void setMotorVoltage(double volts) {
    Elevator.setVoltage(volts);
  }

  public void stopMotor() {
    Elevator.stopMotor();
  }
  
}