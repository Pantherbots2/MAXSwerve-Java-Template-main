// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase  {

  // Initialize the motor (Flex/MAX are setup the same way)
  private SparkMax Climber  = new SparkMax(14, MotorType.kBrushed);
  double reference = 0;

  /** Creates a new Elevator. */
  public Climber() {           
  }

  public void setSpeed(double speed){
    Climber.set(speed);
  }
  
  // new methods from PSU
  public void setMotorVoltage(double volts) {
    Climber.setVoltage(volts);
  }

  public void stopMotor() {
    Climber.stopMotor();
  }
  
}