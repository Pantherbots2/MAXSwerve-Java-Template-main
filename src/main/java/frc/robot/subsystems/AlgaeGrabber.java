// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeGrabber extends SubsystemBase  {

  // Initialize the motor (Flex/MAX are setup the same way)
  private SparkMax grabber  = new SparkMax(10, MotorType.kBrushed);
  double reference = 0;

  /** Creates a new Elevator. */
  public AlgaeGrabber() {           
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    grabber.set(speed);
  }
  
}