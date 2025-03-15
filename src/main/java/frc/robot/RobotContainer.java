// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.AlgaeElevator;
import frc.robot.subsystems.AlgaeGrabber;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private final PS4Controller m_driverController = new PS4Controller(0);
    private final CommandXboxController m_OperatorController = new CommandXboxController(1);


    /* Subsystems */
    private final CoralArm coralArm = new CoralArm();
    private final CoralElevator coralElevator = new CoralElevator();
    private final Climber climber = new Climber();
    private final AlgaeElevator algaeElevator = new AlgaeElevator();
    private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();

     /* Operator Key Bindings */
     private final Trigger operatorLeftStickUp = new Trigger(()->m_OperatorController.getRawAxis(XboxController.Axis.kLeftY.value) > .3);
     private final Trigger operatorLeftStickDown = new Trigger(()->m_OperatorController.getRawAxis(XboxController.Axis.kLeftY.value) < -.3);
     private final Trigger operatorRightStickUp = new Trigger(()->m_OperatorController.getRawAxis(XboxController.Axis.kRightY.value) > .3);
     private final Trigger operatorRightStickDown = new Trigger(()->m_OperatorController.getRawAxis(XboxController.Axis.kRightY.value) < -.3);
     private final Trigger operatorRightTrigger = new Trigger(()-> m_OperatorController.getRawAxis(XboxController.Axis.kRightTrigger.value) > .1);
     private final Trigger operatorLeftTrigger = new Trigger(()-> m_OperatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value) > .1);
     
     
     public class LinearServoL16 {

        private Servo l16Servo;
        private final double MAX_PWM = 2.0;
        private final double MIN_PWM = 1.0;

        public LinearServoL16(int 0);
            l16Servo = new Servo(0);
        l16Servo.setBounds(MAX_PWM, 1.8, 1.5, 1.2, MIN_PWM);
    }

    public void setPosition(double Position) {
        l16Servo.set(Position);
    }
/*   Make the actuator extend */
    public void extend() {
        m_OperatorController.rightTrigger().onTrue(Position.set,1));
            l16Servo.set(2.0);
        m_OperatorController.rightTrigger().onfalse(0);
            l16Servo.set(1);
    }

/*    Make the Acutator Retract */
      m_OperatorController.leftTrigger().onTrue(1);
      l16Servo.set(0);
      m_OperatorController.leftTrigger().onFalse(0);
      l16Servo.set(1);
    }


 
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    //-----------------------------------------------
    // Operator Controls
    //-----------------------------------------------
    // Algae Elevator     
    operatorLeftStickUp.onTrue(new SetElevatorSpeed(algaeElevator, -1));  
    operatorLeftStickUp.onFalse(new SetElevatorSpeed(algaeElevator, 0));

    operatorLeftStickDown.onTrue(new SetElevatorSpeed(algaeElevator, 1) );
    operatorLeftStickDown.onFalse(new SetElevatorSpeed(algaeElevator, 0));

    // Algae Grabber
        //Grab
      m_OperatorController.rightBumper().onTrue(new GrabberSequenceAlgae(algaeGrabber, -1));
      m_OperatorController.rightBumper().onFalse(new GrabberSequenceAlgae(algaeGrabber, 0));
      // Release
      m_OperatorController.leftBumper().onTrue(new GrabberSequenceAlgae(algaeGrabber, 1));
      m_OperatorController.leftBumper().onFalse(new GrabberSequenceAlgae(algaeGrabber, 0));

    // Climber
    operatorRightStickUp.onTrue(new SetClimberSpeed(climber, -.2));  
    operatorRightStickUp.onFalse(new SetClimberSpeed(climber, 0));

    operatorRightStickDown.onTrue(new SetClimberSpeed(climber, .2) );
    operatorRightStickDown.onFalse(new SetClimberSpeed(climber, 0));
       
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    //--------------------------------------------------------------------------------------------------
    /*                                           Controls                                             */
    //--------------------------------------------------------------------------------------------------
    //-----------------------------------------------
    // Driver Controls
    //-----------------------------------------------
    // Coral Arm
 /*   new JoystickButton(m_driverController, Button.kCircle.value)
        .whileTrue(new StartEndCommand(
            () -> coralArm.setMotorVoltage(4),
            () -> coralArm.stopMotor(),
            coralArm));
    new JoystickButton(m_driverController, Button.kSquare.value)
        .whileTrue(new StartEndCommand(
            () -> coralArm.setMotorVoltage(-4),
            () -> coralArm.stopMotor(),
            coralArm));
    // Coral Elevator
    new JoystickButton(m_driverController, Button.kTriangle.value)
    .whileTrue(new StartEndCommand(
        () -> coralElevator.setMotorVoltage(-12),
        () -> coralElevator.stopMotor(),
        coralElevator));
    new JoystickButton(m_driverController, Button.kCross.value)
        .whileTrue(new StartEndCommand(
            () -> coralElevator.setMotorVoltage(12),
            () -> coralElevator.stopMotor(),
            coralElevator));   
    }
*/

    m_OperatorController.b().whileTrue(new StartEndCommand(
        () -> coralArm.setMotorVoltage(4),
        () -> coralArm.stopMotor(),
        coralArm));
    m_OperatorController.x().whileTrue(new StartEndCommand(
            () -> coralArm.setMotorVoltage(-4),
            () -> coralArm.stopMotor(),
            coralArm));
    // Coral Elevator
    m_OperatorController.y().whileTrue(new StartEndCommand(
        () -> coralElevator.setMotorVoltage(-12),
        () -> coralElevator.stopMotor(),
        coralElevator));
    m_OperatorController.a().whileTrue(new StartEndCommand(
            () -> coralElevator.setMotorVoltage(12),
            () -> coralElevator.stopMotor(),
            coralElevator));   
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new DriveForward(m_robotDrive, 0.07);
    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  */
}
}
