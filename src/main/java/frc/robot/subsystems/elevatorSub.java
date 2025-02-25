// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.constants;

public class elevatorSub extends SubsystemBase implements constants.elevator{
  /** Creates a new elevatorSub. */
  SparkMax rightElevatorMotor, leftElevatorMotor;
  SparkClosedLoopController elevatorClosedLoopController;
  AbsoluteEncoder encoder;
  public double kP = 0.5;
  public double kD = 0.0;

  

  
  public elevatorSub() {
    SparkMax rightElevatorMotor = new SparkMax(rightElevatorMotorID, MotorType.kBrushless);
    SparkMax leftElevatorMotor = new SparkMax(leftElevatorMotorID, MotorType.kBrushless);
    encoder = rightElevatorMotor.getAbsoluteEncoder();
    SparkClosedLoopController elevatorClosedLoopController = rightElevatorMotor.getClosedLoopController();

    


    

    rightElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    leftElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
    leftElevatorConfig.follow(rightElevatorMotorID, true);
    rightElevatorConfig.inverted(false);

    rightElevatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(kP)
    .d(kD)
    .outputRange(-0.5, 0.5);
    
    

    rightElevatorMotor.configure(rightElevatorConfig
    ,ResetMode.kResetSafeParameters
    ,PersistMode.kPersistParameters
    );

    leftElevatorMotor.configure(leftElevatorConfig
    ,ResetMode.kResetSafeParameters
    ,PersistMode.kPersistParameters
    );

  }

  public void pidSetPosition(double pos){
    elevatorClosedLoopController.setReference(pos, ControlType.kPosition);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
