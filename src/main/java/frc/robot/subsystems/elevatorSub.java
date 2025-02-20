// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.constants;

public class elevatorSub extends SubsystemBase implements constants.elevator{
  /** Creates a new elevatorSub. */
  SparkMax rightElevatorMotor, leftElevatorMotor;
  SparkClosedLoopController elevatorClosedLoopController;
  AbsoluteEncoder encoder;
  public elevatorSub() {
    SparkMax rightElevatorMotor = new SparkMax(rightElevatorMotorID, MotorType.kBrushless);
    SparkMax leftElevatorMotor = new SparkMax(leftElevatorMotorID, MotorType.kBrushless);
    encoder = rightElevatorMotor.getAbsoluteEncoder();

    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(1)
    .outputRange(-1, 1)
    .maxMotion
    .maxVelocity(4200)
    .maxAcceleration(6000)
    .allowedClosedLoopError(0.5);

    rightElevatorMotor.configure(elevatorConfig
    ,ResetMode.kResetSafeParameters
    ,PersistMode.kPersistParameters
    );

    

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
