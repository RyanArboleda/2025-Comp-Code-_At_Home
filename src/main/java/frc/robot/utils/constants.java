// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public interface constants {

    
  interface elevator {
    int rightElevatorMotorID = 0;
    int leftElevatorMotorID = 1;

    public static final SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();    
    


    public static final double kDown = 0.0;
    public static final double kLVL1 = 3.05;
    public static final double kLVL2 = 4.52;
    public static final double kLVL3 = 2.95;
    
 }

}
