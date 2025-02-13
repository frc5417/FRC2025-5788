// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private final SparkMaxConfig leftMotorConfig;
  private final SparkMaxConfig rightMotorConfig;

  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new SparkMax(Constants.ElevatorConstants.leftSparkMax, SparkMax.MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();

    leftMotorConfig
      .inverted(Constants.ElevatorConstants.leftInverted)
      .smartCurrentLimit(Constants.ElevatorConstants.leftCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .encoder.positionConversionFactor(1/12);

    leftMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1, 1)
      .pid(Constants.ElevatorConstants.leftKP, Constants.ElevatorConstants.leftKI, Constants.ElevatorConstants.leftKD);

    leftController = leftMotor.getClosedLoopController();

    rightMotor = new SparkMax(Constants.ElevatorConstants.rightSparkMax, SparkMax.MotorType.kBrushless);
    rightMotorConfig = new SparkMaxConfig();

    rightMotorConfig
      .inverted(Constants.ElevatorConstants.rightInverted)
      .smartCurrentLimit(Constants.ElevatorConstants.rightCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .encoder.positionConversionFactor(1/12);

    rightMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1, 1)
      .pid(Constants.ElevatorConstants.rightKP, Constants.ElevatorConstants.rightKI, Constants.ElevatorConstants.rightKD);

    rightController = rightMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPosition(double position) {
    leftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
