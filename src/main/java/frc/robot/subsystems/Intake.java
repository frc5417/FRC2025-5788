// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final SparkMax wristMotor;
  private final SparkMax spinnerMotor;

  private final SparkMaxConfig wristConfig;
  private final SparkMaxConfig spinnerConfig;
 
  private SparkClosedLoopController wristController;

  /** Creates a new Intake. */
  public Intake() {
    wristMotor = new SparkMax(Constants.IntakeConstants.wristSparkMax, MotorType.kBrushless);
    wristConfig = new SparkMaxConfig();

    wristConfig
      .inverted(Constants.IntakeConstants.wristInverted)
      .smartCurrentLimit(Constants.IntakeConstants.wristCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .encoder.positionConversionFactor(1/12);

    wristConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1, 1)
      .pid(Constants.IntakeConstants.wristKP, Constants.IntakeConstants.wristKI, Constants.IntakeConstants.wristKD);

    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, null);
    wristController = wristMotor.getClosedLoopController();

    spinnerMotor = new SparkMax(Constants.IntakeConstants.spinnerSparkMax, MotorType.kBrushless);
    spinnerConfig = new SparkMaxConfig();

    spinnerConfig
      .inverted(Constants.IntakeConstants.spinnerInverted)
      .smartCurrentLimit(Constants.IntakeConstants.spinnerCurrentLimit)
      .idleMode(IdleMode.kBrake);

    spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWristPosition(double position) {
    wristController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setSpinnerPower(double power) {
    spinnerMotor.set(power);
  }
}
