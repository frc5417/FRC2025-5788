// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Passage extends SubsystemBase {
  private final SparkMax spinnerMotor;

  private final SparkMaxConfig spinnerConfig;

  /** Creates a new Passage. */
  public Passage() {
    spinnerMotor = new SparkMax(Constants.PassageConstants.spinnerSparkMax, SparkMax.MotorType.kBrushless);
    spinnerConfig = new SparkMaxConfig();

    spinnerConfig
      .inverted(Constants.PassageConstants.spinnerInverted)
      .smartCurrentLimit(Constants.PassageConstants.spinnerCurrentLimit)
      .idleMode(IdleMode.kCoast);

    spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpinnerSpeed(double speed) {
    spinnerMotor.set(speed);
  }
}
