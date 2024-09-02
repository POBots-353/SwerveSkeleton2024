// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class Conversions {
  public double RPStoMPS(double rps) {
    return rps*(SwerveModuleConstants.WHEEL_CIRCUMFERENCE/(2*3.14));
  }

  public double MPStoRPS(double mps) {
    return mps/(SwerveModuleConstants.WHEEL_CIRCUMFERENCE/(2*3.14));
  }

  public double MechanismRotationsToDegrees(double mechanismRotations) {
    double totalCounts = mechanismRotations * SwerveModuleConstants.talonFXCPR;
    double angleInDegrees = totalCounts * SwerveModuleConstants.degreesPerCount;

    return angleInDegrees;
  }

  public double MechanismRotationsToMeters(double mechanismRotations) {
    double distance = ((mechanismRotations * SwerveModuleConstants.talonFXCPR) / SwerveModuleConstants.GEAR_RATIO) * (2 * 3.141592 * SwerveModuleConstants.WHEEL_RADIUS);
    return distance;
  }
}
