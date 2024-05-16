// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SimModuleIO implements ModuleIO {
  /** Creates a new ModuleIOSim. */

  private DCMotorSim driveSim;
  private DCMotorSim turnSim;

  private double driveVolts;
  private double turnVolts;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kS, SwerveModuleConstants.kV, SwerveModuleConstants.kA);

  public SimModuleIO() {
    driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 6.12, 0.025);
    turnSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 150.0 / 7.0, 0.004);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(turnSim.getOutput(0));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveSim.getOutput(0), getAngle());
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveSim.getOutput(1), getAngle());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVolts = MathUtil.clamp(volts, 0, 12);
    driveSim.setInputVoltage(driveVolts);
  }

  public void runTurnVolts(double volts) {
    turnVolts = MathUtil.clamp(volts, 0, 12);
    turnSim.setInputVoltage(turnVolts);
  }

  @Override
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double desiredVelocity = desiredState.speedMetersPerSecond;
    if (isOpenLoop) {
      setDriveVoltage(desiredVelocity/SwerveDriveConstants.maxSpeed);
    } else {
      double driveFeedForward = feedforward.calculate(desiredVelocity);
      double pidOutput = (getModuleState().speedMetersPerSecond - desiredVelocity) * SwerveModuleConstants.kP;
      pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);
      setDriveVoltage(driveFeedForward + pidOutput);
    }
  }

  @Override
  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d desiredAngle = desiredState.angle;
    Rotation2d angleError = desiredAngle.minus(getAngle());

    double pidOutput = angleError.getRadians() * SwerveModuleConstants.kP;
    pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);

    runTurnVolts(pidOutput);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getModuleState().angle); 
    
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void stop() {
    setDriveVoltage(0);
    runTurnVolts(0);
  }
}
