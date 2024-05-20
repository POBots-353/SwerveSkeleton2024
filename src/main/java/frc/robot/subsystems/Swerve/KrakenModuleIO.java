// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class KrakenModuleIO implements ModuleIO {
  /** Creates a new SwerveModule. */
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder cancoder;

  private VelocityVoltage desiredVelocity;
  private PositionVoltage anglePosition;

  private DutyCycleOut driveDutyCycle;

  private SwerveModuleState previousState = new SwerveModuleState();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kS, SwerveModuleConstants.kV, SwerveModuleConstants.kA);

  public KrakenModuleIO(int driveMotorID, int turnMotorID, int canCoderID, String moduleName) {

    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    cancoder = new CANcoder(canCoderID);

    talonConfig();
  }

  public void talonConfig() {
    driveMotor.setInverted(false);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    turnMotor.setInverted(false);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    
    Slot0Configs slot0 = new Slot0Configs();

    slot0.kS = SwerveModuleConstants.kS; 
    slot0.kV = SwerveModuleConstants.kV;
    slot0.kA = SwerveModuleConstants.kA; 
    slot0.kP = SwerveModuleConstants.kP; 
    slot0.kI = SwerveModuleConstants.kI; 
    slot0.kD = SwerveModuleConstants.kD; 

    talonConfigs.Slot0 = slot0;

    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = SwerveModuleConstants.maxCurrent;
    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveModuleConstants.maxCurrent;

    driveMotor.getConfigurator().apply(talonConfigs);
    turnMotor.getConfigurator().apply(talonConfigs);
  }

  public double RPStoMPS(double rps) {
    return rps*(SwerveModuleConstants.WHEEL_CIRCUMFERENCE/(2*3.14));
  }

  public double MPStoRPS(double mps) {
    return mps/(SwerveModuleConstants.WHEEL_CIRCUMFERENCE/(2*3.14));
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue());
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(RPStoMPS(driveMotor.getVelocity().getValue()), Rotation2d.fromRadians(turnMotor.getPosition().getValue()));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(RPStoMPS(driveMotor.getVelocity().getValue()), Rotation2d.fromRadians(turnMotor.getPosition().getValue()));
  }

  @Override
  public void setAngle(SwerveModuleState desiredState) {
    anglePosition.FeedForward = feedforward.calculate(desiredState.angle.getRotations());
    turnMotor.setControl(anglePosition);
  }

  @Override
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double desiredSpeed = desiredState.speedMetersPerSecond;
    
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredSpeed / SwerveDriveConstants.maxSpeed;
      driveMotor.setControl(driveDutyCycle);
    } else {
    desiredVelocity.Velocity = MPStoRPS(desiredSpeed);
    desiredVelocity.FeedForward = feedforward.calculate(desiredSpeed, (desiredSpeed - previousState.speedMetersPerSecond) / 0.02);
    driveMotor.setControl(desiredVelocity); 
    }
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getModuleState().angle); 
    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState);
  }

  public void periodic() {
  }
}