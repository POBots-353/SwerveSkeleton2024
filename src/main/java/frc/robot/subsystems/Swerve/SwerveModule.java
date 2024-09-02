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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModule extends SubsystemBase {
  
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder cancoder;

  String moduleName;

  private VelocityVoltage desiredVelocity;
  private PositionVoltage anglePosition;

  private Conversions Conversions;

  private DutyCycleOut driveDutyCycle;

  private SwerveModuleState previousState = new SwerveModuleState();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kS, 
                                                                          SwerveModuleConstants.kV, 
                                                                          SwerveModuleConstants.kA);

  private PIDController driveController = new PIDController(SwerveModuleConstants.kP, 0, 0);
  private PIDController turnController = new PIDController(SwerveModuleConstants.kP, 0, 0);
  
  private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(0.16, 0.0917),
          DCMotor.getKrakenX60(1),
          12.8
  );

  private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(2, 1.24),
          DCMotor.getKrakenX60Foc(1),
          8.16
  );

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;
  private EncoderSim simulationTurnEncoderSim;
  private EncoderSim simulationThrottleEncoderSim;

  private double simTurnEncoderDistance;
  private double simThrottleEncoderDistance;

  private double driveOutput;
  private double turnOutput;
  
  public SwerveModule(int driveMotorID, 
                      int turnMotorID, 
                      int canCoderID, 
                      String moduleName,
                      int simThrottleChannelA,
                      int simThrottleChannelB,
                      int simTurnChannelA,
                      int simTurnChannelB) {

    this.moduleName = moduleName;

    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    cancoder = new CANcoder(canCoderID);

    talonConfig();

    simulationThrottleEncoder = new Encoder(simThrottleChannelA, simThrottleChannelB);
    simulationTurnEncoder = new Encoder(simTurnChannelA, simTurnChannelB);

    simulationThrottleEncoderSim = new EncoderSim(simulationThrottleEncoder);
    simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
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

  public void setAngle(SwerveModuleState desiredState) {
    if (Robot.isReal()) {
      anglePosition.FeedForward = feedforward.calculate(desiredState.angle.getRotations());
      turnMotor.setControl(anglePosition);
    } else {
      Rotation2d desiredAngle = desiredState.angle;

      turnOutput = turnController.calculate(simulationTurnEncoder.getDistance(), desiredAngle.getRadians());
      double turnFeedForward = feedforward.calculate(turnController.getSetpoint());
      turnOutput = turnOutput + turnFeedForward;
    }
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (Robot.isReal()) {
      double desiredSpeed = desiredState.speedMetersPerSecond;
    
      if (isOpenLoop) {
        driveDutyCycle.Output = desiredSpeed / SwerveDriveConstants.maxSpeed;
        driveMotor.setControl(driveDutyCycle);
      } else {
      desiredVelocity.Velocity = Conversions.MPStoRPS(desiredSpeed);
      desiredVelocity.FeedForward = feedforward.calculate(desiredSpeed, (desiredSpeed - previousState.speedMetersPerSecond) / 0.02);
      driveMotor.setControl(desiredVelocity); 
      }
    } 
    
    else {
      double desiredVelocity = desiredState.speedMetersPerSecond;
      driveOutput = driveController.calculate(getVelocity(), desiredVelocity);
      double driveFeedForward = feedforward.calculate(desiredVelocity);

      driveOutput = driveOutput + driveFeedForward;

      moduleThrottleSimModel.setInputVoltage(driveOutput / SwerveDriveConstants.maxSpeed * RobotController.getBatteryVoltage());
    }

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getModuleState().angle); 
    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState);
  }

  public SwerveModulePosition getModulePosition() {
    if (Robot.isReal()) {
      return new SwerveModulePosition(Conversions.MechanismRotationsToMeters(driveMotor.getPosition().getValueAsDouble()), getAngle());
    } else {
      return new SwerveModulePosition(getVelocity(), getAngle());
    }
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public double getVelocity() {
    if (Robot.isReal()) {
      return Conversions.RPStoMPS(driveMotor.getVelocity().getValueAsDouble());
    } else {
      return simulationThrottleEncoder.getRate();
    }
  }

  public Rotation2d getAngle() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(Conversions.MechanismRotationsToDegrees(turnMotor.getPosition().getValueAsDouble()));
    } else {
      return Rotation2d.fromRadians(simulationTurnEncoder.getDistance());
    }
  }

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValueAsDouble());
  }

  public double simGetPosition() {
    return simulationThrottleEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String telemetryKey = "Swerve/" + moduleName + "/";

    SmartDashboard.putNumber(telemetryKey + "Position", getModulePosition().distanceMeters);

    SmartDashboard.putNumber(telemetryKey + "Velocity", getModuleState().speedMetersPerSecond);
    SmartDashboard.putNumber(telemetryKey + "Angle", getAngle().getDegrees());


    moduleThrottleSimModel.setInputVoltage(driveOutput / SwerveDriveConstants.maxSpeed * RobotController.getBatteryVoltage());
    moduleRotationSimModel.setInputVoltage(turnOutput / SwerveDriveConstants.maxAngularSpeed * RobotController.getBatteryVoltage());

    moduleRotationSimModel.update(0.02);
    moduleThrottleSimModel.update(0.02);

    simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationTurnEncoderSim.setDistance(simTurnEncoderDistance);
    simulationTurnEncoderSim.setRate(moduleRotationSimModel.getAngularVelocityRadPerSec());

    simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationThrottleEncoderSim.setDistance(simThrottleEncoderDistance);
    simulationThrottleEncoderSim.setRate(moduleThrottleSimModel.getAngularVelocityRadPerSec());
  }
}
