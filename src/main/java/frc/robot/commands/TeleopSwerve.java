// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private DoubleSupplier forwardSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier angleX;
  private DoubleSupplier angleY;
  private double maxSpeed;
  private BooleanSupplier turnToAngle;
  private double maxAngularSpeed;

  SwerveDrive swerve;

  private PIDController turnToAngleController = new PIDController(SwerveModuleConstants.kP, 0, SwerveModuleConstants.kD);

  private SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(SwerveDriveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(SwerveDriveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter angleRateLimiter = new SlewRateLimiter(SwerveDriveConstants.maxAngularAcceleration); 

  private final boolean isOpenLoop = false;

  public TeleopSwerve(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier angleX,
      DoubleSupplier angleY,
      BooleanSupplier turntoAngle,
      double maxSpeed,
      double maxAngularSpeed,
      SwerveDrive swerve) {
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.angleX = angleX;
    this.angleY = angleY;
    this.maxSpeed = maxSpeed;
    this.maxAngularSpeed = maxAngularSpeed;

    this.swerve = swerve;

    turnToAngleController.enableContinuousInput(-Math.PI, Math.PI);
    turnToAngleController.setTolerance(Units.degreesToRadians(1.0));

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = -forwardSupplier.getAsDouble() * maxSpeed;
    double strafeSpeed = strafeSupplier.getAsDouble() * maxSpeed;

    double angleXDrive = -angleX.getAsDouble();
    double angleYDrive = -angleY.getAsDouble();

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
    angleXDrive = angleRateLimiter.calculate(angleXDrive);

    forwardSpeed = MathUtil.applyDeadband(forwardSpeed, 0.5);
    strafeSpeed = MathUtil.applyDeadband(strafeSpeed, 0);

    if (!turnToAngle.getAsBoolean()) {
      angleXDrive = angleRateLimiter.calculate(angleXDrive);

      angleXDrive = MathUtil.applyDeadband(angleXDrive, Units.degreesToRadians(1.0));

      swerve.drive(forwardSpeed, strafeSpeed, angleXDrive * maxAngularSpeed,
          isOpenLoop);
    } else {
      angleRateLimiter.reset(0.0);

      if (Math.abs(angleXDrive) > 0.05 || Math.abs(angleYDrive) > 0.05) {
        Rotation2d desiredAngle = new Rotation2d(-angleXDrive, -angleYDrive);

        double angularSpeed = turnToAngleController.calculate(swerve.getHeading().getRadians(),
            -desiredAngle.getRadians() - (Math.PI / 2));

        angularSpeed = MathUtil.clamp(angularSpeed, -0.75, 0.75);

        swerve.drive(forwardSpeed, strafeSpeed,
            angularSpeed * SwerveDriveConstants.turnToAngleMaxVelocity, false);
      } else {
        swerve.drive(forwardSpeed, strafeSpeed, 0, isOpenLoop);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
