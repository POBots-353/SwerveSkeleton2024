// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Simulation.FieldSim;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final XboxController driverControllerHID = driverController.getHID();
  
  private final SwerveDrive drive;
  private final FieldSim field;
  private final ExampleCommand exampleCommand = new ExampleCommand();
  
  public RobotContainer() {
    drive = SwerveDrive.create();

    field = new FieldSim(drive);

    drive.setDefaultCommand(
        new TeleopSwerve(driverController::getLeftY, driverController::getLeftX,
            driverController::getRightX, driverController::getRightY, driverControllerHID::getLeftBumper,
            SwerveDriveConstants.maxSpeed, SwerveDriveConstants.maxAngularSpeed, drive));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return exampleCommand;
  }

  public void teleopInit() {
    if (Robot.isSimulation()) {
      
    }
  }
  public void simulationInit() {
    field.initSim();
  }

  public void simulationPeriodic() {
    field.simulationPeriodic();
  }
}
