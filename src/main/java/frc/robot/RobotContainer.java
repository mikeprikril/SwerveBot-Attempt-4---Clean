// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.PIDConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(Constants.Operators.driver);

  /* Subsystems */
  final Swerve s_Swerve = new Swerve();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setName("Drive");
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> driver.getLeftY(),
        () -> driver.getLeftX(),
        () -> driver.getRightX(),
        () -> driver.leftBumper().getAsBoolean(),
        () -> driver.rightBumper().getAsBoolean(),
        () -> driver.y().getAsBoolean(),
        () -> driver.b().getAsBoolean(),
        () -> driver.a().getAsBoolean(),
        () -> driver.x().getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Actions that we want to do when the robot is disabled.
   */
  public void disabledActions() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */
    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));

  }

  public void configureAutoCommands() {
    // this.autoCommands.put("L2 Link Farside", new FarsideL2Link(s_Swerve,
    // poseEstimator, wrist, arm, leds));
    // this.autoCommands.put("Cable Side L3", new CableSideL3(s_Swerve,
    // poseEstimator, arm, wrist, leds));

    // this.autoCommands.put("Two cone auto", new TwoConeAuto(s_Swerve,
    // poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("Cone and Cube L3", new ConeAndCubeL3(s_Swerve,
    // poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L2 Cone Charge Station",
    // new L2ChargeStationCone(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L3 Cone Farside",
    // new FarsideConeL3(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L2 Cone Center Balance",
    // new CenterChargeStation(s_Swerve, poseEstimator, arm, wrist, leds));
  }


  public void sendAutoCommands() {
    SmartDashboard.putString("selectedAuto", "");
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  return new exampleAuto(s_Swerve);
  }
}
