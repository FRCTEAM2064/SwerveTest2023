// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick driverTurnJoystick = new Joystick(OIConstants.kDriverTurnControllerPort);
  private final Joystick pxnController = new Joystick(OIConstants.kPXNControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> driverJoystick.getRawAxis(0), // X axis
        () -> driverJoystick.getRawAxis(1), // y axis
        () -> -driverTurnJoystick.getRawAxis(0), // turning speed
        () -> !driverJoystick.getRawButton(1), // field oriented button
        () -> driverTurnJoystick.getRawButton(1)));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverTurnJoystick, 2)
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String pathName) {
    // An example command will be run in autonomous
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    SwerveAutoBuilder builder = new SwerveAutoBuilder(
        swerveSubsystem::getPose,
        swerveSubsystem::resetOdometry,
        DriveConstants.kDriveKinematics,
        new PIDConstants(AutoConstants.kPXController, 0, 0),
        new PIDConstants(AutoConstants.kPThetaController, 0, 0),
        swerveSubsystem::setModuleStates,
        eventMap,
        true,
        swerveSubsystem);
    return builder.fullAuto(path);
  }
}
