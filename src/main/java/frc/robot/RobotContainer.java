// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PositionPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward = m_driverController.getLeftY();
              double strafe = m_driverController.getLeftX();
              double turn = m_driverController.getRightX();

              m_drive.drive(
                -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_driverController.start().onTrue(new InstantCommand(()->m_drive.zeroHeading(), m_drive));
    m_driverController.leftBumper().onTrue(new InstantCommand(()->PositionPIDCommand.generateCommand(m_drive, PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2), Seconds.of(2)).schedule()));
    m_driverController.rightBumper().whileTrue(new InstantCommand(()->m_drive.PathToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, -2), 0.0).schedule()));
    // Operator bindings
    SmartDashboard.putData("Drive 2m Back", new InstantCommand(()->m_drive.PathToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, -2), 0.0).schedule()));
    SmartDashboard.putData("Drive 2m Forward", new InstantCommand(()->m_drive.PathToPose(PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2), 0.0).schedule()));
    SmartDashboard.putData("PID 2m Forward", new InstantCommand(()->PositionPIDCommand.generateCommand(m_drive, PositionCalculations.translateCoordinates(m_drive::getCurrentPose, 0, 2), Seconds.of(2)).schedule()));
  
  }

  private void configureStateTriggers() {}
    

  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    return 
    m_drive.PathToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4)), 0).andThen(
      m_drive.PathToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)), 0)
    );
  }
}
