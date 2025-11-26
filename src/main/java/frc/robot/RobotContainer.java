// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
              double forward = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double strafe = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
              double turn = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kTurnDeadband);

              m_drive.drive(forward, strafe, turn, true);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_driverController.start().onTrue(new InstantCommand(()->m_drive.zeroHeading(), m_drive));
    // Operator bindings
  
  }

  private void configureStateTriggers() {}
    

  /** Returns the autonomous command. */
  public SequentialCommandGroup getAutonomousCommand() {
    // TODO: Fill in with auto
    return new SequentialCommandGroup();
  }
}
