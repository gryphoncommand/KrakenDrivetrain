// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterTalonFX;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterIO m_shooter = Robot.isReal() ? new ShooterTalonFX() : new ShooterSim();

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

  // Robot state machine
  public enum State {
    NoPiece,
    StowedPiece,
    ReadyForShoot,
    Shooting,
    Intaking,
    Jammed
  }

  private State currentState = State.StowedPiece;

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
    m_driverController.rightBumper().whileTrue(new AlignToGoal(m_drive, m_driverController, DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7));
    m_driverController.rightTrigger().whileTrue(
      new RunCommand(()->{
        if (currentState == State.ReadyForShoot || currentState == State.Shooting){
          currentState = State.Shooting;
          // TODO: feed passthrough/intake to spun-up shooter
          currentState = State.StowedPiece;
        }
      })
    );
    
    // Operator bindings
    m_operatorController.start().onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));
    m_operatorController.a().onTrue(new InstantCommand(m_drive::stop, m_drive));
    m_operatorController.y().onTrue(new InstantCommand(m_drive::setX, m_drive));
    m_operatorController.rightTrigger().whileTrue(
        new RunCommand(() -> m_shooter.set(1.0), m_shooter)).onFalse(
          new InstantCommand(()->m_shooter.set(0), m_shooter));
    
    SmartDashboard.putData("Drive Forward A Lot", new RunCommand(()->m_drive.driveRobotRelativeChassis(new ChassisSpeeds(DriveConstants.kMaxSpeedMetersPerSecond, 0, 0)), m_drive).withTimeout(3));
    SmartDashboard.putData("Drive Back A Lot", new RunCommand(()->m_drive.driveRobotRelativeChassis(new ChassisSpeeds(-DriveConstants.kMaxSpeedMetersPerSecond, 0, 0)), m_drive).withTimeout(3));
    SmartDashboard.putData("Drive Forward + Turn A Lot", new RunCommand(()->m_drive.drive(1, 0, 1, true), m_drive).withTimeout(3));
  
  }

  private void configureStateTriggers() {
    Trigger aligned = new Trigger(m_drive::getAligned);
    Trigger inRange =
        new Trigger(() -> m_drive.getCurrentPose().getTranslation().getDistance(
            VisionConstants.kTagLayout
                .getTagPose(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7)
                .get().toPose2d().getTranslation())
            < AlignmentConstants.MAX_DIST);
            
    Trigger SpinUpRange =
            new Trigger(() -> m_drive.getCurrentPose().getTranslation().getDistance(
                VisionConstants.kTagLayout
                    .getTagPose(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7)
                    .get().toPose2d().getTranslation())
                < AlignmentConstants.SPIN_DIST);

    // TODO: Replace with real sensor for piece detection
    Trigger hasPiece = new Trigger(() -> true);

    hasPiece.onFalse(new InstantCommand(()->currentState = State.NoPiece)).onTrue(new InstantCommand(()->currentState = State.StowedPiece));

    Trigger SpunUp = new Trigger(() -> m_shooter.getVelocity() > 3000);

    (SpinUpRange.and(hasPiece)).whileTrue(new RunCommand(() -> m_shooter.setVelocity(3500), m_shooter)).whileFalse(new RunCommand(() -> m_shooter.set(0), m_shooter));


    Trigger readyToShoot = new Trigger(aligned.and(inRange).and(SpunUp).and(hasPiece));

    readyToShoot.onChange(new InstantCommand(()->SmartDashboard.putBoolean("Ready To Shoot", readyToShoot.getAsBoolean())));

    readyToShoot.whileTrue(new RepeatCommand(new InstantCommand(() -> {if (currentState == State.StowedPiece) {currentState = State.ReadyForShoot;}})));

    readyToShoot.onFalse(new InstantCommand(()->{
      if (hasPiece.getAsBoolean()){
        currentState = State.StowedPiece;
      } else {
        currentState = State.NoPiece;
      }
    }));
  }
    

  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    return 
    m_drive.PathToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4)), 0).andThen(
      m_drive.PathToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)), 0)
    );
  }

  /** Returns current robot state. */
  public State getCurrentState() {
    return currentState;
  }
}
