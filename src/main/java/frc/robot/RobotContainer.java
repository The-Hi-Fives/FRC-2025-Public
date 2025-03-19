// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeIn;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.L1Intake;
import frc.robot.subsystems.Intake.L1Outtake;
import frc.robot.subsystems.Intake.OuttakeOut;
import frc.robot.subsystems.Intake.Wrist_Intake.Wrist_Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AprilTagLock;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  public static CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Wrist_Intake m_wristintake = new Wrist_Intake();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final ElevatorSubsystem m_elevatorsubsystem = new ElevatorSubsystem();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines

    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("Intake", new IntakeIn(m_intakeSubsystem));
    NamedCommands.registerCommand("Outtake", new OuttakeOut(m_intakeSubsystem));
    NamedCommands.registerCommand("IntakeOff", Commands.runOnce(() -> m_intakeSubsystem.stop()));
    NamedCommands.registerCommand("TM-AI/CO", new L1Outtake(m_intakeSubsystem));
    NamedCommands.registerCommand("TM-AO/CI", new L1Intake(m_intakeSubsystem));
    NamedCommands.registerCommand(
        "WristL2", Commands.runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(20))));
    NamedCommands.registerCommand(
        "L2Ele", Commands.runOnce(() -> m_elevatorsubsystem.setHeight(0.08)));
    NamedCommands.registerCommand(
        "WristL4", Commands.runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(35))));
    NamedCommands.registerCommand(
        "L4Ele", Commands.runOnce(() -> m_elevatorsubsystem.setHeight(1.32)));
    NamedCommands.registerCommand(
        "StowWrist", Commands.runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(5))));
    NamedCommands.registerCommand(
        "StowEle", Commands.runOnce(() -> m_elevatorsubsystem.setHeight(0)));
    NamedCommands.registerCommand(
        "WristL3", Commands.runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(20))));
    NamedCommands.registerCommand(
        "L3Ele", Commands.runOnce(() -> m_elevatorsubsystem.setHeight(0.56)));
    NamedCommands.registerCommand(
        "CoralStationWrist",
        Commands.runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(5))));
    NamedCommands.registerCommand(
        "CoralStationEle", Commands.runOnce(() -> m_elevatorsubsystem.setHeight(0.23)));
    NamedCommands.registerCommand("Align", new AutoAlign(drive).withTimeout(1));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () ->
                ((driver.y().getAsBoolean() || driver.rightBumper().getAsBoolean()))
                    ? AprilTagLock.getR()
                    : -driver.getRightX()));

    //     drive.setDefaultCommand(
    // DriveCommands.joystickDrive(
    //     drive, () -> -master.getLeftY(), () -> -master.getLeftX(), () -> -master.getRightX()));

    // // Lock to 0° when A button is held
    // driver.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driver.getLeftY(), ()
    // -> -driver.getLeftX(), () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed

    // Driver\\

    // Intake\\
    driver.a().whileTrue(new OuttakeOut(m_intakeSubsystem)); // Coral outtake, Algae intake

    // Outtake\\
    driver.b().whileTrue(new IntakeIn(m_intakeSubsystem)); // Coral intake, Algae Outtake
    driver.rightTrigger().whileTrue(new L1Outtake(m_intakeSubsystem)); // AI-CO
    driver.leftTrigger().whileTrue(new L1Intake(m_intakeSubsystem)); // AO-CI
    // Trim   d-pad???\\
    // Trim intake up
    // Trim intake down
    // Elevator up
    // Elevator Down

    // Zero Drive
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Stow\\
    // driver.x().whileTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(10)))); //
    // 10, Stow
    // driver.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // 0, stow

    // Grnd Algae\\
    driver.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // 0, stow
    driver
        .x()
        .whileTrue(
            runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(55)))); // 55 Grnd Algae

    // Operator\\

    // Setpoints\\
    // Stow & Procesor
    driver.y().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(10)))); // 10
    driver.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // 0

    // Coral station
    operator
        .rightTrigger()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(5)))); // 5
    operator.rightTrigger().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.23))); // 0.23

    // Level 1
    operator.a().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(35)))); // 35
    operator.a().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // 0

    // Level 2
    operator.x().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(20)))); // 20
    operator.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.08))); // 0.08

    // Level 2-3 Algae
    operator
        .povUp()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(34)))); // 34
    operator.povUp().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.55))); // 0.55

    // Level 3
    operator.b().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(34)))); // 34
    operator.b().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.64))); // 0.64

    // Level 3-4 Algaex
    operator
        .leftTrigger()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(22)))); // 22
    operator.leftTrigger().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.85))); // 0.85

    // Level 4
    operator.y().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(32)))); // 32
    operator.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.34))); // 1.34
    // Climber\\
    // operator.povLeft().whileTrue(new ClimberUpCommand(m_climber)); // Climb Down
    // operator.povRight().whileTrue(new ClimberDownCommand(m_climber)); // Climb Up
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
