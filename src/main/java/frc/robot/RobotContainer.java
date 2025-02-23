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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeIn;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.OuttakeOut;
import frc.robot.subsystems.Intake.Wrist_Intake.Wrist_Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController master = new CommandXboxController(3);

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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("outtake", (new OuttakeOut(m_intakeSubsystem)));
    NamedCommands.registerCommand("intake", (new IntakeIn(m_intakeSubsystem)));
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
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

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
    driver.x().whileTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(10)))); // Stow
    driver.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // stow

    // Stow\\
    driver.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // stow
    driver
        .y()
        .whileTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(55)))); // Grnd Algae

    // Operator\\

    // Setpoints\\
    // Stow & Procesor
    operator.povDown().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(10))));
    operator.povDown().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0)));

    // Coral station
    operator
        .rightTrigger()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(27))));
    operator.rightTrigger().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.28)));

    // Level 1
    operator.a().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(38))));
    operator.a().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0)));

    // Level 2
    operator.x().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(47))));
    operator.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.42)));

    // Level 3
    operator.b().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(47))));
    operator.b().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.82)));

    // Level 4
    operator.y().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(30))));
    operator.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.35)));

    // Climber\\
    operator.povLeft().whileTrue(new ClimberUpCommand(m_climber)); // Climb Up

    operator.povRight().whileTrue(new ClimberDownCommand(m_climber)); // Climb Down

    // MASTER CONTROLS\\

    // Elevator Positions\\
    master.a().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); // stow
    master.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.26))); // Coral Station
    master.b().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.57))); // lv 2
    master.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.82))); // lv 3
    master.povLeft().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.35))); // lv 4

    // Wrist Positions\\
    master.povUp().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(0)))); // stow
    master
        .povRight()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(30)))); // Coral Station
    master
        .povDown()
        .onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(55)))); // Grnd Algae

    // Intake\\
    master.leftTrigger().toggleOnTrue(new IntakeIn(m_intakeSubsystem)); // intake
    master.rightTrigger().toggleOnTrue(new OuttakeOut(m_intakeSubsystem)); // outtake

    // Climber\\
    master.rightTrigger().whileTrue(new ClimberUpCommand(m_climber)); // Climb Up
    master.leftTrigger().whileTrue(new ClimberDownCommand(m_climber)); // Climb Down
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
