// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagLock;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeIn;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.OuttakeOut;
import frc.robot.subsystems.Intake.Intake_PID.Wrist_Intake;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity.

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    //private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevatorsubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();
    public final Wrist_Intake m_wristintake = new Wrist_Intake();
    //public final Intakecoral m_intakecoral = new Intakecoral();
    //public final Robot robot = new Robot();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(driver.rightBumper().getAsBoolean() ? (AprilTagLock.getR()*MaxSpeed) : (-driver.getRightX()*MaxSpeed)))); // Drive counterclockwise with negative X (left)

        // reset the field-centric heading on start button press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //zero gyro

        drivetrain.registerTelemetry(logger::telemeterize);

        //Elevator\\
        operator.a().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); //stow
        operator.x().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.45))); //lv 1
        operator.b().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.65))); //lv 2
        operator.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.85))); //lv 3
        operator.povUp().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.02))); //lv 4

        //Wrist Intake\\
        operator.povLeft().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(0)))); //Stow
        operator.povRight().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(20)))); //Coral Station intake
        operator.povDown().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(90)))); //Algae intake


        //Intake\\
        driver.rightBumper().whileTrue(new IntakeIn(m_intakesubsystem)); //Intake
        driver.leftBumper().whileTrue(new OuttakeOut(m_intakesubsystem)); //Outtake

        
        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
