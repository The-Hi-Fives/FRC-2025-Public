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
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagLock;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeIn;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.OuttakeOut;
import frc.robot.subsystems.Intake.Wrist_Intake.Wrist_Intake;

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
     private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
     private final Wrist_Intake m_wristintake = new Wrist_Intake();
     public final ClimberSubsystem m_climber = new ClimberSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevatorsubsystem = new ElevatorSubsystem();
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
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left) Remove semicolon, blue and yellow parenthasis to get apriltag tracking back
                    .withRotationalRate(driver.rightBumper().getAsBoolean() ? (AprilTagLock.getR()*MaxSpeed) : (-driver.getRightX()*MaxSpeed)))); // Drive counterclockwise with negative X (left)

        // reset the field-centric heading on start button press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //zero gyro

        drivetrain.registerTelemetry(logger::telemeterize);


        //Elevator Positions\\
        driver.a().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0))); //stow
        driver.povUpLeft().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.39))); //lv 1
        operator.b().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.62))); //lv 2
        driver.y().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(0.82))); //lv 3
        //operator.rightBumper().onTrue(runOnce(() -> m_elevatorsubsystem.setHeight(1.02))); //lv 4

        //Wrist Positions\\
        driver.povUp().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(0)))); //test bindings, will copy elevator pos. bindings
        driver.povUpRight().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(35)))); //test bindings, will copy elevator pos. bindings
        driver.povRight().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(45)))); //test bindings, will copy elevator pos. bindings
        driver.povDownRight().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(60)))); //test bindings, will copy elevator pos. bindings
        driver.povDown().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(75)))); //test bindings, will copy elevator pos. bindings
        driver.povDownLeft().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(80)))); //test bindings, will copy elevator pos. bindings
        driver.povLeft().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(90)))); //test bindings, will copy elevator pos. bindings
        //operator.povUpLeft().onTrue(runOnce(() -> m_wristintake.setAngle(Rotation2d.fromDegrees(100)))); //test bindings, will copy elevator pos. bindings
        
        
        //Intake\\
        driver.x().toggleOnTrue(new IntakeIn(m_intakeSubsystem)); //intake
        driver.b().toggleOnTrue(new OuttakeOut(m_intakeSubsystem)); //outtake

        //Climber\\
        driver.rightTrigger().whileTrue(new ClimberUpCommand(m_climber)); //Climb Up
        driver.leftTrigger().whileTrue(new ClimberDownCommand(m_climber)); //Climb Down
       
        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
