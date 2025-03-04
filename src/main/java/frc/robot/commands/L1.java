package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.Wrist_Intake.Wrist_Intake;

public class L1 extends Command {
  private ElevatorSubsystem elevator;
  private Wrist_Intake wrist;
  private Boolean auto;
  private int x;

  /** Creates a new Shoot. */
  public L1(ElevatorSubsystem elevator, Wrist_Intake wrist, Boolean auto) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.auto = auto;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int x = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setHeight(0);
    wrist.setAngle(Rotation2d.fromDegrees(35));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    wrist.stow();
    x = 0;
  }

  // Returns true when the command should end.
  //   @Override
  //   public boolean isFinished() {

  // }
}
