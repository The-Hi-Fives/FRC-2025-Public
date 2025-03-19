package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlign extends Command{
    Drive drive;
    
    public static PIDController rotationPID = createPIDController();

    private static PIDController createPIDController() {
        PIDController pid = new PIDController(0.017, 1e-6, 0.000001); // kp, ki, kd
        pid.setTolerance(0); // degree tolerance
        pid.enableContinuousInput(0, 360); // range of degrees
        pid.setSetpoint(0); // initial setpoint
        return pid;
    }
    
    public AutoAlign(Drive drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        DriveCommands.joystickDrive(drive, 
        () -> 0,
        () -> 0,
        () -> rotationPID.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)));
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}