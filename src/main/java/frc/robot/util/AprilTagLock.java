package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLock {

   public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {
        PIDController pid = new PIDController(0.017, 1e-6, 0.000001); //kp, ki, kd
        pid.setTolerance(0); //degree tolerance
        pid.enableContinuousInput(0, 360); //range of degrees
        pid.setSetpoint(0); //initial setpoint
        return pid;
        
    }
   
    public static double getR() {
    //Calculates the control loop output of the "tx" limelight entry in degrees.
    return rotationPID.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    }
    
}