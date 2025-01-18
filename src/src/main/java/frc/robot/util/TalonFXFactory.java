package frc.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.ErrorCheckUtil.CommonErrorNames;

/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static double kTimeoutSeconds = 0.05;


    private static final TalonFXConfiguration kDefaultConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast));

    /**
     * Create a CANTalon with a default configuration
     * 
     * @param id Device's CAN ID
     * @param canBus Device's CAN Bus
     * @return The newly created TalonFX
     */
    public static TalonFX createDefaultTalon(int id, String canBus) {
        return createTalon(id, canBus, kDefaultConfiguration);
    }

    /**
     * Creates a new TalonFX and applies the given configuration
     * 
     * @param id     Device's CAN ID
     * @param canBus Device's CAN Bus
     * @param config Config to apply
     * @return The newly created TalonFX
     */
    public static TalonFX createTalon(int id, String canBus, TalonFXConfiguration config) {
        TalonFX talon = new TalonFX(id, canBus);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        talon.clearStickyFaults(kTimeoutSeconds);
        talon.setPosition(0);

        motorConfig = config;

        ErrorCheckUtil.checkError(talon.getConfigurator().apply(motorConfig), CommonErrorNames.ConfiguringTalon(id));
        return talon;
    }
}