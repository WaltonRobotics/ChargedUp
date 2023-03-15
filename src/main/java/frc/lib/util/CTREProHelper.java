package frc.lib.util;

import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class CTREProHelper {
    private CTREProHelper() {}

    public static void setNeutralMode(TalonFX motor, NeutralModeValue mode) {
        var cfgr = motor.getConfigurator();
        var motorOutCfg = new MotorOutputConfigs();
        cfgr.refresh(motorOutCfg);
        motorOutCfg.NeutralMode = mode;
        cfgr.apply(motorOutCfg);
    }
}
