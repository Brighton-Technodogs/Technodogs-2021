// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/** Add your docs here. */
public class LazyVictorSPX extends VictorSPX {

    private double prevValue = 0;
    private final ControlMode prevControlMode = ControlMode.Disabled;

    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
        enableVoltageCompensation(true);
        configVoltageCompSaturation(12, 10);
    }

    @Override
    public void set(ControlMode mode, double outputValue) {
        //return;

        if (outputValue != prevValue || mode != prevControlMode) {
            super.set(mode, outputValue);
            prevValue = outputValue;
        }
    }

    public double getSetpoint() {
        return prevValue;
    }
}
