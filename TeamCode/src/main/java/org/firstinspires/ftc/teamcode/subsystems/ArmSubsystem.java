package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {

    private HardwareMap hMap;

    private Motor armMotor;

    public ArmSubsystem(HardwareMap hMap) {

        this.hMap = hMap;

        armMotor = new MotorEx(hMap, "armMotor");

    }


    @Override
    public void periodic() {

    }
}
