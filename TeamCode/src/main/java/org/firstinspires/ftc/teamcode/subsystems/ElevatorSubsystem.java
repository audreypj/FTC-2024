package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem extends SubsystemBase {

    private Motor elevatorMotor;

    public ElevatorSubsystem(HardwareMap hMap, Telemetry tele) {

        elevatorMotor = new MotorEx(hMap, "eleMotor");

    }


    @Override
    public void periodic() {

    }
}
