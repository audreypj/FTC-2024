package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private HardwareMap hMap;

    private SimpleServo servo;

    public ShooterSubsystem(HardwareMap hMap) {
        this.hMap = hMap;

        servo = new SimpleServo(hMap, "shootMotor", 0 ,360);
    }

    @Override
    public void periodic() {
        
    }

}
