package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    public enum Modes {
        INTAKE, OUTTAKE, OFF
    }

    private CRServo intakeServo;

    private Modes currentMode;

    public IntakeSubsystem(HardwareMap hMap) {

        intakeServo = new CRServo(hMap, "intakeServo");

        currentMode = Modes.OFF;

    }

    public void setMode(Modes mode) {
        currentMode = mode;
    }

    private void applyMode() {
        //FIXME placeholder values
        switch(currentMode) {
            case INTAKE:
                intakeServo.set(0);
                break;
            case OUTTAKE:
                intakeServo.set(0);
                break;
            case OFF:
            default:
                intakeServo.set(0);
        }
    }

    @Override
    public void periodic() {

        applyMode();

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("currentIntakeMode", currentRunMode);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
