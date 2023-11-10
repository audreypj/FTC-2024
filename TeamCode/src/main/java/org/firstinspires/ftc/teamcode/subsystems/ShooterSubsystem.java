package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

public class ShooterSubsystem extends SubsystemBase {

    public enum Modes {
        OFF, LAUNCH, REVERSE
    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    private final CRServo servo;

    private Modes currentMode;

    private double lastChangeTime;

    public ShooterSubsystem(HardwareMap hMap) {
        servo = new CRServo(hMap, "shootMotor");

        servo.setRunMode(Motor.RunMode.RawPower);

        currentMode = Modes.OFF;
    }

    public void setMode(Modes mode) {
        if(mode != Modes.OFF) {
            lastChangeTime = Robot.currentTimestamp();
        }
        currentMode = mode;
    }

    public Modes getMode() {
        return currentMode;
    }

    private void advanceMode() {
        switch(currentMode) {
            case OFF:
                break;
            case LAUNCH:
                if(lastChangeTime > Constants.Shooter.Timings.LAUNCH_DURATION) currentMode = Modes.REVERSE;
                break;
            case REVERSE:
                if(lastChangeTime > Constants.Shooter.Timings.REVERSE_DURATION) currentMode = Modes.OFF;
                break;
        }
    }

    private void applyMode() {
        switch(currentMode) {
            case OFF:
                servo.set(Constants.Shooter.Powers.OFF);
                break;
            case LAUNCH:
                servo.set(Constants.Shooter.Powers.LAUNCH);
                break;
            case REVERSE:
                servo.set(Constants.Shooter.Powers.REVERSE);
                break;
        }
    }

    @Override
    public void periodic() {

        advanceMode();
        applyMode();

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("shooterMode", currentMode.toString());

            dashboard.sendTelemetryPacket(packet);
        }
    }

}
