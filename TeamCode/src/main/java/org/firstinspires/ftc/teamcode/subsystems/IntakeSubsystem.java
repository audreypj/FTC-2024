package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public enum Modes {
        DRIVE, TIMED
    }

    public enum RunModes {
        OFF, INTAKE, OUTTAKE
    }

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet;

    private MotorEx intakeMotor;

    private Modes currentMode;
    private RunModes currentRunMode;

    public IntakeSubsystem(HardwareMap hMap) {

        intakeMotor = new MotorEx(hMap, "intakeMotor");

        currentMode = Modes.DRIVE;
        currentRunMode = RunModes.OFF;

    }

    public void setMode(Modes mode) {
        currentMode = mode;
    }

    public void setRunMode(RunModes mode) {
        currentRunMode = mode;
    }

    public RunModes getRunMode() {
        return currentRunMode;
    }

    private void runModePeriodic() {
        switch(currentRunMode) {
            case INTAKE:
                intakeMotor.set(Constants.Intake.ModePowers.INTAKE);
                break;
            case OUTTAKE:
                intakeMotor.set(Constants.Intake.ModePowers.OUTTAKE);
                break;
            case OFF:
            default:
                intakeMotor.set(Constants.Intake.ModePowers.OFF);
        }
    }

    private void runTimedModePeriodic() {

    }

    @Override
    public void periodic() {

        if(currentMode == Modes.DRIVE) {
            runModePeriodic();
        } else {
            runTimedModePeriodic();
        }

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("currentIntakeMode", currentRunMode);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
