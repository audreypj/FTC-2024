package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class DrivebaseSubsystem extends SubsystemBase {

    private HardwareMap hMap;
    private Telemetry tele;

    private Motor fL,fR,bL,bR;
    private MecanumDrive mecanum;

    public DrivebaseSubsystem(HardwareMap hMap, Telemetry tele) {
        this.hMap = hMap;
        this.tele = tele;

        fL = new MotorEx(hMap, "fL");
        fR = new MotorEx(hMap, "fR");
        bL = new MotorEx(hMap, "bL");
        bR = new MotorEx(hMap, "bR");

        mecanum = new MecanumDrive(true, fL, fR, bL, bR);
    }

    public void driveMotors(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        mecanum.driveRobotCentric(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }

    @Override
    public void periodic() {

    }

}
