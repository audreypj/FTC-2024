package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Primaris")
public class TeleOp extends OpMode {



    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(Robot.OpModeType.TELEOP, hardwareMap, Optional.of(gamepad1), Optional.of(gamepad2));
    }

    @Override
    public void start() {
        com.arcrobotics.ftclib.command.Robot.enable();
    }

    @Override
    public void stop() {
        com.arcrobotics.ftclib.command.Robot.disable();
    }

    @Override
    public void loop() {
        robot.run();
    }

}
