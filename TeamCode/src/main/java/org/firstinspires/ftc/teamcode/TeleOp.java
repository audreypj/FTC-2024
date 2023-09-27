package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Primaris")
public class TeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(Robot.OpModeType.TELEOP);
    }

    @Override
    public void start() {
        robot.enable();
    }

    @Override
    public void stop() {
        robot.disable();
    }

    @Override
    public void loop() {
        robot.run();
    }

}
