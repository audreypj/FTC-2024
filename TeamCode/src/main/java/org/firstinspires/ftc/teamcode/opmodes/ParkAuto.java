package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.autonomous.commands.AutoTest;

@Autonomous(name = "Park", preselectTeleOp = "Primaris")
public class ParkAuto extends TeleOp {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(Robot.OpModeType.TELEOP, hardwareMap, gamepad1, gamepad2);
        robot.setAutonomousCommand(RobotContainer.AutonomousSelection.PARK);
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
