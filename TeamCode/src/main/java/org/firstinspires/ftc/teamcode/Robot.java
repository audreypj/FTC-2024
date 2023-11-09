package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

public class Robot extends com.arcrobotics.ftclib.command.Robot {

    public enum OpModeType {
        TELEOP, AUTO
    }

    private RobotContainer robotContainer;
    private static Timing.Timer timer;

    private CommandBase autonomousCommand;

    public Robot(OpModeType opModeType, HardwareMap hardwareMap, Optional<Gamepad> gamepad1, Optional<Gamepad> gamepad2) {

        if(gamepad1.isPresent()) {
            robotContainer = new RobotContainer(hardwareMap, gamepad1, gamepad2);
        } else {
            robotContainer = new RobotContainer(hardwareMap);
        }


        timer = new Timing.Timer(600);

        //check if opmode is auto or tele and init accordingly
        if(opModeType == OpModeType.TELEOP) {
            initTele();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public Robot(OpModeType opModeType, HardwareMap hardwareMap) {
        this(opModeType, hardwareMap, Optional.empty(), Optional.empty());
    }

    private void initTele() {

        if(autonomousCommand != null) {
            super.reset();
        } else {
            timer.start();
        }

    }

    private void initAuto() {
        timer.start();
    }

    public static double currentTimestamp() {
        return timer.elapsedTime();
    }

}
