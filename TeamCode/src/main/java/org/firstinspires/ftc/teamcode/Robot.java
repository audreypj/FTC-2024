package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.Command;
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

    private Command autonomousCommand;

    public Robot(OpModeType opModeType, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {

        robotContainer = new RobotContainer(hardwareMap, gamepad1, gamepad2);

        timer = new Timing.Timer(600); //placeholder length

        //check if opmode is auto or tele and init accordingly
        if(opModeType == OpModeType.TELEOP) {
            initTele();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    private void initTele() {
        if(autonomousCommand != null) {
            autonomousCommand.cancel();
        } else {
            timer.start();
        }
    }

    private void initAuto() {
        timer.start();
        if(autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    public static double currentTimestamp() {
        return timer.elapsedTime();
    }

    public void setAutonomousCommand(RobotContainer.AutonomousSelection autonomousSelection) {
        this.autonomousCommand = robotContainer.getAutonomousCommand(autonomousSelection);
    }

}
