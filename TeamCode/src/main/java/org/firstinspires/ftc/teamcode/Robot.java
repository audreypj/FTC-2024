package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;

public class Robot extends com.arcrobotics.ftclib.command.Robot {

    public enum OpModeType {
        TELEOP, AUTO
    }

    private RobotContainer robotContainer;
    private static Timing.Timer timer;

    private CommandBase autonomousCommand;

    public Robot(OpModeType opModeType) {

        robotContainer = new RobotContainer();
        timer = new Timing.Timer(600);

        //check if opmode is auto or tele and init accordingly
        if(opModeType == OpModeType.TELEOP) {
            initTele();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    private void initTele() {

        if(autonomousCommand != null) {

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
