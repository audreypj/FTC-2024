package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class Robot extends com.arcrobotics.ftclib.command.Robot {

    public enum OpModeType {
        TELEOP, AUTO
    }

    private RobotContainer robotContainer;

    private CommandBase autonomousCommand;

    public Robot(OpModeType opModeType) {

        robotContainer = new RobotContainer();

        //check if opmode is auto or tele and init accordingly
        if(opModeType == OpModeType.TELEOP) {
            initTele();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    private void initTele() {

    }

    private void initAuto() {
        
    }
}
