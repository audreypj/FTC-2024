package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;

public class Robot extends com.arcrobotics.ftclib.command.Robot {

    public enum OpModeType {
        TELEOP, AUTO
    }

    public Robot(OpModeType opModeType) {
        //check if opmode is auto or tele and init accordingly
        if(opModeType == OpModeType.TELEOP) {
            initTele();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public void initTele() {
        
    }

    public void initAuto() {
        
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void reset() {
        CommandScheduler.getInstance().reset();
    }
}
