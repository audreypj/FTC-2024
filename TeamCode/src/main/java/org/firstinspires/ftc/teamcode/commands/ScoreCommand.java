package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ScoreCommand extends SequentialCommandGroup {

    public class ScoreState {



        public ScoreState() {

        }
    }

    public ScoreCommand(ElevatorSubsystem elevatorSubsystem) {

        addCommands();
    }

}
