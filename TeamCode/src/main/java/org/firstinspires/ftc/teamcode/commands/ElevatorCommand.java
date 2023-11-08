package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.Optional;

public class ScoreCommand extends SequentialCommandGroup {

    public class ScoreState {

        public double extension, angle;

        public ScoreState(Optional<Double> extension, Optional<Double> angle) {
            extension.ifPresent();

        }
    }

    public ScoreCommand(ElevatorSubsystem elevatorSubsystem) {

        addCommands();
    }

}
