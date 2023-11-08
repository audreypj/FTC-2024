package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.Optional;

public class ElevatorCommand extends CommandBase {

    public class ScoreState {
        public double extension, angle;

        public ScoreState(Optional<Double> extension, Optional<Double> angle) {
            if(extension.isPresent()) { this.extension = extension.get(); }
            if(angle.isPresent()) { this.angle = angle.get(); }
        }
    }

    private ElevatorSubsystem elevatorSubsystem;

    private double extension, angle;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double extension, double angle) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extension = extension;
        this.angle = angle;
    }

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ScoreState scoreState) {
        this(elevatorSubsystem, scoreState.extension, scoreState.angle);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorTargetInches(extension);
        elevatorSubsystem.setWristTargetAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atTargetAll();
    }

}
