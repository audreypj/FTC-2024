package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.Optional;

public class ElevatorCommand extends CommandBase {

    public static class ScoreState {
        public double extension, angle;

        public ScoreState(Optional<Double> extension, Optional<Double> angle) {
            this.extension = extension.orElse(Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES);
            this.angle = angle.orElse(Constants.Wrist.Setpoints.STOWED);
            if(extension.isPresent()) { this.extension = extension.get(); }
            if(angle.isPresent()) { this.angle = angle.get(); }
        }

        public ScoreState(double extension, double angle) {
            this(Optional.of(extension), Optional.of(angle));
        }

        public ScoreState(double extension) {
            this(Optional.of(extension), Optional.empty());
        }
    }

    private final ElevatorSubsystem elevatorSubsystem;

    private double extension, angle;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double extension, double angle) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extension = extension;
        this.angle = angle;

        addRequirements(elevatorSubsystem);
    }

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ScoreState scoreState) {
        this(elevatorSubsystem, scoreState.extension, scoreState.angle);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorTargetInches(extension);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atTargetElevator();
    }

}
