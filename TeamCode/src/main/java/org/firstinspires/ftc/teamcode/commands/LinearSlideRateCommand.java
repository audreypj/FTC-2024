package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class LinearSlideRateCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private DoubleSupplier rate;

    public LinearSlideRateCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier rate) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.rate = rate;

        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorTargetInches(elevatorSubsystem.getCurrentElevatorExtension() + (rate.getAsDouble() * 0.3));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}