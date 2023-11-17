package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class LinearSlideCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private double targetHeight;

    public LinearSlideCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight) {

        this.elevatorSubsystem = elevatorSubsystem;

        this.targetHeight = targetHeight;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorTargetInches(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Util.atTargetTolerance(elevatorSubsystem.getCurrentElevatorExtension(), targetHeight, 0.1);
    }
}
