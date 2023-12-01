package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorRateCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;
    private DoubleSupplier extensionRate, angleRate;

    public ElevatorRateCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier extensionRate, DoubleSupplier angleRate) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extensionRate = extensionRate;
        this.angleRate = angleRate;

        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorTargetInches(elevatorSubsystem.getCurrentElevatorExtension() + extensionRate.getAsDouble() * 4);
        elevatorSubsystem.setWristControl(angleRate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
