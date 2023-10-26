package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DrivebaseCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class RobotContainer {

    private final GamepadEx brandon = new GamepadEx(gamepad1);

    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(hardwareMap);

    public RobotContainer() {

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(
                drivebaseSubsystem,
                new DrivebaseCommand(
                        drivebaseSubsystem,
                        brandon::getLeftY,
                        brandon::getLeftX,
                        brandon::getRightX));
    }

    private void configureButtonBindings() {
        brandon.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MAX_HEIGHT_INCHES));

        brandon.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MID_HEIGHT_INCHES));

        brandon.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MIN_HEIGHT_INCHES));
    }

}
