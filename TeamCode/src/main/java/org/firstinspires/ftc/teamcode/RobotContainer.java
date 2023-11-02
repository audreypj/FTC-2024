package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DrivebaseCommand;
import org.firstinspires.ftc.teamcode.commands.LinearSlideCommand;
import org.firstinspires.ftc.teamcode.commands.ForceIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.WristRateCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final GamepadEx brandon = new GamepadEx(gamepad1);

    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

    public RobotContainer() {

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(elevatorSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

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
                .whenPressed(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MAX_EXTENSION_INCHES));

        brandon.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MID_HEIGHT_INCHES));

        brandon.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES));

        brandon.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new WristRateCommand(elevatorSubsystem, -0.3));

        brandon.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new WristRateCommand(elevatorSubsystem, 0.3));

        new Trigger(() -> brandon.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.INTAKE));

        new Trigger(() -> brandon.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.OUTTAKE));

    }

}
