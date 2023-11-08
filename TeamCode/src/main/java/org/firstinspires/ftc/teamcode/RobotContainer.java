package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
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
                new DefaultDriveCommand(
                        drivebaseSubsystem,
                        brandon::getLeftY,
                        brandon::getLeftX,
                        brandon::getRightX));
    }

    private void configureButtonBindings() {

        new ButtonObject(brandon, GamepadKeys.Button.Y)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.HIGH_SETPOINT));

        new ButtonObject(brandon, GamepadKeys.Button.B)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MID_HEIGHT_INCHES));

        new ButtonObject(brandon, GamepadKeys.Button.A)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES));

        new ButtonObject(brandon, GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new WristRateCommand(elevatorSubsystem, 0));

        new ButtonObject(brandon, GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new WristRateCommand(elevatorSubsystem, 30));

        new Trigger(() -> brandon.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.INTAKE));

        new Trigger(() -> brandon.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.OUTTAKE));

    }

}
