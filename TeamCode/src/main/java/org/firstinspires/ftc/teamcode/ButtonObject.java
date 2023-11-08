package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ButtonObject extends Trigger {

    private final Trigger trigger;

    public ButtonObject(GamepadEx gamepad, GamepadKeys.Button button) {
        trigger = new Trigger(() -> gamepad.getButton(button));
    }

    @Override
    public boolean get() {
        return trigger.get();
    }

    @Override
    public Trigger whenActive(Command command) {
        return trigger.whenActive(command);
    }

    @Override
    public Trigger whileActiveContinuous(Command command) {
        return trigger.whileActiveContinuous(command);
    }

    @Override
    public Trigger whileActiveOnce(Command command) {
        return trigger.whileActiveOnce(command);
    }

    @Override
    public Trigger whenInactive(Command command) {
        return trigger.whenInactive(command);
    }

    @Override
    public Trigger toggleWhenActive(Command command) {
        return trigger.toggleWhenActive(command);
    }

    @Override
    public Trigger cancelWhenActive(Command command) {
        return trigger.cancelWhenActive(command);
    }

    @Override
    public Trigger and(Trigger trigger) {
        return trigger.and(trigger);
    }

    @Override
    public Trigger or(Trigger trigger) {
        return trigger.or(trigger);
    }

    @Override
    public Trigger negate() {
        return trigger.negate();
    }
}
