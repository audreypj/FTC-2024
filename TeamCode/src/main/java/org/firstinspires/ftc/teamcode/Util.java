package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.MathUtils;

public class Util {

    public static double normalizeDegrees(double degrees) {
        return (degrees % 360 + 360) % 360;
    }

    public static boolean atTargetTolerance(double input, double target, double tolerance) {
        return (input >= (target - tolerance)) && (input <= (target + tolerance));
    }

    public static double modifyJoystick(double value, double deadband) {
        // Deadband
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                value = (value - deadband) / (1.0 - deadband);
            } else {
                value = (value + deadband) / (1.0 - deadband);
            }
        } else {
            value = 0;
        }

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

}
