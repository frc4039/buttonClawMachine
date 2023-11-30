package org.firstinspires.ftc.teamcode;

public final class BCMConstants {
    public static final class AnalogInputConstants {

        // The voltage at open switch -Joystick at rest - oscillates around 2v with 3K3 resistors
        // on engaging the switch, the voltage on the Analog Input drops down and it's
        // oscillating around 0.4v => we'll consider 0.9 our threshold, so any voltage below
        // that, will we considered "switch is closed"
        public static final double kVoltageJoystickEngagedThreshold = 0.9;

        //Analog Inputs names
        public static final String kJoystickUp = "JoystickUp"; //Analog Input port #0
        public static final String kJoystickDown = "JoystickDown"; //Analog Input port #1
        public static final String kJoystickLeft = "JoystickLeft"; //Analog Input Port #2
        public static final String kJoystickRight = "JoystickRight"; //Analog Input Port #3
    }

    public static final class MotorConstants {
        public static final String kMotorLeftRight = "MotorLeftRight"; // Left and right traversing Port #1
        public static final String kMotorBackForward = "MotorBackForward"; //Back and forward traversing Port #0
    }
}
