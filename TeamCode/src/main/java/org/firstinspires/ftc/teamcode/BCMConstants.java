package org.firstinspires.ftc.teamcode;

public final class BCMConstants {
    public static final class AnalogInputConstants {

        // The voltage at open switch -Joystick at rest - oscillates around 2v with 3K3 resistors
        // on engaging the switch, the voltage on the Analog Input drops down and it's
        // oscillating around 0.4v => we'll consider 0.9 our threshold, so any voltage below
        // that, will we considered "switch is closed"
        public static final double kVoltageJoystickEngagedThreshold = 0.9;

        //Analog Inputs names
        public static final String kJoystickForward = "JoystickForward"; //Analog Input port #0
        public static final String kJoystickBack = "JoystickBack"; //Analog Input port #1
        public static final String kJoystickLeft = "JoystickLeft"; //Analog Input Port #2
        public static final String kJoystickRight = "JoystickRight"; //Analog Input Port #3
    }

    public  static final class  DigitalInputConstants {
        //Digital Inputs names
        public static final String kStopSwitchBack = "StopSwitchBack"; //Limiting switch on Digital port #1
        public static final String kStopSwitchLeft = "StopSwitchLeft"; //Limiting switch on Digital port #3
        public static final String kStopSwitchRight = "StopSwitchRight"; //Limiting switch on Digital port #5
        public static final String kButtonPickUpClaw = "ButtonPickUpClaw"; //Claw Trigger button on Digital port #
    }

    public static final class MotorConstants {
        //Motors power
        public static final double kMotorPowerForward = 0.70;
        public static final double kMotorPowerBack = - kMotorPowerForward;
        public static final double kMotorPowerLeft = -0.40;
        public static final double kMotorPowerRight = -kMotorPowerLeft;
        public static final double kMotorPowerStop = 0;
        public static final double kServoPowerMoveDown = 1;
        public static final double kServoPowerMoveUp = -kServoPowerMoveDown;

        //Motors Names
        public static final String kMotorLeftRight = "MotorLeftRight"; //Left and right traversing Port #1
        public static final String kMotorBackForward = "MotorBackForward"; //Back and forward traversing Port #0
        public static final String kServoPickUpClaw = "ServoPickUpClaw"; //Claw Servo

        //Positions
        public static final double kMotorBackForwardLimitFowardPosition = 4830;

        //Claw travel times. All these values are milliseconds
        public static final int kServoTimeToPickupPositionDown = 3710; //The magnet must be flash with the release tube
        public static final int kServoTimeToTravelPositionUp = 1500; //Time to travel position from pickup position
        public static final int kServoTimeToReleasePositionUp = 1600; //Time to release position from travel position
        public static final int kServoTimeToResetPositionDown = 100; //Time to reset position to flash with release tube from release position

        //Time specific
        public static final int kServoMoveDelaySeconds = 1; //Time to stop between servo movements
    }
}
