package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.BCMConstants.AnalogInputConstants;

/******************************************************/
/*        Hub Controller Status LED:                  */
/*          - Blue: robot code booting up             */
/*          - Green: robot code ready to run          */
/******************************************************/
@TeleOp(name="Teleop Button Claw Game", group="Iterative OpMode")

/* What we'll need
1. On the Driver Station, configure and name Analog Input ports #2 & #3- please pay attention to naming convention
2. Define 2 more analog input variables- please pay attention to naming convention
3. Initialize the 2 new analog input variables using hardwareMap.get()
4. Identify all subsystem components- motors, switches, etc- make a list
5. Create variables for each component identified in step #4
6. Initialize component variables created in step #5
*/

public class TeleopButtonClawGame extends OpMode
{
    AnalogInput joystickUpAnalog0;
    AnalogInput joystickDownAnalog1;
    AnalogInput joystickLeftAnalog2;
    AnalogInput joystickRightAnalog3;
    double voltageUp;
    double voltageDown;
    double voltageLeft;
    double voltageRight;



    @Override
    public void init()
    {
        //the deviceName parameter is the name defined in the Driver's Station
        joystickUpAnalog0 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickUp);
        joystickDownAnalog1 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickDown);
        joystickLeftAnalog2 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickLeft);
        joystickRightAnalog3 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickRight);
    }

    @Override
    public void loop() {
        voltageUp = joystickUpAnalog0.getVoltage();
        voltageDown = joystickDownAnalog1.getVoltage();
        voltageLeft = joystickLeftAnalog2.getVoltage();
        voltageRight = joystickRightAnalog3.getVoltage();
        telemetry.addData("Analog 0 'Up' voltage: ", voltageUp);
        telemetry.addData("Analog 1 'Down' voltage: ", voltageDown);
        telemetry.addData("Analog 2 'Down' voltage: ", voltageLeft);
        telemetry.addData("Analog 3 'Down' voltage: ", voltageRight);
        telemetry.update();
    }
}
