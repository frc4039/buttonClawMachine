package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.BCMConstants.AnalogInputConstants;
import org.firstinspires.ftc.teamcode.BCMConstants.DigitalInputConstants;
import org.firstinspires.ftc.teamcode.BCMConstants.MotorConstants;

/******************************************************/
/*        Hub Controller Status LED:                  */
/*          - Blue: robot code booting up             */
/*          - Green: robot code ready to run          */
/******************************************************/
@TeleOp(name="Teleop Button Claw Game", group="Iterative OpMode")

/* What we'll need
5. Create all other required variables
6. Initialize component variables created in step #5
8. Figure out encoder logic for stopping Forward movement
9. On trigger button pressed, set the pickUpClawTriggered flag => might need multiple states
*/

public class TeleopButtonClawGame extends OpMode
{
    /********Joystick - Analog Inputs********/
    AnalogInput joystickForwardAnalog0;
    AnalogInput joystickBackAnalog1;
    AnalogInput joystickLeftAnalog2;
    AnalogInput joystickRightAnalog3;
    double voltageForward;
    double voltageBack;
    double voltageLeft;
    double voltageRight;
    /****************************************/

    /**Switches and Button - Digital Inputs**/
    TouchSensor stopSwitchBackDigital1;
    TouchSensor stopSwitchLeftDigital3;
    TouchSensor stopSwitchRightDigital5;

    /****************************************/

    /************Motors and Servo************/
    DcMotor motorLeftRightPort1;
    DcMotor motorBackForwardPort0;
    // servo motor servoPickUpClaw
    /****************************************/

    boolean pickUpClawTriggered = false;

    @Override
    public void init()
    {
        //the deviceName parameter is the name defined in the Driver's Station
        //Joystick - Analog Inputs
        joystickForwardAnalog0 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickForward);
        joystickBackAnalog1 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickBack);
        joystickLeftAnalog2 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickLeft);
        joystickRightAnalog3 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickRight);

        //Switches - Digital Inputs
        stopSwitchBackDigital1 = hardwareMap.get(TouchSensor.class, DigitalInputConstants.kStopSwitchBack);
        stopSwitchLeftDigital3 = hardwareMap.get(TouchSensor.class, DigitalInputConstants.kStopSwitchLeft);
        stopSwitchRightDigital5 = hardwareMap.get(TouchSensor.class, DigitalInputConstants.kStopSwitchRight);
        //create the other two

        //Motors
        motorLeftRightPort1 = CreateDCMotor(MotorConstants.kMotorLeftRight);
        motorBackForwardPort0 = CreateDCMotor(MotorConstants.kMotorBackForward);
        //get the servo using hardwareMap
    }

    @Override
    public void loop() {
        if (!pickUpClawTriggered)
        {
            //Add logic to check the trigger button
            //if pressed => pickUpClawTriggered = true

            voltageForward = joystickForwardAnalog0.getVoltage();
            voltageBack = joystickBackAnalog1.getVoltage();
            voltageLeft = joystickLeftAnalog2.getVoltage();
            voltageRight = joystickRightAnalog3.getVoltage();

            if (voltageForward <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                //check for encoder
                motorBackForwardPort0.setPower(MotorConstants.kMotorPowerForward);
            }
            else if (voltageBack <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                if(!stopSwitchBackDigital1.isPressed())
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerBack);
                }
                else
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
                }
            }
            else
            {
                motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
            }

            if (voltageLeft <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                if(!stopSwitchLeftDigital3.isPressed())
                {
                    motorLeftRightPort1.setPower(MotorConstants.kMotorPowerLeft);
                }
                else
                {
                    motorLeftRightPort1.setPower(MotorConstants.kMotorPowerStop);
                }
            }
            else if (voltageRight <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                if(!stopSwitchRightDigital5.isPressed())
                {
                    motorLeftRightPort1.setPower(MotorConstants.kMotorPowerRight);
                }
                else
                {
                    motorLeftRightPort1.setPower(MotorConstants.kMotorPowerStop);
                }
            }
            else
            {
                motorLeftRightPort1.setPower(MotorConstants.kMotorPowerStop);
            }
        }
        else
        {
            //Activate motorBackForwardPort0 with kMotorPowerBack
            //Activate motorLeftRightPort1 with kMotorPowerLeft
            //check left and back switches to turn off motors
        }

        DisplayTelemetry();
    }

    private DcMotor CreateDCMotor(String motorName)
    {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }

    private void DisplayTelemetry()
    {
        telemetry.addData("Analog 0 'Forward' voltage: ", voltageForward);
        telemetry.addData("Analog 1 'Back' voltage: ", voltageBack);
        telemetry.addData("Analog 2 'Left' voltage: ", voltageLeft);
        telemetry.addData("Analog 3 'Right' voltage: ", voltageRight);
        telemetry.update();
    }
}
