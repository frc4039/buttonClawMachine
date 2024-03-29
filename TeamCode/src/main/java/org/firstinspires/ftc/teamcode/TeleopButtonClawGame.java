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
/*          - Purple: driver station config mode      */
/******************************************************/
@TeleOp(name="Teleop Button Claw Game - v.1.0.0", group="Iterative OpMode")
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
    TouchSensor pickUpClawButton;
    /****************************************/

    /************Motors and Servo************/
    DcMotor motorLeftRightPort1;
    DcMotor motorBackForwardPort0;
    DcMotor motorPickUpClawPort2;
    /****************************************/

    boolean pickUpClawTriggered = false;
    enum pickUpStages
    {
        idle,
        pickUp,
        travel,
        release,
        reset
    }
    pickUpStages currentStage = pickUpStages.idle;

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
        pickUpClawButton = hardwareMap.get(TouchSensor.class, DigitalInputConstants.kButtonPickUpClaw);

        //Motors
        motorLeftRightPort1 = CreateDCMotor(MotorConstants.kMotorLeftRight);
        motorBackForwardPort0 = CreateDCMotor(MotorConstants.kMotorBackForward);
        motorPickUpClawPort2 = CreateDCMotor(MotorConstants.kMotorPickUpClaw);

        //Initialize the Button Claw Machine
        MoveToHomePosition();

        //ResetMotorEncoder(motorPickUpClawPort2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(!pickUpClawTriggered && pickUpClawButton.isPressed())
        {
            pickUpClawTriggered = true;
            currentStage = pickUpStages.pickUp;
            DisplayTelemetry();
        }

        if(pickUpClawTriggered)
        {
            switch (currentStage)
            {
                case pickUp:
                    if(motorPickUpClawPort2.getCurrentPosition() <= MotorConstants.kMotorPickUpClawPickUpPosition)
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerDown);
                    }
                    else
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerStop);
                        currentStage = pickUpStages.travel;
                    }
                    break;
                case travel:
                    if(motorPickUpClawPort2.getCurrentPosition() >= MotorConstants.kMotorPickUpClawTravelPosition)
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerUp);
                    }
                    else
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerStop);
                        MoveToHomePosition();
                        currentStage = pickUpStages.release;
                    }
                    break;
                case release:
                    if(motorPickUpClawPort2.getCurrentPosition() >= MotorConstants.kMotorPickUpClawReleasePosition)
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerUp);
                    }
                    else
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerStop);
                        currentStage = pickUpStages.reset;
                    }
                    break;
                case reset:
                    if(motorPickUpClawPort2.getCurrentPosition() <= MotorConstants.kMotorPickUpClawResetPosition)
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerDown);
                    }
                    else
                    {
                        motorPickUpClawPort2.setPower(MotorConstants.kMotorPowerStop);
                        currentStage = pickUpStages.idle;
                        pickUpClawTriggered = false;
                    }
                    break;
            }
        }
        else
        {
            voltageForward = joystickForwardAnalog0.getVoltage();
            voltageBack = joystickBackAnalog1.getVoltage();
            voltageLeft = joystickLeftAnalog2.getVoltage();
            voltageRight = joystickRightAnalog3.getVoltage();

            if (voltageForward <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                //stop driven by encoder
                if(motorBackForwardPort0.getCurrentPosition() <= MotorConstants.kMotorBackForwardLimitFowardPosition)
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerForward);
                }
                else
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
                }
            }
            else if (voltageBack <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                //stop driven by limiting switch
                if(!stopSwitchBackDigital1.isPressed())
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerBack);
                }
                else
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
                    ResetMotorEncoder(motorBackForwardPort0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            else
            {
                motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
            }

            if (voltageLeft <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
                //stop driven by limiting switch
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
                //stop driven by limiting switch
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

        DisplayTelemetry();
    }

    private DcMotor CreateDCMotor(String motorName)
    {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }

    private void MoveToHomePosition()
    {
        while (!(stopSwitchBackDigital1.isPressed() & stopSwitchLeftDigital3.isPressed())) {
            if(motorBackForwardPort0.getPower() != MotorConstants.kMotorPowerBack)
            {
                if(!stopSwitchBackDigital1.isPressed())
                {
                    motorBackForwardPort0.setPower(MotorConstants.kMotorPowerBack);
                }
            }
            if(stopSwitchBackDigital1.isPressed())
            {
                motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
            }
            if(motorLeftRightPort1.getPower() != MotorConstants.kMotorPowerLeft)
            {
                if(!stopSwitchLeftDigital3.isPressed())
                {
                    motorLeftRightPort1.setPower(MotorConstants.kMotorPowerLeft);
                }
            }
            if(stopSwitchLeftDigital3.isPressed())
            {
                motorLeftRightPort1.setPower(MotorConstants.kMotorPowerStop);
            }
        }
        //Turn motors off in case timing exited the loop after limiting switches were checked
        //and therefore a motor could still be running
        motorBackForwardPort0.setPower(MotorConstants.kMotorPowerStop);
        motorLeftRightPort1.setPower(MotorConstants.kMotorPowerStop);

        //Reset the back-forward motor encoder
        ResetMotorEncoder(motorBackForwardPort0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DisplayTelemetry();
    }

    private void ResetMotorEncoder(DcMotor motor, DcMotor.RunMode previousMode)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Need to set the motor running mode back for normal operation
        motorBackForwardPort0.setMode(previousMode);
    }

    private void DisplayTelemetry()
    {
        telemetry.addData("Back/Forward Motor Encoder: ", motorBackForwardPort0.getCurrentPosition());
        telemetry.addData("Claw PickUp Motor Encoder: ", motorPickUpClawPort2.getCurrentPosition());
        telemetry.addData("Pickup pressed: ", pickUpClawTriggered);
        telemetry.update();
    }
}
