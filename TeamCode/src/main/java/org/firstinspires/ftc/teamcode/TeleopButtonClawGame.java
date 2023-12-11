package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
    CRServo servoMotorPickUpClaw;
    /****************************************/

    boolean pickUpClawTriggered = false;
    int servoPort;

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
        servoMotorPickUpClaw = CreateCRServoMotor(MotorConstants.kServoPickUpClaw);

        //Initialize the Button Claw Machine
        MoveToHomePosition();
        MovePickUpClawToPosition(MotorConstants.kServoTravelPosition);
    }

    @Override
    public void loop() {
        if(pickUpClawButton.isPressed())
        {
            pickUpClawTriggered = true;
            MovePickUpClawToPosition(MotorConstants.kServoPickUpPosition);
            MovePickUpClawToPosition(MotorConstants.kServoTravelPosition);
            MoveToHomePosition();
            MovePickUpClawToPosition(MotorConstants.kServoReleasePosition);
            MovePickUpClawToPosition(MotorConstants.kServoTravelPosition);
            pickUpClawTriggered = false;
        }

        if (!pickUpClawTriggered)
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

    private CRServo CreateCRServoMotor(String servoName)
    {
        CRServo servo = hardwareMap.get(CRServo.class, servoName);
        servoPort = servo.getPortNumber();
        servo.setPower(MotorConstants.kServoPowerOn);

        return servo;
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

        MovePickUpClawToPosition(MotorConstants.kServoTravelPosition);
    }

    private void ResetMotorEncoder(DcMotor motor, DcMotor.RunMode previousMode)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Need to set the motor running mode back for normal operation
        motorBackForwardPort0.setMode(previousMode);
    }

    private void MovePickUpClawToPosition(double targetPosition)
    {
        servoMotorPickUpClaw.getController().setServoPosition(servoPort, targetPosition);
    }

    private void DisplayTelemetry()
    {
        telemetry.addData("Back/Forward Motor Encoder: ", motorBackForwardPort0.getCurrentPosition());
        telemetry.addData("Analog 0 'Forward' voltage: ", voltageForward);
        telemetry.addData("Analog 1 'Back' voltage: ", voltageBack);
        telemetry.addData("Analog 2 'Left' voltage: ", voltageLeft);
        telemetry.addData("Analog 3 'Right' voltage: ", voltageRight);
        telemetry.addData("Servo Position: ", servoMotorPickUpClaw.getController().getServoPosition(0));
        telemetry.update();
    }
}
