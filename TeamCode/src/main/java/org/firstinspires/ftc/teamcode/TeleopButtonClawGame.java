package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BCMConstants.AnalogInputConstants;
import org.firstinspires.ftc.teamcode.BCMConstants.MotorConstants;

/******************************************************/
/*        Hub Controller Status LED:                  */
/*          - Blue: robot code booting up             */
/*          - Green: robot code ready to run          */
/******************************************************/
@TeleOp(name="Teleop Button Claw Game", group="Iterative OpMode")

/* What we'll need
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

    DcMotor motorLeftRightP1;
    DcMotor motorBackForwardP0;



    @Override
    public void init()
    {
        //the deviceName parameter is the name defined in the Driver's Station
        joystickUpAnalog0 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickUp);
        joystickDownAnalog1 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickDown);
        joystickLeftAnalog2 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickLeft);
        joystickRightAnalog3 = hardwareMap.get(AnalogInput.class, AnalogInputConstants.kJoystickRight);

        motorLeftRightP1 = hardwareMap.get(DcMotor.class, MotorConstants.kMotorLeftRight);
        motorLeftRightP1.setDirection(DcMotor.Direction.FORWARD);
        motorLeftRightP1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackForwardP0 = hardwareMap.get(DcMotor.class, MotorConstants.kMotorBackForward);
        motorBackForwardP0.setDirection(DcMotor.Direction.FORWARD);
        motorBackForwardP0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        if (voltageUp <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
            motorBackForwardP0.setPower(0.15);
        }
        else if (voltageDown <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {
            motorBackForwardP0.setPower(0.15);
        }
        if (voltageLeft <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {

        }
        else if (voltageRight <= AnalogInputConstants.kVoltageJoystickEngagedThreshold) {

        }
    }
}
