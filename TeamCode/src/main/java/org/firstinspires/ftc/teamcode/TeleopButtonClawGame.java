package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp(name="Teleop Button Claw Game", group="Iterative OpMode")

/* What we'll need
1. In the Driving Station, configure and name the 4 Analog Input ports
2. Define 4 analog input variables- with names more meaningful than my "analogTest"
3. Initialize the 4 analog input variables using hardwareMap.get()
4. Identify all subsystem components- motors, switches, etc
5. Create variables for each component
6. Initialize components
*/

public class TeleopButtonClawGame extends OpMode
{
    AnalogInput analogTest;
    double currentVoltage;

    @Override
    public void init()
    {
        //the deviceName parameter is the name defined in the Driving Station
        analogTest = hardwareMap.get(AnalogInput.class, "Analog0");
    }

    @Override
    public void loop() {
        currentVoltage = analogTest.getVoltage();
        telemetry.addData("Analog 0 voltage", currentVoltage);
        telemetry.update();
    }
}
