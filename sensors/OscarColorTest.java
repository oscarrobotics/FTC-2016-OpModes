package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * Created by mike on 12/1/16.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Color Test", group = "Oscar")
public class OscarColorTest extends OpMode {

    AMSColorSensor redBlueSensor;

    @Override
    public void init() {

        redBlueSensor  = (AMSColorSensor)hardwareMap.colorSensor.get("redBlueSensor"); // Use proper name here
        /*AMSColorSensor.Parameters params = AMSColorSensor.Parameters.createForAdaFruit();
        params.integrationTime = AMSColorSensor.IntegrationTime.MS_24;
        redBlueSensor.initialize(params);*/
    }

    @Override
    public void loop() {
        redBlueSensor.enableLed(gamepad1.x);
        telemetry.addLine()
                .addData("R", redBlueSensor.red())
                .addData("G", redBlueSensor.green())
                .addData("B", redBlueSensor.blue())
                .addData("alpha", redBlueSensor.alpha());
    }
}
