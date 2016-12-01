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
    I2cDevice i2cDevice;
    i2cDevice = hardwareMap.get(I2cDevice.class, "redBlueSensor"); // Use proper name here
    AMSColorSensor.Parameters params = AMSColorSensor.Parameters.createForAdaFruit();
    params.integrationTime = AMSColorSensor.IntegrationTime.MS_24;
    redBlueSensor = AMSColorSensorImpl.create(params, i2cDevice);
  }

  @Override
  public void loop() {
    redBlueSensor.enableLed(gamepad1.x);
    telemetry.addData("R",  redBlueSensor.red());
    telemetry.addData("G",  redBlueSensor.green());
    telemetry.addData("B",  redBlueSensor.blue());
    telemetry.addData("alpha",  redBlueSensor.alpha());
  }
}
