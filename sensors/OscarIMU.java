package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BaseOp;

import java.util.Locale;

@TeleOp(name = "Sensor: Oscar IMU", group = "Oscar")
@Disabled
public class OscarIMU extends BaseOp {

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {
        super.init();
        //telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();
    }

    @Override
    public void init_loop(){}

    @Override
    public void loop() {
        composeTelemetry();
        telemetry.update();
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void composeTelemetry() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        gravity = imu.getGravity();
        telemetry.addLine()
                .addData("heading", getHeading())
                .addData("roll", getRoll())
                .addData("pitch", getPitch());
        telemetry.addLine()
                .addData("miketest", angles.secondAngle);

    }

    public double getHeading() {
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }
    public double getRoll() {
        //return Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
        return 1.0;
    }
    public double getPitch() {
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
    }

}
