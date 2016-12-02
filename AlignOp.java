package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: AlignOp", group = "Oscar")
public class AlignOp extends BaseOp {

    public int v1, v2, v3, v4;
    double odValue;
    String leftValues, rightValues, sensorValues1;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        MecanumGamepadDrive();
        encoderTelemetry();
        whiteLineSensor();
        addTelemetry();

        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        if (gamepad1.back || gamepad2.back) {
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderTelemetry() {
        v1 = leftFront.getCurrentPosition();
        v2 = rightFront.getCurrentPosition();
        v3 = leftBack.getCurrentPosition();
        v4 = rightBack.getCurrentPosition();

        leftValues = "FrontL: " + v1 + " BackL: " + v3;
        rightValues = "FrontR: " + v2 + " BackR: " + v4;
    }

    public void whiteLineSensor() {
        odSensor.enableLed(true);
        odValue = odSensor.getLightDetected();
        sensorValues1 = "odSensor: " + odValue;
    }

    public void addTelemetry() {
        telemetry.addData("1", leftValues);
        telemetry.addData("2", rightValues);
        telemetry.addData("3", sensorValues1);
        telemetry.update();
    }

}
