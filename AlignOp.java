package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: AlignOp", group = "Oscar")
public class AlignOp extends BaseOp {

    public int v1, v2, v3, v4;
    String leftValues, rightValues;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        MecanumGamepadDrive();
        encoderTelemetry();

        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void encoderTelemetry() {
        v1 = leftFront.getCurrentPosition();
        v2 = rightFront.getCurrentPosition();
        v3 = leftBack.getCurrentPosition();
        v4 = rightBack.getCurrentPosition();

        leftValues = "FrontL: " + v1 + " BackL: " + v3;
        rightValues = "FrontR: " + v2 + " BackR: " + v4;
        telemetry.addData("1", leftValues);
        telemetry.addData("2", rightValues);
        telemetry.update();
    }
}
