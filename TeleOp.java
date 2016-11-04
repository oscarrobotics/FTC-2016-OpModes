package org.firstinspires.ftc.teamcode;


/**
 * Created by Ultra on 9/29/2016.
 * Edited by Banks T on 11/3/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Tank", group = "Oscar")
// sets what it will appear in the driverstation
public class TeleOp extends BaseOp {

    //AMSColorSensor

    public void init() {
        super.init(); // overrides the BaseOp init
        setTeleRunMode(); // uses what is in BaseOp
    }

    @Override
    public void loop() {

        super.loop();
        /*
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        rightFront.setPower(right); // sets the power of rightFront motor to the y axis of the right stick
        rightBack.setPower(right);
        leftFront.setPower(left);  // sets the power of the leftFront motor to the y axis of the left stick
        leftBack.setPower(left);
        */
        RobotDrive();
        Shoot();

        // ShootEnc();
        Collect();
        Load();
        ButtonPress();
        updateTelemetry(telemetry);              //sends feedback

    }

    double scaleInput(double dVal) {
        /*double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
       if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }


        // return scaled value. */
        return Math.abs(dVal) * dVal;


    }

    public void RobotDrive() {

        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        leftPower = Math.pow(leftPower, 3);
        rightPower = Math.pow(rightPower, 3);

        rightFront.setPower(rightPower);
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);

    }


    public void Shoot() {
        if (gamepad2.right_trigger > 0.5) {
            shooter.setPower(1.0);
        } else
            shooter.setPower(0.0);
    }

    /*
    public void ShootEnc() {
        if (gamepad1.right_trigger > 0.5)
            shooter.setTargetPosition(1120 + shooter.getCurrentPosition());
    }
    */

    public void Load() { // loader servo
        if (gamepad2.dpad_down) {
            loader.setPosition(.5);
        } else {
            loader.setPosition(0.15);

        }
    }




    public void ButtonPress() {
        if (gamepad2.dpad_right){
            buttonPress.setPosition(0.0);
        }

        else if (gamepad2.dpad_left){
            buttonPress.setPosition(1.0);
        }

        else{
            buttonPress.setPosition(.5);
        }
    }

    public void Collect() { // big spinny foam dude
        if (gamepad2.b) {
            collector.setPower(0.0);
        }
        else if (gamepad2.a) {
            collector.setPower(1.0);
        }
        else {
            collector.setPower(-1.0);
        }
    }
}

