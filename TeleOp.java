package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ultra on 9/29/2016.
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


        Shoot();
        try {
            load();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        Collect();
        updateTelemetry(telemetry);
        telemetry.addData("1", "shooterPos: " + shooterEncoder());
        telemetry.addData("2", "loaderPos: " + servoLoader());
    }


    double scaleInput(double dVal) {
  return Math.abs(dVal) * dVal;

    }



    public void RobotDrive() {
        Double left = Math.pow(gamepad1.left_stick_y, 3),
                right = Math.pow(gamepad1.right_stick_y, 3),
                leftMot = leftFront.getPower(),
                rightMot = rightFront.getPower(),
                rampRate = 0.01;
        if (Math.abs(leftMot - left) > rampRate) {
            leftFront.setPower((leftMot + Math.signum((left - leftMot) * rampRate)));
            leftBack.setPower((leftMot + Math.signum((left - leftMot) * rampRate)));
        } else {
            leftFront.setPower(left);
            leftBack.setPower(left);
        }

        if (Math.abs(rightMot - right) > rampRate) {
            rightFront.setPower((rightMot + Math.signum((right - rightMot) * rampRate)));
            rightBack.setPower((rightMot + Math.signum((right - rightMot) * rampRate)));
        } else {
            rightFront.setPower(right);
            rightBack.setPower(right);
        }


    }


    public void Shoot() {
        if (gamepad2.right_trigger > 0.5) {
            shooter.setPower(1.0);
        } else
            shooter.setPower(0.0);
    }

    /* public void ShootEnc() {
        if (gamepad1.x ) {
            shooter.setTargetPosition(1120 + shooter.getCurrentPosition());
            shooter.setPower(.75);
        }
        else shooter.setPower(0.0);
    }
*/
    public void Collect() {
        if (gamepad1.a || gamepad2.a) {
            collector.setPower(-.75);
        } else if (gamepad1.b || gamepad2.b) {
            collector.setPower(.75);
        } else
            collector.setPower(0.0);

      /*
        if ( gamepad1.a || gamepad2.a)
            collector.setPower(.75);

        else
            collector.setPower(-.75);
        */

    }

     public void load() throws InterruptedException {


         if (gamepad2.dpad_down)
             loader.setPosition(.5);
             else
             loader.setPosition(.15);

     }

}


