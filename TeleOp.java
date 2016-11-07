package org.firstinspires.ftc.teamcode;


/**
 * Created by Ultra on 9/29/2016.
 * Edited by Banks T on 11/3/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Tank", group = "Oscar")
// sets what it will appear in the driverstation
public class TeleOp extends BaseOp {

    public void init() {
        super.init(); // overrides the BaseOp init
        setTeleRunMode(); // uses what is in BaseOp
    }

    @Override
    public void loop() {

        super.loop();
        RobotDrive();
        Shoot();

        // ShootEnc();
        Collect();
        Load();
        ButtonPress();
        updateTelemetry(telemetry);              //sends feedback

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
            shootParticle();
        }
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
        if (gamepad2.dpad_right) {
            buttonPress.setPosition(0.0);
        } else if (gamepad2.dpad_left) {
            buttonPress.setPosition(1.0);
        } else {
            buttonPress.setPosition(.5);
        }
    }

    public void Collect() { // big spinny foam dude
        if (gamepad2.b) {
            collector.setPower(0.0);
        } else if (gamepad2.a) {
            collector.setPower(1.0);
        } else {
            collector.setPower(-1.0);
        }
    }
}