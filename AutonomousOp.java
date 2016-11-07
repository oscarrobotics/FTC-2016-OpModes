package org.firstinspires.ftc.teamcode;

import android.animation.TimeAnimator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseOp;

import java.sql.Time;
import java.util.Timer;

import com.qualcomm.robotcore.util.ElapsedTime;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Tank", group = "Oscar")
public class AutonomousOp extends BaseOp {
    public int position, encoderTarget;
    public double drivePower;
    public long beginTime = 0;
    public int particlesShot = 0;
    public boolean weDidit = false;

    public ElapsedTime mRuntime = new ElapsedTime();
    @Override
    public void init() {
        super.init();



        // Init variables
        position = shooter.getCurrentPosition();
        encoderTarget = 4950; // encoder target in native Ticks
        drivePower = 1.0;
        loader.setPosition(.15); // added to resolve loader servo not initialized properly - george

        // Init other stuff
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zeroes encoders
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // what purpose does this serve?
        rightFront.setTargetPosition((rightFront.getCurrentPosition()));
        rightBack.setTargetPosition((rightBack.getCurrentPosition())); // sets the target position of the encoders to the current position + target position
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
        buttonPress.setPosition(0.5);

        // Init commands
        setAutoRunMode();
        runToInit();

    }

    public void loop() {
        super.loop();

        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        /*
       if (((rightFront.getCurrentPosition() < encoderTarget) && (leftFront.getCurrentPosition()) < encoderTarget)) { // if we aren't at target, go full power
            rightFront.setPower(drivePower);
            leftFront.setPower(drivePower);
            rightBack.setPower(drivePower);
            leftBack.setPower(drivePower);
        }
        else { // once we reach target, stop
            rightFront.setPower(0.0);
            leftFront.setPower(0.0);
            rightBack.setPower(0.0);
            leftBack.setPower(0.0);
        }
        if (rightFront.getCurrentPosition() >= 4950) { // if we are at target, turn
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + 1650);
            rightBack.setTargetPosition(rightBack.getCurrentPosition() + 1650);
            rightFront.setPower(1.0);
            rightBack.setPower(1.0);
            leftBack.setPower(-1.0);
            leftFront.setPower(-1.0);
        }
        */
        /*
        if (particlesShot == 0) {
            shootParticle();
            beginTime = System.currentTimeMillis();
            particlesShot = 1;
        }
        if (particlesShot == 1 && (System.currentTimeMillis() > (beginTime + 2000))) {
            shootParticle();
            particlesShot = 2;
        }
        if(particlesShot == 2) {
            //Put everything else here
        }
        */
        if(!weDidit) { // if we haven't fired twice yet
            if(particlesShot == 0) { // if we haven't shot
                chrisAutoShoot(); // shoot
                particlesShot++; // we've now shot once
            }

            chrisAutoLoad(); // now load next ball

            if(particlesShot == 1) { // if we've already shot once
                chrisAutoShoot(); // shoot again
                weDidit = true; // we're done shooting
            }
        }
    }

    public void runToInit() {

        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void chrisAutoShoot() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterTargetPosition -= 3360;
        do {
            shooter.setTargetPosition(shooterTargetPosition);
        } while (shooter.getCurrentPosition() >= shooterTargetPosition + 10);
    }


    public void chrisAutoLoad(){ // make sure servo is initialized to .15
        if(shooter.getCurrentPosition() >= shooterTargetPosition) {
            loader.setPosition(.15);
            do {
                loader.setPosition(.5);
            } while (loader.getPosition() != .5);
        }
    }


    public void stopShooter() { //checks to see if the target
        shooter.setPower(0.0);
    }
}
