package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

/**
 * Created by Banks & Chris
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Auto Test", group = "Oscar")
public class AutoTest extends BaseOp {

    public int position, encoderTarget, timesMoved;
    public static int particlesToShoot = 15; // change this to adjust how many times we shoot
    public static int particlesShot = 0; // counter for how many shots we've done
    public static int firstPTS = Math.min(1, particlesToShoot);
    public static int lastPTS = Math.max(1, particlesToShoot);
    public double drivePower;
    public boolean didWeMoveYet = (timesMoved >= 1); // if timesMoved is 1 or greater, we've moved
    public boolean doneShooting = false;


    @Override
    public void init() {
        super.init();

        // Init variables
        position = shooter.getCurrentPosition();
        encoderTarget = 4950; // encoder target in native Ticks

        drivePower = 1.0;
        // Init motor settings
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zeroes encoders
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setTargetPosition((rightFront.getCurrentPosition()));
        rightBack.setTargetPosition((rightBack.getCurrentPosition()));
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());

        // Init servo settings
        beaconPress.setPosition(0.5);
        loader.setPosition(0.15);

        // Init commands
        setAutoRunMode();
        init_loop();
    }

    public void loop() {
        super.loop();

        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        telemetry.update();

        // Shooter (This will happen before anything else, and only runs once)
        if (!doneShooting) { // if we haven't shot all particles yet
            autoMultiBall(); // shoot em
        }

        // First move (This will only happen after shooting, and only runs once)
        if (!didWeMoveYet && doneShooting) { // if we haven't moved yet, but we're done shooting
            turnLeft(90); // turn left 90 degrees
            stopDriving();
            // Uncomment below line after testing all the drive commands
            //timesMoved = 1; // we've now moved once
        }
        // Second and subsequent moves
        // TODO: test these on a real robot. Also how would we break between two cases to do something else entirely? i.e. press the beacon
        switch (timesMoved) {
            default:
                break; // if 'timesMoved' isn't 1 or greater, don't execute any movement actions
            case 1: // moved once
                forward(5); // move forward 5 inches
                stopDriving();
                timesMoved++;

            case 2: // moved twice
                turnRight(45); // turn right 45 degress
                stopDriving();
                timesMoved++;

            case 3: // moved three times
                forward(5); // move forward 5 inches
                stopDriving();
                break;
        }
    }

    // Methods we use in the auto loop
    public void waitFor(long ms) { // this is the only part that may not (probably won't) work on the robot
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void autoMultiBall() {
        for (particlesShot = 0; particlesShot <= (lastPTS - 1); ) { // shoot 'particlesToShoot' times
            chrisAutoShoot();
            waitFor(500); // wait 0.5sec after shot before loading
            if (particlesShot >= firstPTS && particlesShot < lastPTS) { // do not load on the first and last shot
                chrisAutoLoad();
            }
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


    public void chrisAutoLoad() { // make sure servo is initialized to .15
        if (shooter.getCurrentPosition() >= shooterTargetPosition) {
            loader.setPosition(.15);
            do {
                loader.setPosition(.5);
            } while (loader.getPosition() != .5);
        }
    }

    public void forward(double a) {
        final int move = ((int)(a * 44.8));
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition(rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition(rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition(rightFront.getCurrentPosition() + move);

        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);
        }
        stopDriving();
    }

    public void turnRight(double a) {
        final int move = ((int)(a * 44.8 * .1351944)); // TODO: Maybe shorten this by pre-multiplying these two values
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition(rightFront.getCurrentPosition() - move);
        rightBack.setTargetPosition(rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition(rightFront.getCurrentPosition() - move);

        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(-.75);
            leftBack.setPower(-.75);
        }
        stopDriving();
    }

    public void turnLeft(double a) {
        final int move = ((int)(a * 44.8 * .1351944)); // TODO: Maybe shorten this by pre-multiplying these two values
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - move);
        leftFront.setTargetPosition(rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition(rightFront.getCurrentPosition() - move);
        leftBack.setTargetPosition(rightFront.getCurrentPosition() + move);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(-.75);
            rightBack.setPower(-.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);
        }
        stopDriving();
    }

    public void stopDriving() {
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);

    }
}



