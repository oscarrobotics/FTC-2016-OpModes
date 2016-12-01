package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Auto 2 Ball", group = "Oscar")
public class Auto2Ball extends BaseOp {
    public int position, encoderTarget;
    public double drivePower;
    public int particlesShot = 0;
    public boolean weDidIt = false;

    //public ElapsedTime mRuntime = new ElapsedTime();
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
        beaconPress.setPosition(0.5);

        // Init commands
        setAutoRunMode();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    public void loop() {
        super.loop();

        // Telemetry block
        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        telemetry.addData("particlesShot", particlesShot); // telemetry for how many shots we've taken
        telemetry.update(); // update telemetry

        // Auto code block
        auto2Ball(); // Shoot 2 balls
        if (weDidIt && particlesShot == 2) { // Do this after we finish shooting
            telemetry.addData("4", "Hi");
        }
    }

    public void auto2Ball() {
        if(!weDidIt) { // if we haven't fired twice yet
            if(particlesShot == 0) { // if we haven't shot
                chrisAutoShoot(); // shoot
                particlesShot++; // we've now shot once
            }

            chrisAutoLoad(); // now load next ball
            //waitFor(125);
            if(particlesShot == 1) { // if we've already shot once
                chrisAutoShoot(); // shoot again
                weDidIt = true; // we're done shooting
                particlesShot++;
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

    public static void waitFor(long ms) { // this is the only part that may not work on the robot
        try { TimeUnit.MILLISECONDS.sleep(ms);	}
        catch (InterruptedException e) {
            e.printStackTrace(); }
    }
}




