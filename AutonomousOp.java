package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseOp;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Tank", group = "Oscar")
public class AutonomousOp extends BaseOp {
    public int position, encoderTarget;
    public double drivePower;

    @Override
    public void init() {
        super.init();

        // Init variables
        position = shooter.getCurrentPosition();
        encoderTarget = 4950; // encoder target in native Ticks
        drivePower = 1.0;

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

        // Init commands
        setAutoRunMode();
        runToInit();
    }

    public void loop() {
        super.loop();

        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos

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

    }

    public void runToInit() {
        if (gamepad1.a || gamepad2.a){
            shooter.setPower(50.0); // this won't work
        }
        if (gamepad1.b || gamepad2.b){
            shooter.setPower(-50.0);
        }
        if (gamepad1.start || gamepad2.start){ // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stopShooter() { //checks to see if the target
        shooter.setPower(0.0);
    }
}




