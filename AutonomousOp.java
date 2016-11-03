package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseOp;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Tank", group = "Oscar")
public class AutonomousOp extends BaseOp {
    // public int rightFrontEncoderTarget= 5000;
    public int position = shooter.getCurrentPosition();

    @Override
    public void init() {
        super.init();
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the encoders to zero
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int rightFrontEncoderTarget = 5000;
        int rightBackEncoderTarget = 5000;
        int leftFrontEncoderTarget = 5000; // tells it how many ticks to go
        int leftBackEncoderTarget = 5000;
        setAutoRunMode();
        runToInit();
    }

    public void loop() {
        super.loop();


        rightFront.setTargetPosition((rightFront.getCurrentPosition()));
        rightBack.setTargetPosition((rightBack.getCurrentPosition())); // sets the target position of the encoders to the current position + target position
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());

       /* if (((rightFront.getCurrentPosition() < 4950) && (leftFront.getCurrentPosition()) < 4950)) {
            rightFront.setPower(1.0);
            leftFront.setPower(1.0);
            rightBack.setPower(1.0);
            leftBack.setPower(1.0); // sets the motor power
        } else {
            rightFront.setPower(0.0);
            leftFront.setPower(0.0);
            rightBack.setPower(0.0);
            leftBack.setPower(0.0);

        }

        if (rightFront.getCurrentPosition() >= 4950) {
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + 1650);
            rightBack.setTargetPosition(rightBack.getCurrentPosition() + 1650);


            rightFront.setPower(1.0);
            rightBack.setPower(1.0);
            leftBack.setPower(-1.0);
            leftFront.setPower(-1.0);
        }
        */
    }

    public void runToInit(){
        if (gamepad1.a || gamepad2.a){
            shooter.setPower(50.0);
        }
        if (gamepad1.b || gamepad2.b){
            shooter.setPower(-50.0);
        }
        if (gamepad1.start || gamepad2.start){
            shooter.getCurrentPosition();
        }
    }

    public void stopShooter() {//checks to see if the target
        shooter.setPower(0.0);
    }
}




