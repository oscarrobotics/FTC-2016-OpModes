package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Tank - Linear", group = "Oscar")
public class LinearAutoOp extends LinearOpMode {

    // Variables
    public int shooterTargetPosition = 0;
    public boolean shooting = false;
    public boolean loading = false;
    public boolean isReady = false;

    // Motor block
    public DcMotor shooter;

    public DcMotor collector;

    public DcMotor rightFront;

    public DcMotor rightBack;

    public DcMotor leftFront;

    public DcMotor leftBack;


    // Servo block
    public Servo loader;

    public Servo buttonPress;

    public void stopShooter() {
        shooter.setPower(0.0);
    }

    public void runToInit() {
        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean shooterReady() {
        return shooter.getCurrentPosition() <= shooterTargetPosition + 10; // TODO: Fix if in reverse
    }

    public void shootParticle() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!shooting && shooterReady()) { // if NOT SHOOTING but READY TO SHOOT
            shooterTargetPosition -= 3360;  // TODO: Reverse motor and go positive?
            shooter.setTargetPosition(shooterTargetPosition);
            shooting = true;
            loading = false;
            if (loader.getPosition() != 0.15) loader.setPosition(.15);
            telemetry.addData("getting ready to shoot", "");
        } else if (shooting && shooterReady() && !loading) { // IF SHOOTING and READY TO SHOOT, try to load
            loading = true;
            loader.setPosition(0.5);
            telemetry.addData("Trying to Load", "");
        } else if (loading && loader.getPosition() <= 0.49) { // IF LOADING and LOADER NOT THERE
            loading = false;
            shooting = false;
            loader.setPosition(0.15);
            telemetry.addData("else done shooting", "");
        } else
            telemetry.addData("None of the Above", "");
    }

    public void isSet() {
        if (gamepad1.start || gamepad2.start) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isReady = true;
        }
    }

    public void stopRobot() {
        stopShooter();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        start();
        // Motor block
        shooter = hardwareMap.dcMotor.get("shooter");

        collector = hardwareMap.dcMotor.get("collector");

        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront = hardwareMap.dcMotor.get("leftFront");

        leftBack = hardwareMap.dcMotor.get("leftBack");


        // Servo block
        loader = hardwareMap.servo.get("loader");

        buttonPress = hardwareMap.servo.get("buttonPress");
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector.setDirection(DcMotor.Direction.REVERSE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loader.setPosition(0.0);
        buttonPress.setPosition(90.0);

        while (!isReady) {
            isSet();
        }
        runToInit();
        waitForStart();
        int didIt = 0;

        loader.setPosition(0.5);
        if (loader.getPosition() <= .49) { // if not LOADER AT .5
            loader.setPosition(.5); // GO TO .5
        } else if (loader.getPosition() == .5) { // ONCE LOADER AT .5
            shootParticle(); // SHOOT
        }
        stopRobot();
        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        telemetry.addData("did we do it?", didIt);
        updateTelemetry(telemetry);
        didIt = 1;

        idle();

        // Loop code
        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        telemetry.addData("did we do it?", didIt);

        updateTelemetry(telemetry);
    }
}





