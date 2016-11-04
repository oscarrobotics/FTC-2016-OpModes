package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


/**
 * Created by Ultra on 9/29/2016.
 * Edited by Banks T on 11/4/2016.
 */

public class BaseOp extends OpMode {

    // Controllers
    DcMotorController frontController;
    DcMotorController backController;
    DcMotorController shooterController;
    ServoController servoController;
    DeviceInterfaceModule cdi;

    // Motors
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter;
    DcMotor collector;

    // Servos
    Servo loader;
    Servo buttonPress;

    // Sensors
    ColorSensor redBlueSensor; // Adafruit RGBW sensor
    ColorSensor odSensor; // Modern Robotics RGBW sensor

    public void init() { // runs when any OpMode is initalized, sets up everything needed to run robot

        // Controller block
        frontController = hardwareMap.dcMotorController.get("Motor Controller 1");

        backController = hardwareMap.dcMotorController.get("Motor Controller 2");

        shooterController = hardwareMap.dcMotorController.get("Motor Controller 3");

        servoController = hardwareMap.servoController.get("Servo Controller");

        cdi = hardwareMap.deviceInterfaceModule.get("Core Device Interface");
        cdi.setLED(2, true); // turn on Blue LED


        // Motor block
        shooter = hardwareMap.dcMotor.get("shooter"); // Shooter Motor
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector = hardwareMap.dcMotor.get("collector"); // Spinny Foam Thing Motor
        collector.setDirection(DcMotor.Direction.REVERSE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo block
        loader = hardwareMap.servo.get("loader");
        loader.setPosition(0.0);

        buttonPress = hardwareMap.servo.get("buttonPress");
        buttonPress.setPosition(90.0);


        // Sensor block
        redBlueSensor = hardwareMap.colorSensor.get("redBlueSensor");
        redBlueSensor.enableLed(false); // disable LED by default

        odSensor = hardwareMap.colorSensor.get("odSensor");
        odSensor.enableLed(false); // disable LED by default
    }

    public void setAutoRunMode() { // sets things specific to autonomous
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets the encoders to the right mode, tells the motor to run to the position it is given
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTeleRunMode() { //sets things specific to teleop
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // tells the motors to use encoders
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ResetShooter() {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
    }
}
