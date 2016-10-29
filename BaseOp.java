package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.ftccommon.SoundPlayer;


/**
 * Created by Ultra on 9/29/2016.
 */

public class BaseOp extends OpMode {


    DcMotor rightFront;
    DcMotor rightBack;  //names the motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter;
    DcMotor collector;
    public void init() //starts of the code, runs once
    {


        rightFront = hardwareMap.dcMotor.get("rightFront"); //tells the robot what the hardware is calling it
        rightFront.setDirection(DcMotor.Direction.REVERSE); // sets the direction

        shooter = hardwareMap.dcMotor.get("shooter"); //tells the robot what the hardware is calling it
        shooter.setDirection(DcMotor.Direction.FORWARD); // sets the direction


        collector = hardwareMap.dcMotor.get("collector"); //tells the robot what the hardware is calling it
        collector.setDirection(DcMotor.Direction.REVERSE); // sets the direction

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);




    }

    public void setAutoRunMode() // sets thing specific to autonomous
    {
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets the encoders to the right mode, tells the motor to run to the position it is given
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTeleRunMode()//sets things specific to teleop
    {
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // tells the motors to use encoders
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void ResetShooter() {

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }



    public void loop()// constantly running code

    {


    }

}
