package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="FTC 14133 2021", group="Iterative Opmode")
public class Conveyor_Belt extends OpMode {

    private DcMotor intake = null;          // Sets the variable of the intake
    private DcMotor conveyor = null;          // Sets the variable of the conveyor
    DigitalChannel beamBreak;          // Sets the variable of the beamBreak


    public void init() {

        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        beamBreak.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

    }


    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {

        //Intake and Conveyor and Conveyor Detection System

        if (gamepad2.right_trigger > 0) {       //runs the intake forward
            intake.setPower(1);
        }

        if (gamepad2.left_trigger > 0) {        //runs the intake backwards
            intake.setPower(-1);
            conveyor.setPower(-1);
        }

        if (beamBreak.getState()) {
            if (gamepad2.left_trigger > 0) {        //BEAM BREAK grace can you do this
                conveyor.setPower(-1);
            } else {
                conveyor.setPower(1);
            }
        }
        //Big Else Statement

        else {
            conveyor.setPower(0);        // This tells the program to set the Intake, Long Arm, and Shooter
            intake.setPower(0);        //to turn them off when not being used

        }
    }
}