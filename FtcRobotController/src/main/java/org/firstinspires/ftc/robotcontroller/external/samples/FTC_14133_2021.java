package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FTC_14133_2021 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftback = null;
    private DcMotor rightback = null;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private int NormScaling = 0 ;


    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "left_drive");
        rightfront = hardwareMap.get(DcMotor.class, "right_drive");
        leftback  = hardwareMap.get(DcMotor.class, "left_drive");
        rightback = hardwareMap.get(DcMotor.class, "right_drive");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
    }


    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {
        double leftPowerY;
        double leftPowerX;
        double rightPowerX;
        double NormScaling;
        if (leftfront =>` 1) {
            // Find the largest power
            double max = 0;
        leftPowerY  = -gamepad1.left_stick_y ;
        leftPowerX  = -gamepad1.left_stick_x ;
        rightPowerX = gamepad1.right_stick_x ;
        NormScaling = Math.max(leftfront, rightfront, leftback, rightback);
        leftfront = leftfront/=NormScaling;
        rightfront = rightfront/=NormScaling;
        leftback = leftback/=NormScaling;
        leftfront = rightback/=NormScaling;



        Servo Claw = null;

        if(gamepad1.y) {
            // move to 0 degrees.
            Claw.setPosition(0);
        } else if (gamepad1.x) {
            // move to 90 degrees.
            Claw.setPosition(0.5);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            Claw.setPosition(1);
        }
    }

    private void max(DcMotor leftfront, DcMotor rightfront, DcMotor leftback, DcMotor rightback) {
    }

    private void max() {
    }
}
