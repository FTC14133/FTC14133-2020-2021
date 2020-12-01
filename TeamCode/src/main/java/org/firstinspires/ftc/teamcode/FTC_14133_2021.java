package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class FTC_14133_2021 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftback = null;
    private DcMotor rightback = null;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor Shooter = null;
    HardwarePushbot robot = new HardwarePushbot();
    private DcMotor LongArm = null;
    private DcMotor Conveyor_Belt_Inner = null;
    private DcMotor Conveyor_Belt_Outer = null;
    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "left_drive");
        rightfront = hardwareMap.get(DcMotor.class, "right_drive");
        leftback  = hardwareMap.get(DcMotor.class, "left_drive");
        rightback = hardwareMap.get(DcMotor.class, "right_drive");
        LongArm = hardwareMap.get(DcMotor.class, "Long_Arm");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Conveyor_Belt_Inner = hardwareMap.get(DcMotor.class, "Conveyor_Belt");
        Conveyor_Belt_Outer = hardwareMap.get(DcMotor.class, "Conveyor_Belt");

        Shooter.setDirection(DcMotor.Direction.FORWARD);
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
        double leftfront;
        double rightback;
        double leftback;
        double rightfront;

        leftPowerY  = -gamepad1.left_stick_y ;
        leftPowerX  = gamepad1.left_stick_x ;
        rightPowerX = gamepad1.right_stick_x ;

        leftfront = leftPowerY + leftPowerX + rightPowerX;
        rightfront = leftPowerY - leftPowerX - rightPowerX;
        leftback = leftPowerY + leftPowerX - rightPowerX;
        rightback = leftPowerY - leftPowerX + rightPowerX;

        NormScaling = Math.max(Math.max(Math.abs(leftfront), Math.abs(rightfront)), Math.max(Math.abs(leftback), Math.abs(rightback)));
        leftfront = leftfront/=NormScaling;
        rightfront = rightfront/=NormScaling;
        leftback = leftback/=NormScaling;
        rightback = rightback/=NormScaling;

        Servo Claw = null;
        Servo StopperS = null;
        float StopperB = 0;

        if(gamepad2.y) {
            // move to 0 degrees.
            Claw.setPosition(0);
        } else if (gamepad2.x) {
            // move to 90 degrees.
            Claw.setPosition(0.5);
        } else if (gamepad2.a) {
            // move to 180 degrees.
            Claw.setPosition(1);
        }



        if (gamepad2.right_bumper) {
            LongArm.setDirection(DcMotor.Direction.FORWARD);
            LongArm.setPower(3);
        } else if (gamepad2.left_bumper) {
            LongArm.setDirection(DcMotor.Direction.REVERSE);
            LongArm.setPower(3);
        }



        if (gamepad2.b) {
            Shooter.setPower(3);
        }



        if (gamepad1.left_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt_Inner.setPower(5);
            Conveyor_Belt_Outer.setPower(5);
        }
        if (gamepad1.right_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt_Inner.setPower(5);
            Conveyor_Belt_Outer.setPower(5);
        }


        if (gamepad1.y) {
            StopperB = 1;
        if (gamepad1.a) {
            StopperB = 0;
            }
        if (StopperB == 1) ;{
            StopperS.setPosition(90);
            }
        if (StopperB == 0){
            StopperS.setPosition(0);
        }
                }
    }
}
