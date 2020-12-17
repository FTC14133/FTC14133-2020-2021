package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    DigitalChannel LimitSwitchLongArm ;
    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "left_drive");       //sets the names of the motors on the hardware map
        rightfront = hardwareMap.get(DcMotor.class, "right_drive");
        leftback  = hardwareMap.get(DcMotor.class, "left_drive");
        rightback = hardwareMap.get(DcMotor.class, "right_drive");
        LongArm = hardwareMap.get(DcMotor.class, "Long_Arm");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Conveyor_Belt_Inner = hardwareMap.get(DcMotor.class, "Conveyor_Belt");
        Conveyor_Belt_Outer = hardwareMap.get(DcMotor.class, "Conveyor_Belt");
        LimitSwitchLongArm = hardwareMap.get(DigitalChannel.class, "LimitSwitchLongArm");

        Shooter.setDirection(DcMotor.Direction.FORWARD);            //sets the directions of the motors
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        LimitSwitchLongArm.setMode(DigitalChannel.Mode.INPUT);

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
        double leftfrontpower;      //Power level for leftfront
        double rightbackpower;      //Power level for rightback
        double leftbackpower;       //Power level for leftback
        double rightfrontpower;     //Power level for rightfront
        int ClawButton = 1;

        leftPowerY  = -gamepad1.left_stick_y ;      //find the value of y axis on the left joystick
        leftPowerX  = gamepad1.left_stick_x ;      //find the value of x axis on the left joystick
        rightPowerX = gamepad1.right_stick_x ;      //find the value of x axis on the right joystick

        leftfrontpower = leftPowerY + leftPowerX + rightPowerX;
        rightfrontpower = leftPowerY - leftPowerX - rightPowerX;
        leftbackpower = leftPowerY + leftPowerX - rightPowerX;
        rightbackpower = leftPowerY - leftPowerX + rightPowerX;

        NormScaling = Math.max(Math.max(Math.abs(leftfrontpower), Math.abs(rightfrontpower)), Math.max(Math.abs(leftbackpower), Math.abs(rightbackpower)));
        if (NormScaling == 0) {}
        if (NormScaling > 0) {
            leftfrontpower /= NormScaling;
            rightfrontpower /= NormScaling;
            leftbackpower /= NormScaling;
            rightbackpower /= NormScaling;

            leftfront.setPower(leftfrontpower);
            leftback.setPower(leftbackpower);
            rightfront.setPower(rightfrontpower);
            rightback.setPower(rightbackpower);
        }

        Servo Claw = null;
        Servo Stopper = null;

        if(gamepad2.y) {
            ClawButton = ClawButton * -1;
        }
        if(ClawButton == 1){
            Claw.setPosition(0);
        }
        if(ClawButton == -1){
            Claw.setPosition(90);
        }


        if (gamepad2.right_bumper) {
            LongArm.setDirection(DcMotor.Direction.FORWARD);       //sets the long arm forward
            LongArm.setPower(1);
        } else if (gamepad2.left_bumper) {
            LongArm.setDirection(DcMotor.Direction.REVERSE);        //sets the long arm backwards
            LongArm.setPower(1);
        }


        if (gamepad2.b) {
            Shooter.setPower(1);            // This Controls the shooter
            Stopper.setPosition(90);        // This sets the Stopper to allow rings to come in the Shooter
        }



        if (gamepad1.left_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.FORWARD);        // This makes the intake run forward
            Conveyor_Belt_Inner.setPower(1);
            Conveyor_Belt_Outer.setPower(1);
        }
        if (gamepad1.right_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.REVERSE);        // This makes the intake run backward
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt_Inner.setPower(1);
            Conveyor_Belt_Outer.setPower(1);
        }
        else {
            Conveyor_Belt_Outer.setPower(0);        // This tells the program to set the Intake, Long Arm, and Shooter
            Conveyor_Belt_Inner.setPower(0);        //to turn them off when not being used
            Stopper.setPosition(0);
            LongArm.setPower(0);
            Shooter.setPower(0);
        }


            }
        }


