package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class Conveyor_Belt extends OpMode {
    private DcMotor Conveyor_Belt = null;
    public void init() {
        Conveyor_Belt = hardwareMap.get(DcMotor.class, "left_drive");
    }


    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {
        if (gamepad2.left_bumper) {
            Conveyor_Belt.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt.setPower(4);
        }
        if (gamepad2.right_bumper) {
            Conveyor_Belt.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt.setPower(4);

        }
    }
}
