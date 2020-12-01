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
    private DcMotor Conveyor_Belt_Inner = null;
    private DcMotor Conveyor_Belt_Outer = null;
    public void init() {
        Conveyor_Belt_Inner = hardwareMap.get(DcMotor.class, "Conveyor_Belt_Inner");
        Conveyor_Belt_Outer = hardwareMap.get(DcMotor.class, "Conveyor_Belt_Outer");
    }

    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {
        if (gamepad2.left_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.FORWARD);
            Conveyor_Belt_Inner.setPower(5);
            Conveyor_Belt_Outer.setPower(5);
        }
        if (gamepad2.right_bumper) {
            Conveyor_Belt_Inner.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt_Outer.setDirection(DcMotor.Direction.REVERSE);
            Conveyor_Belt_Inner.setPower(5);
            Conveyor_Belt_Outer.setPower(5);

        }
    }
}
