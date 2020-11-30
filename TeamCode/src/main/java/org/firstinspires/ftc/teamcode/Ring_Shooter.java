package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class Ring_Shooter extends OpMode{
    private DcMotor Shooter = null;
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {
        Shooter = hardwareMap.get(DcMotor.class, "left_drive");
        Shooter.setDirection(DcMotor.Direction.FORWARD);
    }

    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start(){

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            Shooter.setPower(3);
        }
    }
}

