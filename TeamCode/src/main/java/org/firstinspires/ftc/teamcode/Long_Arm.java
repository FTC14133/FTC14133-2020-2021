package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class Long_Arm extends OpMode {
    private DcMotor LongArm = null;
    private float Timer = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LongArm = hardwareMap.get(DcMotor.class, "Long_Arm");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
    }

    @Override
    public void loop() {
        LongArm.setDirection(DcMotor.Direction.FORWARD);
        LongArm.setPower(4);
        Timer = Timer + 1;
        if (Timer > 5) {
            LongArm.setDirection(DcMotor.Direction.REVERSE);
            LongArm.setPower(4);

        }

    }


}