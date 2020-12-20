/**package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class Long_Arm extends OpMode {
    private DcMotor LongArm = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
/**    @Override
    public void init() {
        LongArm = hardwareMap.get(DcMotor.class, "Long_Arm");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
 /**   @Override
    public void start() {

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
 //   }

 /**   @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            LongArm.setDirection(DcMotor.Direction.FORWARD);        //sets the long arm forward
            LongArm.setPower(3);
        } else if (gamepad1.left_bumper) {
            LongArm.setDirection(DcMotor.Direction.REVERSE);        //sets the long arm backwards
            LongArm.setPower(3);
        }
        else {
            LongArm.setPower(0);// This tells the program to set the Intake, Long Arm, and Shooter
                                //to turn them off when not being used
        }

    }


}
  */