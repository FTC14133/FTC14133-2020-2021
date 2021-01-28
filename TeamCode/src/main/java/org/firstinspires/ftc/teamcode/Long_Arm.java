package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class Long_Arm extends OpMode {
    private DcMotor LongArm = null;
    static final double MOTOR_TICK_COUNT = 2800;
    DigitalChannel LimitSwitchLongArm;          // Sets the variable of the LimitSwitchLongArm

    @Override
    public void init() {
        LongArm = hardwareMap.get(DcMotor.class, "Long_Arm");
        LongArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up

        LongArm.setDirection(DcMotor.Direction.FORWARD);
        LimitSwitchLongArm.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        double armrotation = MOTOR_TICK_COUNT * (90 / 360);

        if (gamepad2.right_bumper) {            //turns the arm that is long but there is not a arm that is short
            LongArm.setTargetPosition((int) armrotation);        //Tell the motor to go to 90 degrees when told to
            LongArm.setPower(1);        //Sets the power for the Long arm
            LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.left_bumper) {      //rotates the arm that is long but there is not a arm that is short
            LongArm.setTargetPosition(0);        //Tell the motor to go to 90 degrees when told to
            LongArm.setPower(1);        //Sets the power for the Long arm
            LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }


}