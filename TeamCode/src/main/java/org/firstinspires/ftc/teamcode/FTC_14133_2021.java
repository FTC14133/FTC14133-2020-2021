package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

 @TeleOp(name="FTC 14133 2021", group="Iterative Opmode")
public class FTC_14133_2021 extends OpMode {
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftback = null;        // Sets the variables of the mecanum wheels
     private DcMotor rightback = null;
     private DcMotor lf = null;
     private DcMotor rightfront = null;

     // COMMENTED OUT THINGS ARE NOT TO BE DELETED
     static final double MOTOR_TICK_COUNT = 2800;
     private DcMotor Shooter = null;         // Sets the variable of the shooter
     private DcMotor LongArm = null;         // Sets the variable of the arm that is long but there is not a arm that is short
     private DcMotor intake = null;          // Sets the variable of the intake
     private DcMotor conveyor = null;          // Sets the variable of the conveyor
     DigitalChannel beamBreak;          // Sets the variable of the beamBreak
     Servo Claw = null;          // Sets the variable of the Claw
     boolean clawstate = false;          // Sets the variable of the clawstate
     boolean toggle = true;          // Sets the variable of the toggle
     double ShooterPower = 1;             // mayhaps
     DigitalChannel limit;


     public void init() {
         lf = hardwareMap.get(DcMotor.class, "lf");       //sets the names of the motors on the hardware map
         rightfront = hardwareMap.get(DcMotor.class, "rightfront");
         leftback = hardwareMap.get(DcMotor.class, "leftback");
         rightback = hardwareMap.get(DcMotor.class, "rightback");
         LongArm = hardwareMap.get(DcMotor.class, "LongArm");
         Shooter = hardwareMap.get(DcMotor.class, "Shooter");
         intake = hardwareMap.get(DcMotor.class, "intake");
         conveyor = hardwareMap.get(DcMotor.class, "conveyor");
         beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
         Claw = hardwareMap.get(Servo.class, "Claw");
         limit = hardwareMap.get(DigitalChannel.class, "limit");

         Shooter.setDirection(DcMotor.Direction.REVERSE);            //sets the directions of the motors
         lf.setDirection(DcMotor.Direction.FORWARD);
         rightfront.setDirection(DcMotor.Direction.REVERSE);
         leftback.setDirection(DcMotor.Direction.FORWARD);
         rightback.setDirection(DcMotor.Direction.REVERSE);
         beamBreak.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
         Claw.setPosition(1);
         LongArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         LongArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up
         LongArm.setDirection(DcMotor.Direction.FORWARD);
         intake.setDirection(DcMotor.Direction.FORWARD);
         conveyor.setDirection(DcMotor.Direction.FORWARD);




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
         double armrotation = MOTOR_TICK_COUNT * (0.375);
         double armrotationmiddle = MOTOR_TICK_COUNT * (0.25);

         telemetry.addData("Is pressed?    ", limit.getState());

         telemetry.update();



         //Mecanum Wheels

         leftPowerY = -gamepad1.left_stick_y;      //find the value of y axis on the left joystick
         leftPowerX = gamepad1.left_stick_x;      //find the value of x axis on the left joystick
         rightPowerX = gamepad1.right_stick_x;      //find the value of x axis on the right joystick

         leftfrontpower = leftPowerY + leftPowerX - rightPowerX;     //Power of Mecanum wheels
         rightfrontpower = leftPowerY - leftPowerX + rightPowerX;
         leftbackpower = leftPowerY - leftPowerX - rightPowerX;
         rightbackpower = leftPowerY + leftPowerX + rightPowerX;

         NormScaling = Math.max(Math.max(Math.abs(leftfrontpower), Math.abs(rightfrontpower)), Math.max(Math.abs(leftbackpower), Math.abs(rightbackpower)));     //This line of code gets the max of the the absolute values of the power of the wheels. WARNING DO NOT TRY THIS AT HOME

         if (NormScaling > 1) {
             leftfrontpower /= NormScaling;      //If the max of the the absolute values of the power of the wheels is greater than 1 it will divide all of the powers of the wheels by the max of the the absolute values of the power of the wheels
             rightfrontpower /= NormScaling;
             leftbackpower /= NormScaling;
             rightbackpower /= NormScaling;
         }
         lf.setPower(leftfrontpower);     //Sets the power of the wheels
         leftback.setPower(leftbackpower);
         rightfront.setPower(rightfrontpower);
         rightback.setPower(rightbackpower);




        //Claw

        if (toggle && gamepad2.y) {  // Only execute once per Button push
            toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            if (clawstate) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                clawstate = false;
                Claw.setPosition(1);
            } else {
                clawstate = true;
                Claw.setPosition(0);
            }
        } else if (!gamepad2.y) {
            toggle = true; // Button has been released, so this allows a re-press to activate the code above.
        }



         //Long Arm

         if (gamepad2.dpad_down) {            //turns the arm that is long but there is not a arm that is short
             LongArm.setTargetPosition((int) armrotation);        //Tell the motor to go to 90 degrees when told to
             LongArm.setPower(0.3);        //Sets the power for the Long arm
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }
         if (gamepad2.dpad_up) {      //rotates the arm that is long but there is not a arm that is short
             LongArm.setTargetPosition(0);        //Tell the motor to go to 90 degrees when told to
             LongArm.setPower(0.3);        //Sets the power for the Long arm
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }

         if (gamepad2.dpad_left || gamepad2.dpad_right) {      //rotates the arm that is long but there is not a arm that is short
             LongArm.setTargetPosition((int) armrotationmiddle);
             LongArm.setPower(0.3);        //Sets the power for the Long arm
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }

         if (gamepad2.a) {
             LongArm.setTargetPosition(-1400);        //Tell the motor to go to 90 degrees when told to
             LongArm.setPower(0.3);
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       }

         if (limit.getState()) {
             LongArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             LongArm.setTargetPosition(40);        //Tell the motor to go to 90 degrees when told to
             LongArm.setPower(0.3);
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }









        //Shooter and Shooter Power

        if (gamepad2.left_stick_y < -0.25) {
            ShooterPower += 0.01;
        }

        if (gamepad2.left_stick_y > 0.25) {
            ShooterPower -= 0.01;
        }

        if (ShooterPower > 1) {
            ShooterPower = 1;
        }

        if (ShooterPower < 0.75) {
            ShooterPower = 0.75;
        }

        if (gamepad2.b) {
            Shooter.setPower(ShooterPower);            // This Controls the shooter
            intake.setPower(1);
            conveyor.setPower(1);
        }





        //Intake and Conveyor and Conveyor Detection System

        if (gamepad2.right_trigger > 0) {       //runs the intake forward
            intake.setPower(1);
        }

        if (gamepad2.left_trigger > 0) {        //runs the intake backwards
            intake.setPower(-1);
            conveyor.setPower(-1);
        }

        if (!beamBreak.getState()) {
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
            Shooter.setPower(0.75);
        }
    }
}
