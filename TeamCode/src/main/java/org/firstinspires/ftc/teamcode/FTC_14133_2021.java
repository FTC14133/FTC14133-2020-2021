package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;

    @TeleOp(name="FTC 14133 2021", group="Iterative Opmode")
    public class FTC_14133_2021 extends OpMode {
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotorEx lb = null;        // Sets the variables of the mecanum wheels
     private DcMotorEx rb = null;
     private DcMotorEx lf = null;
     private DcMotorEx rf = null;

     // COMMENTED OUT THINGS ARE NOT TO BE DELETED
     static final double MOTOR_TICK_COUNT = 2800;
     private DcMotorEx shooter = null;         // Sets the variable of the shooter
     private DcMotor arm = null;         // Sets the variable of the arm that is long but there is not a arm that is short
     private DcMotor intake = null;          // Sets the variable of the intake
     private DcMotor conveyor = null;          // Sets the variable of the conveyor
     DigitalChannel beambreak;          // Sets the variable of the beamBreak
     Servo claw = null;          // Sets the variable of the Claw
     boolean clawstate = false;          // Sets the variable of the clawstate
     boolean toggle = true;          // Sets the variable of the toggle
     double shooterpower = 1;             // mayhaps
     Servo light = null;
     DigitalChannel limitup;
     DigitalChannel limitdown;

     public void init() {
         lf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lf");       //sets the names of the motors on the hardware map
         rf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rf");
         lb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lb");
         rb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rb");
         arm = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "arm");
         shooter = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "shooter");
         intake = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "intake");
         conveyor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "conveyor");
         beambreak = hardwareMap.get(DigitalChannel.class, "beambreak");
         claw = hardwareMap.get(Servo.class, "claw");
         limitup = hardwareMap.get(DigitalChannel.class, "limitup");
         limitdown = hardwareMap.get(DigitalChannel.class, "limitdown");
         light = hardwareMap.get(Servo.class, "light");

         shooter.setDirection(DcMotorEx.Direction.REVERSE);            //sets the directions of the motors
         lf.setDirection(DcMotorEx.Direction.FORWARD);
         rf.setDirection(DcMotorEx.Direction.REVERSE);
         lb.setDirection(DcMotorEx.Direction.FORWARD);
         rb.setDirection(DcMotorEx.Direction.REVERSE);
         beambreak.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
         claw.setPosition(1);

          final double driveP = 2.5;        //PID values will change, these are filler values
          final double driveI = 0.1;
          final double driveD = 0.2;
          PIDCoefficients drivePID = new PIDCoefficients(driveP, driveI, driveD);
          lf.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePID);
          rf.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePID);
          lb.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePID);
          rb.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePID);


         // RevBlinkinLedDriver blinkinLedDriver = null;
         //GREEN;
         light.setPosition(-0.71);
        // blinkinLedDriver.setPattern(GREEN);

         /*
         display = telemetry.addData("Display Kind: ", displayKind.toString());
         patternName = telemetry.addData("Pattern: ", pattern.toString());

         ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
         gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
          */
         arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up
         shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         arm.setDirection(DcMotor.Direction.FORWARD);
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
         //double armrotation = MOTOR_TICK_COUNT * (0.375);
         // double armrotationmiddle = MOTOR_TICK_COUNT * (0.25);

         telemetry.addData("Is pressed? ", limitup.getState());
         telemetry.addData("Is pressed? ", limitdown.getState());
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
         lb.setPower(leftbackpower);
         rf.setPower(rightfrontpower);
         rb.setPower(rightbackpower);

        //Claw

        if (toggle && gamepad2.y) {  // Only execute once per Button push
            toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            if (clawstate) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                clawstate = false;
                claw.setPosition(1);
            } else {
                clawstate = true;
                claw.setPosition(0);
            }
        } else if (!gamepad2.y) {
            toggle = true; // Button has been released, so this allows a re-press to activate the code above.
        }

         //Long Arm

         if (limitup.getState()) {
             arm.setPower(0);
         }

         if (limitdown.getState()) {
             arm.setPower(0);
         }

         if (gamepad2.dpad_down && !limitdown.getState()) {            //turns the arm that is long but there is not a arm that is short
             arm.setPower(-0.3);        //Sets the power for the Long arm
         }

         if (gamepad2.dpad_up && !limitup.getState()) {      //rotates the arm that is long but there is not a arm that is short
             arm.setPower(0.3);        //Sets the power for the Long arm
         }

         /*if (gamepad2.dpad_left || gamepad2.dpad_right) {      //rotates the arm that is long but there is not a arm that is short
             LongArm.setTargetPosition((int) armrotationmiddle);
             LongArm.setPower(0.3);        //Sets the power for the Long arm
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }

         if (gamepad2.a) {
             LongArm.setTargetPosition(-1440);        //Tell the motor to go to 90 degrees when told to
             LongArm.setPower(0.3);
             LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       }

          */

        //Shooter and Shooter Power

        if (gamepad2.left_stick_y < -0.25) {
            shooterpower += 10;
        }

        if (gamepad2.left_stick_y > 0.25) {
            shooterpower -= 10;
        }

        if (shooterpower < 700) {
            shooterpower = 700;
        }

        if (shooterpower > 2500) {
            shooterpower = 2500;
        }

        if (gamepad2.b) {
            shooter.setVelocity(shooterpower);
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

        if (!beambreak.getState()) {
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
            shooter.setVelocity(700);
            arm.setPower(0);
        }
    }
}
