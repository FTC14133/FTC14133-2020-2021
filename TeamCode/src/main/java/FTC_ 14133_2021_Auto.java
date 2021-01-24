// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList;

@Autonomous(name="FTC 14133 2021 Auto", group="Auto")
class FTC_14133_2021_Auto extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftback = null;        // Sets the variables of the mecanum wheels
    private DcMotor rightback = null;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    static final double MOTOR_TICK_COUNT = 2800;
    static final double Drive_Tick_Count = 560;
    private DcMotor Shooter = null;         // Sets the variable of the shooter
    private DcMotor LongArm = null;         // Sets the variable of the arm that is long but there is not a arm that is short
    private DcMotor intake = null;          // Sets the variable of the intake
    private DcMotor conveyor = null;          // Sets the variable of the conveyor
    DigitalChannel LimitSwitchLongArm;          // Sets the variable of the LimitSwitchLongArm
    DigitalChannel beamBreak;          // Sets the variable of the beamBreak
    Servo Claw = null;          // Sets the variable of the Claw
    Servo Stopper = null;          // Sets the variable of the stopper
    boolean clawstate = false;          // Sets the variable of the clawstate
    boolean toggle = true;          // Sets the variable of the toggle
    //double distance = 0;
  //  double turn = 0;


    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");       //sets the names of the motors on the hardware map
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        LongArm = hardwareMap.get(DcMotor.class, "LongArm");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        LimitSwitchLongArm = hardwareMap.get(DigitalChannel.class, "LimitSwitchLongArm");
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Stopper = hardwareMap.get(Servo.class, "Stopper");


        Shooter.setDirection(DcMotor.Direction.FORWARD);            //sets the directions of the motors
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        LimitSwitchLongArm.setMode(DigitalChannel.Mode.INPUT);
        beamBreak.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        Claw.setPosition(0);
        Stopper.setPosition(0);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

    }

    void ForwardorBackwards(double distance, double speed) {
        //Driving forward/backwards
      //  double distance= 5; //(in)
        double encodercounts= distance*(1/(75*(1/25.4)))*560;
        int encodercountsint= (int) encodercounts;
        leftfront.setTargetPosition(encodercountsint);
        leftfront.setPower(speed);        //Sets the power for the Long arm
        rightfront.setTargetPosition( encodercountsint);
        rightfront.setPower(speed);        //Sets the power for the Long arm
        leftback.setTargetPosition( encodercountsint);
        leftback.setPower(speed);        //Sets the power for the Long arm
        rightback.setTargetPosition( encodercountsint);
        rightback.setPower(speed);        //Sets the power for the Long arm
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void Rotate(double turn, double speed) {
        //Driving left/right
        //NOT DONE
        double encodercounts= turn*(1/(75*(1/25.4)))*560*1.4142135623730950488016887242097;
        int encodercountsint= (int) encodercounts;
        leftfront.setTargetPosition(encodercountsint);
        leftfront.setPower(speed);        //
        rightfront.setTargetPosition(-encodercountsint);
        rightfront.setPower(speed);        //Sets the power for the Long arm
        leftback.setTargetPosition(encodercountsint);
        leftback.setPower(speed);        //Sets the power for the Long arm
        rightback.setTargetPosition(-encodercountsint);
        rightback.setPower(speed);        //Sets the power for the Long arm
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void Strafing(double Straf, double speed) {
        //Driving left/right
        //Positive is Strafing left negative is Strafing right
        double encodercounts= Straf*(1/(75*(1/25.4)))*560*1.4142135623730950488016887242097;
        int encodercountsint= (int) encodercounts;
        leftfront.setTargetPosition(encodercountsint);
        leftfront.setPower(speed);        //
        rightfront.setTargetPosition(-encodercountsint);
        rightfront.setPower(speed);        //Sets the power for the Long arm
        leftback.setTargetPosition(encodercountsint);
        leftback.setPower(speed);        //Sets the power for the Long arm
        rightback.setTargetPosition(-encodercountsint);
        rightback.setPower(speed);        //Sets the power for the Long arm
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void IntakeFunction(double speed){
        intake.setPower(speed);
        if (beamBreak.getState()) {
            conveyor.setPower(speed);
        }else{
            conveyor.setPower(0);
        }
    }

    void ShooterFunction(double speed){
        Shooter.setPower(speed);
    }

    void ClawFunctionOpen(){
        double armrotation = MOTOR_TICK_COUNT * (90 / 360);
        LongArm.setTargetPosition((int) armrotation);        //Tell the motor to go to 90 degrees when told to
        LongArm.setPower(1);        //Sets the power for the Long arm
        LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void ClawFunctionClose(){
        LongArm.setTargetPosition(0);        //Tell the motor to go to 90 degrees when told to
        LongArm.setPower(1);        //Sets the power for the Long arm
        LongArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runOpMode() {
        //Do the FUNCTION of LEFT OR RIGHT
      //  turn = 5;
        Strafing(5, 0.5);
     //   turn = 0;
        //Do the FUNCTION of FORWARD OR BACKWARDS
     //   distance = 5;
        ForwardorBackwards(5, 0.5);
      //  distance = 0;
        IntakeFunction(0.5);
    }
}

/*
        //Driving forward/backwards
        double distance= 5; //(in)
        double encodercounts= distance*(1/(75*(1/25.4)))*560;
        int encodercountsint= (int) encodercounts;
        leftfront.setTargetPosition(encodercountsint);
        leftfront.setPower(1);        //Sets the power for the Long arm
        rightfront.setTargetPosition( encodercountsint);
        rightfront.setPower(1);        //Sets the power for the Long arm
        leftback.setTargetPosition( encodercountsint);
        leftback.setPower(1);        //Sets the power for the Long arm
        rightback.setTargetPosition( encodercountsint);
        rightback.setPower(1);        //Sets the power for the Long arm
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Driving left/right
        distance= 5; //(in)
        encodercounts= distance*(1/(75*(1/25.4)))*560*1.4142135623730950488016887242097;
        encodercountsint= (int) encodercounts;
        leftfront.setTargetPosition(encodercountsint);
        leftfront.setPower(1);        //
        rightfront.setTargetPosition(-encodercountsint);
        rightfront.setPower(1);        //Sets the power for the Long arm
        leftback.setTargetPosition(encodercountsint);
        leftback.setPower(1);        //Sets the power for the Long arm
        rightback.setTargetPosition(-encodercountsint);
        rightback.setPower(1);        //Sets the power for the Long arm
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

 */