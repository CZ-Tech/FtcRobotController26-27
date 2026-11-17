package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.network.AutoConfig;
import org.firstinspires.ftc.teamcode.common.vision.ColorRange;

// adb connect 192.168.43.1:5555

public class Globals {
    // ------------------------------------------------------------------
    // Debug / runtime
    // ------------------------------------------------------------------
    public static boolean DEBUG = false;
    public static int PID_THREAD_Hz = 120;

    // ------------------------------------------------------------------
    // Hardware map names
    // ------------------------------------------------------------------

    // 底盘
    public static String LeftFrontMotor = "lfm";  // ch0
    public static String RightFrontMotor = "rfm"; // ch1
    public static String LeftBackMotor = "lbm";   // ch2
    public static String RightBackMotor = "rbm";  // ch3

    // 传感器 / 摄像头
    public static String logiC270 = "Webcam 1";
    public static String ImuName = "imu";
    public static String odoName = "odo"; // ci2c0

    // 机构
    public static String Shooter = "shooter";  // eh 2
    //public static String rightShooter = "rs"; // eh 3
    public static String leftArm = "ila";
    public static String rightArm = "ira";
    public static String intaker = "intaker"; // eh 0
    public static String leftBelt = "lb";
    public static String rightBelt = "rb";
    public static String belt = "belt";       // eh 1
    public static String doorController = "dc";
    public static String doorWheel = "dw";    // servo hub 5
    public static String leftPullerServo = "lps";
    public static String rightPullerServo = "rps";
    public static String valve = "valve";
    // ------------------------------------------------------------------
    // Drivetrain
    // ------------------------------------------------------------------

    public static DcMotor.Direction LeftFrontMotorDirection = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RightFrontMotoDirection = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction LeftBackMotorDirection = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RightBackMotorDirection = DcMotor.Direction.FORWARD;

    public static double TURN_GAIN = 0.0005;
    public static double STEER_GAIN = 0.02;

    // 反注释下方代码进入调试模式，用于测试每个电机方向。
    // Globals.DEBUG = true;
    // 一号手柄按住 share 键或者 back 键测试。
    /**
     * Xbox/PS4 Button - Motor
     * X / ▢         - Left  Front
     * Y / Δ         - Right Front
     * B / O         - Right Back
     * A / X         - Left  Back
     * The buttons are mapped to match the wheels spatially if you
     * were to rotate the gamepad 45deg°. x/square is the front left
     * and each button corresponds to the wheel as you go clockwise
     *                 / ______ \
     * ------------.-'   _  '-..+              Front of Bot
     *           /   _  ( Y )  _  \                  ^
              * |  ( X )  _  ( B ) |      Left Front  \    Right Front
     *     ___  '.      ( A )     /|       Wheel       \       Wheel
     *  .'    '.    '-._____.-'  .'       (x/▢)        \       (Y/Δ)
     * |       |                 |                      \
     *  '.___.' '.               |          Left Back    \       Right Back
     *          '.             /             Wheel       \       Wheel
     *           \.          .'              (A/X)        \       (B/O)
     *            \________/
     * https://rr.brott.dev/docs/v1-0/tuning/
     */

    // ------------------------------------------------------------------
    // Odometry
    // ------------------------------------------------------------------
    /** Whether the global odometry pose has been initialized across OpModes. */
    public static volatile boolean isPoseInitialized = false;

    // 设置里程计吊舱相对于跟踪点的位置偏移
    //  @param xOffset 检测前后运动的吊舱偏移量（毫米），中心左侧为正，右侧为负
    //  @param yOffset 检测左右运动的吊舱偏移量（毫米），中心前方为正，后方为负
    //  +++++++++++++++++++++++++++++++
    //  +                        |||  +
    //  +               xOffset  |||  +
    //  +             <--------->     +
    //  +             ● Center        +
    // FIXME:Center指的是机器的旋转中心
    //  +             |               +
    //  +             | yOffset       +
    //  +             V               +
    //  +           =====             +
    //  +           =====             +
    //  +++++++++++++++++++++++++++++++
    public static double odoXOffset = -13.19;
    public static double odoYOffset = -5.86;  // -95.99872249

    // TODO: 确认自己使用的里程计类型
    public static GoBildaPinpointDriver.GoBildaOdometryPods odoType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // 确认 odo 编码器方向，向前读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoXDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    // 确认 odo 编码器方向，向左读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoYDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ------------------------------------------------------------------
    // DECODE mechanism tuning
    // ------------------------------------------------------------------

    public static double diss_far = 3045;
    public static double diss_near = 2715;

    public static double ShooterP = 30;  // 0.0003 0.0007
    public static double ShooterI = 0.0;
    public static double ShooterD = 0.0; // 0.00005
    public static double ShooterF = 15.0;
    public static double Shoot_time = 2.15;
    public static double SHOOT_VEL_TOLERANCE = 120;
    public static double SHOOT_ACCEL_TOLERANCE = 500;
    public static int FAR_PWM_1 = 3450;
    public static int FAR_PWM_2 = 3200;
    public static double HEAD_SHOOT_ANGLE_DEG = 180;  // 定义的车头和发射之间的夹角
    public static double SHOOT_DIS_OFFSET_MM = -650;
    public static double SHOOTER_K_LINEAR = -0.8;
    public static double SHOOTER_B_LINEAR = SHOOTER_K_LINEAR * -1800;  // 此项中的数字为直线零点，左加右减，x轴为距离(mm)
    public static double THIRD_BALL_COMPENSATION_FACTOR = 1.06;

    // ------------------------------------------------------------------
    // Vision / field model
    // ------------------------------------------------------------------

    public static ColorRange targetColor;
    public static boolean UseVisionLocate = false;

    public static Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, 72, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, -72, AngleUnit.DEGREES, 0);

    @AutoConfig("自动驾驶")
    public static final class Auto {
        private Auto() {}
        // ==================================================================
        // Spline autonomous path following
        //
        // Code values are the defaults. @AutoConfig permits temporary reflection
        // writes while the robot program is running; no override is persisted.
        // Most values are read each control tick. Geometry/segment setup values
        // (e.g. ARC_SAMPLES) take effect on the next movement segment.
        // Units: inches, degrees, seconds, inch/s, inch/s^2.
        // ==================================================================

        /** Verbose per-tick spline log output; diagnostics only. */
        public static volatile boolean LOG_VERBOSE = false;

        /** Spline drivetrain zero-power behavior; FLOAT coasts when motor power is zero. */
        public static volatile DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
                DcMotor.ZeroPowerBehavior.FLOAT;

        // ------------------------------------------------------------------
        // Control
        // ------------------------------------------------------------------

        /** Conservative *measured* effective deceleration in inch/s^2. Must be calibrated. */
        public static volatile double SAFE_BRAKE_DECEL = 30.0;
        /** Negative tangential motor-power magnitude requested by the Bang-Bang brake. */
        public static volatile double BRAKE_REVERSE_POWER = 0.75;
        /** Default positive power ceiling once inside a JSON braking zone. */
        public static volatile double DEFAULT_BRAKE_FORWARD_POWER = 0.30;
        /** One-pole filter time constant, seconds; applies ONLY within a braking zone. */
        public static volatile double BRAKE_FILTER_TAU_S = 0.07;

        /** Optional maxSpeed control; never active unless maxSpeed is in the JSON. */
        public static volatile double SPEED_LIMIT_KP = 0.075;
        public static volatile double SPEED_LIMIT_FULL_POWER_BELOW_IN_S = 6.0;
        public static volatile double SPEED_LIMIT_MAX_REVERSE_POWER = 0;

        // Cross-track PID. Integral defaults to zero; PD is normally sufficient.
        public static volatile double K_CROSS_P = 0.10;
        public static volatile double K_CROSS_I = 0.0;
        public static volatile double K_CROSS_D = 0.010;
        public static volatile double CROSS_I_MAX_POWER = 0.20;
        public static volatile double CROSS_D_ALPHA = 0.25;

        // Heading controller kept intentionally simple.
        public static volatile double K_HEADING = 0.02;
        public static volatile double TURN_DEAD_AREA = 0.0;

        /** Low-pass filter for odometry-derived velocity. 1 = raw, smaller = smoother. */
        public static volatile double VELOCITY_FILTER_ALPHA = 0.35;

        /** Translation command vector magnitude limit before sending to drivetrain. */
        public static volatile double MAX_TRANSLATION_POWER = 1.0;

        // ------------------------------------------------------------------
        // Projection / geometry
        // ------------------------------------------------------------------
        public static volatile int ARC_SAMPLES = 240;
        public static volatile int PROJECTION_SAMPLES = 14;
        public static volatile int NEWTON_ITERS = 4;

        /** Search a little behind the previous projection, in path inches. */
        public static volatile double PROJECTION_BACKTRACK_IN = 3.0;

        /** Search ahead of the previous projection, in path inches. */
        public static volatile double PROJECTION_FORWARD_IN = 18.0;

        /** Used when spline derivative is ~0 at a Hermite endpoint. */
        public static volatile double TANGENT_SAMPLE_DU = 0.01;

        // ------------------------------------------------------------------
        // Completion / safety
        // ------------------------------------------------------------------
        public static volatile double END_U_THRESHOLD = 0.94;
        /** Maximum lateral offset at the finish line for a pass-through handoff. */
        public static volatile double PASS_MAX_CROSS_ERROR_IN = 8.0;
        public static volatile double STOP_POS_THRESHOLD_IN = 1.0;
        public static volatile double STOP_HEADING_THRESHOLD_DEG = 3.0;
        public static volatile double STOP_SPEED_THRESHOLD_IN_S = 3.0;
        public static volatile double STOP_CAPTURE_DISTANCE_IN = 3.0;
        public static volatile double STOP_CAPTURE_ENTRY_SPEED_IN_S = 6.0;
        public static volatile double STOP_CAPTURE_KP = 0.12;
        public static volatile double STOP_CAPTURE_MAX_POWER = 0.25;
        public static volatile double DEGENERATE_ARC_THRESHOLD = 0.05;

        /** Movement failure policy, read each route tick; command failures always abort. */
        public static volatile SplineTrajectoryLoader.FailurePolicy FAILURE_POLICY =
                SplineTrajectoryLoader.FailurePolicy.STOP_PATH;

        /** <= 0 disables stall detection. A stall is NOT treated as arrival. */
        public static volatile double STALL_TIMEOUT_S = 2.0;
        public static volatile double STALL_PROGRESS_EPS_IN = 0.08;
        public static volatile double STALL_END_ERROR_EPS_IN = 0.05;

        // ------------------------------------------------------------------
        // Optional voltage compensation (closed-loop control generally needs little)
        // ------------------------------------------------------------------
        public static volatile double motorVoltage = 12.0;
        public static volatile double VOLTAGE_COMP_WEIGHT = 0.0;
    }
}
