#include <Wire.h>
#include <ADS1115_WE.h>

ADS1115_WE ads = ADS1115_WE(0x48);

/*
====================================================
电机PWM引脚
====================================================
*/
int enablePin1 = 10;
int enablePin2 = 9;

/*
====================================================
电机方向引脚
====================================================
*/
int in1Pin = 4;
int in2Pin = 5;

int in3Pin = 3;
int in4Pin = 2;

/*
====================================================
ADS1115阈值
首次运行请观察串口再调整
====================================================
*/
int thresholds[3] =
{
    12700,  // 左: 白~6500 / 黑~18890
    9785,   // 中: 白~1350 / 黑~18220
    11255   // 右: 白~5260 / 黑~17250
};

/*
====================================================
传感器状态
====================================================
*/
int irSensorDigital[3];
int irSensors = B000;

/*
====================================================
基础速度
====================================================
*/
int baseSpeed = 80;

/*
====================================================
PID参数
====================================================
*/
float Kp = 0.45;
float Ki = 0.00;
float Kd = 2.5;

/*
====================================================
PID变量
====================================================
*/
float error = 0;
float lastError = 0;

float integral = 0;
float derivative = 0;

float pid = 0;

/*
====================================================
电机速度
====================================================
*/
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

/*
====================================================
读取ADS1115通道
====================================================
*/
int readADS(byte ch)
{
    switch(ch)
    {
        case 0:
            ads.setCompareChannels(ADS1115_COMP_0_GND);
            break;

        case 1:
            ads.setCompareChannels(ADS1115_COMP_1_GND);
            break;

        case 2:
            ads.setCompareChannels(ADS1115_COMP_2_GND);
            break;
    }

    ads.startSingleMeasurement();

    while(ads.isBusy())
    {
    }

    return ads.getRawResult();
}

/*
====================================================
初始化
====================================================
*/
void setup()
{
    Serial.begin(115200);

    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);

    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);

    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);

    Wire.begin();

    if(!ads.init())
    {
        Serial.println("ADS1115 ERROR");
        while(1);
    }

    ads.setVoltageRange_mV(ADS1115_RANGE_6144);
    ads.setConvRate(ADS1115_860_SPS);

    Serial.println("ADS1115 READY");
}

/*
====================================================
主循环
====================================================
*/
void loop()
{
    Scan();

    CalculateError();

    PID_Control();

    Drive(leftMotorSpeed, rightMotorSpeed);

    delay(2);
}

/*
====================================================
扫描红外
====================================================
*/
void Scan()
{
    irSensors = B000;

    /*
        传感器通道映射:
        ADS A2 ← 原Arduino A5 → 左传感器
        ADS A1 ← 原Arduino A4 → 中间传感器
        ADS A0 ← 原Arduino A3 → 右传感器

        raw数组: 左 中 右
    */
    int rawLeft  = readADS(2);
    int rawMid   = readADS(1);
    int rawRight = readADS(0);

    int raw[3] = {rawLeft, rawMid, rawRight};

    for(int i=0;i<3;i++)
    {
        if(raw[i] >= thresholds[i])
        {
            irSensorDigital[i] = 1;
        }
        else
        {
            irSensorDigital[i] = 0;
        }

        int b = 2 - i;

        irSensors +=
        (
            irSensorDigital[i] << b
        );
    }

    Serial.print("L=");
    Serial.print(rawLeft);

    Serial.print(" M=");
    Serial.print(rawMid);

    Serial.print(" R=");
    Serial.print(rawRight);

    Serial.print(" STATE=B");
    Serial.print(irSensors, BIN);

    Serial.print(" ERR=");
    Serial.print(error);

    Serial.print(" LSPD=");
    Serial.print(leftMotorSpeed);

    Serial.print(" RSPD=");
    Serial.println(rightMotorSpeed);
}

/*
====================================================
状态转误差
====================================================
*/
void CalculateError()
{
    switch(irSensors)
    {
        case B010:
            error = 0;
            break;

        case B110:
            error = -40;
            break;

        case B100:
            error = -80;
            break;

        case B011:
            error = 40;
            break;

        case B001:
            error = 80;
            break;

        case B000:

            if(lastError < 0)
            {
                error = -110;
            }
            else
            {
                error = 110;
            }

            break;

        case B111:
            error = 0;
            break;

        default:
            error = lastError;
    }
}

/*
====================================================
PID
====================================================
*/
void PID_Control()
{
    derivative = error - lastError;

    // ============================================
    // 直角弯检测
    // ============================================
    //
    // 条件1: 只有最外侧传感器看到线 →
    //         B100(仅左) 或 B001(仅右)
    //         这是进入直角弯的最可靠信号
    //
    // 条件2: 完全丢线 B000 且之前在大误差 →
    //         直角弯中途暂时丢线, 按原方向继续转
    //
    bool isSharpTurn =
    (
        irSensors == B100 ||
        irSensors == B001
    );

    bool isLostInTurn =
    (
        irSensors == B000 &&
        abs(lastError) >= 80
    );

    bool useSharpPID =
    (
        isSharpTurn ||
        isLostInTurn
    );

    // ============================================
    // 切换时重置积分, 避免历史积分污染
    // ============================================
    static bool wasSharpLastLoop = false;

    if(useSharpPID != wasSharpLastLoop)
    {
        integral = 0;
    }

    wasSharpLastLoop = useSharpPID;

    // ============================================
    // 积分累加 (只在直角弯模式中启用 Ki)
    // ============================================
    if(useSharpPID)
    {
        integral += error;
    }

    integral =
    constrain
    (
        integral,
        -300,
        300
    );

    // ============================================
    // 双套 PID 参数
    // ============================================
    float currentKp, currentKi, currentKd;
    int   currentBaseSpeed;

    if(useSharpPID)
    {
        // 直角弯专用: 高Kp快速打方向
        //              小Ki维持转弯姿态不丢线
        //              高Kd抑制过冲防摆头
        currentKp  = 0.25;   // 原0.45 → 适中响应，够转且不过冲
        currentKi  = 0.04;   // 原0.00 → 帮助维持转弯
        currentKd  = 10.0;    // 原2.50 → 适度阻尼，兼顾速度与平稳

        currentBaseSpeed = 35;
    }
    else
    {
        // 正常 PID
        currentKp  = Kp;
        currentKi  = Ki;
        currentKd  = Kd;

        // 阶梯降速策略
        if(abs(error) >= 110)       // 完全丢线，最慢
        {
            currentBaseSpeed = 30;
        }
        else if(abs(error) >= 40)   // 一般弯道
        {
            currentBaseSpeed = 55;
        }
        else                        // 直线
        {
            currentBaseSpeed = baseSpeed;  // 70
        }
    }

    pid =
    currentKp  * error +
    currentKi  * integral +
    currentKd  * derivative;

    lastError = error;

    leftMotorSpeed =
    constrain
    (
        currentBaseSpeed + pid,
        0,
        255
    );

    rightMotorSpeed =
    constrain
    (
        currentBaseSpeed - pid,
        0,
        255
    );
}

/*
====================================================
驱动
====================================================
*/
void Drive(int leftSpeed,int rightSpeed)
{
    analogWrite(enablePin1,rightSpeed);

    digitalWrite(in1Pin,HIGH);
    digitalWrite(in2Pin,LOW);

    analogWrite(enablePin2,leftSpeed);

    digitalWrite(in3Pin,LOW);
    digitalWrite(in4Pin,HIGH);
}
