/*
====================================================
舵机固定角度 + 超声波测距（优化版2）
功能：
1. 串口输入角度
2. 舵机转到指定角度
3. 自动测量距离
====================================================
*/

#include <Servo.h>

Servo myServo;

/*
====================================================
超声波引脚
====================================================
*/
int trigPin = 11;
int echoPin = 12;

/*
====================================================
舵机引脚
====================================================
*/
int servoPin = 6;

/*
====================================================
变量
====================================================
*/
long duration;
float cm;
int angle = 90;   // 默认初始角度

void setup() {
    Serial.begin(9600);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    myServo.attach(servoPin);

    // 初始化舵机位置
    myServo.write(angle);

    Serial.println("=================================");
    Serial.println(" Servo Angle Control Start ");
    Serial.println("=================================");
    Serial.println("Input angle (0~180):");
}

void loop() {
    
    // 检测串口输入
    if(Serial.available() > 0) {
        angle = Serial.parseInt();
        angle = constrain(angle, 0, 180); // 限制角度范围
        myServo.write(angle);             // 舵机转动
        delay(500);                       // 给舵机转动时间

        cm = getDistance();               // 测距

        // 输出结果
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.println("°");

        Serial.print("Distance: ");
        if(cm < 0) {
            Serial.println("Out of range"); // 超时未检测到回波
        } else {
            Serial.print(cm, 1);
            Serial.println(" cm");
        }
        Serial.println("----------------------");
    }
}

/*
====================================================
超声波测距函数（使用声音速度 0.034 cm/µs，超时38ms）
====================================================
*/
float getDistance() {
    // 发射超声波
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);          // 确保Trig为低电平
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);         // 触发至少10微秒
    digitalWrite(trigPin, LOW);

    // 接收回波，设置最大等待时间38ms
    duration = pulseIn(echoPin, HIGH, 38000);  // 超时38ms = 38000 µs

    if(duration == 0) {
        // 超时未检测到回波
        return -1; // 返回-1表示无障碍物
    }

    // 根据公式计算距离
    // 声速 0.034 cm/µs，除以2得到单程距离
    float distance = (duration * 0.034) * 0.5;

    return distance;
}
