#include "mbed.h"

#define PI 3.141592653589793
#define PID_FREQ 300
#define PID_PERIOD (1.0f / PID_FREQ)
#define IMU_I2C_ADDR 0X68
#define ACC_W 1.0f
#define GYRO_W 1.0f - ACC_W
#define ACC_ERROR  6.0f / 180.f //This is the error value we measured from the accelerometer and is a constant (6 degrees)
#define ANGLE_TOLERANCE 5.0f / 180.0f //5 degrees tolerance for stopping the motor
#define Kp  7.0f
#define Ki 0.0f
#define Kd 0.0f
#define ALPHA 0.02f // integral = (1 - ALPHA) * integral + ALPHA * error
#define PID_THRESH (PID_PERIOD / 3.0f)

enum motor_state
{
    FORWARD, BACKWARD, BRAKE, STOP
};


I2C imu_i2c(p9, p10);
DigitalOut myLed(LED1);
DigitalOut slave_sel(p14);

DigitalOut Ain1(p23);
DigitalOut Ain2(p22);
DigitalOut Bin1(p19);
DigitalOut Bin2(p18);
PwmOut     PWMA(p21);
PwmOut     PWMB(p24);

EventQueue queue;
void updateFromPID(float);
void updateAngle(float, float, float, float);
Event<void(float)> pid(&queue, updateFromPID);
Event<void(float,float,float,float)> updateAngleEvent(&queue, updateAngle);


// main() runs in its own thread in the OS

void setup_pwm(PwmOut PWM, float duty_cycle, int period_micros)

{
    PWM.period_us(period_micros);
    PWM.write(duty_cycle);
}

void drive_motor(motor_state state)
{
    //forward -> IN1 = 0 , IN2 = 1;
    //backward -> IN1 = 1, IN2 = 0;
    //brake -> IN1 = 1, IN2 = 1;
    //stop -> IN1 = 0, IN2 = 0;

        switch(state)
        {
            case FORWARD:  Ain1 = 0; Ain2 =  1; Bin1 = 0; Bin2 = 1; break;
            case BACKWARD: Ain1 = 1; Ain2 = 0; Bin1 = 1; Bin2 = 0; break;
            case BRAKE: Ain1 = 1; Ain2 = 2; Bin1 = 1; Bin2 = 1; break;
            case STOP: Ain1 = 0; Ain2 = 0; Bin1 = 0; Bin2 = 0; break;
        }

   


}

void imu_wakeup()
{
    int power_addr = 0x6;
    char data[2] = { (char)power_addr, 0x01 }; // First: register address, Second: value to write
    int slave_addr = IMU_I2C_ADDR << 1;
    imu_i2c.write(slave_addr, data, 2);
}

int* imu_read(int start_addr) 
{
    char addr = (char) start_addr;

    imu_i2c.write(IMU_I2C_ADDR << 1, &addr, 1, true); // 'true' -> keep bus active (no stop)

    static int data[3];
    char raw_data[6];

    // Now read 6 bytes
    imu_i2c.read(IMU_I2C_ADDR << 1, raw_data, 6);

    int x_h = raw_data[0];
    int x_l = raw_data[1];
    int y_h = raw_data[2];
    int y_l = raw_data[3];
    int z_h = raw_data[4];
    int z_l = raw_data[5];

    data[0] = (x_h << 8 | x_l) << 16 >> 16; 
    data[1] = (y_h << 8 | y_l) << 16 >> 16; 
    data[2] = (z_h << 8 | z_l) << 16 >> 16; 

    return data;
}

float fast_sqrt(float x) {
    float guess = x / 2.0f;
    for (int i = 0; i < 3; i++) {
        guess = (guess + x / guess) / 2.0f;
    }
    return guess;
}

float fast_arccos(float x)
{
    //MAP OF ARCCOSINE VALUES FROM -1 TO 1 from indices [0, 300]
    static const float arccosMap[] = 
    {PI,
3.026058353305556,2.9781113472186513,2.9412578112666736,
2.910136241811168,2.882671111583572,2.857798544381465,
2.8348868812294103,2.8135247715438427,2.7934266323168324,
2.774384633031956,2.756241952455762,2.738876812009132,
2.7221924107679407,2.706110296865632,2.6905658417935308,
2.6755050556109614,2.6608822873372495,2.646658527248898,
2.6328001290794227,2.619277831783745,2.6060659992754056,
2.5931420215729823,2.580485837359395,2.5680795491666966,
2.555907110132642,2.5439540667066542,2.532207345558998,
2.520655075754206,2.5092864393102867,2.498091544796509,
2.487061319773459,2.4761874187533457,2.4654621440291318,
2.454878377240203,2.444429519947561,2.4341094418104503,
2.4239124352091883,2.4138331753608298,2.4038666851365442,
2.3940083039207725,2.384253659958957,2.3745986457279264,
2.365039395934791,2.3555722678095226,2.3461938234056494,
2.336900813664556,2.3276901640333034,2.318558961454817,
2.3095044425737288,2.300523983021863,2.291615087664986,
2.282775381707464,2.274002602564344,2.2652945924214523,
2.2566492914136074,2.2480647313592916,2.2395390299972684,
2.231070385676808,2.222657072458606,2.214297435588181,
2.205989887307672,2.1977329029755763,2.1895250174671474,
2.1813648218309956,2.173250960179887,2.165182126795959,
2.1571570634324884,2.1491745567961003,2.1412334361948187,
2.133332571338752,2.125470870281424,2.117647277490841,
2.1098607720403977,2.1021103659105753,2.0943951023931957,
2.0867140545906953,2.079066324003543,2.0714510391994847,
2.0638673545588304,2.056314449090488,2.048791525313849,
2.041297808202054,2.033832544182489,2.02639500019072,
2.018984462774328,2.0116002372434183,2.0042416468647826,
1.9969080320969432,1.9895987498634984,1.9823131728623846,
1.9750506889088353,1.9678107003099783,1.9605926232691573,
1.953395887318194,1.946219934775936,1.9390642202315367,
1.9319282100510304,1.9248113819058552,1.9177132243220583,
1.9106332362490184,1.9035709266465735,1.8965258140895267,
1.8894974263885678,1.8824853002266948,1.8754889808102941,
1.8685080215340726,1.8615419836590963,1.8545904360032244,
1.8476529546432756,1.8407291226283,1.8338185297033653,
1.8269207720432998,1.8200354519958641,1.8131621778338598,
1.8063005635156963,1.799450228453979,1.792610797291691,
1.7857818996855725,1.7789631700963175,1.7721542475852274,
1.7653547756169798,1.758564401868189,1.7517827780414443,
1.7450095596845363,1.7382444060145859,1.731486979746807,
1.724736946927652,1.7179939767720835,1.7112577415047523,
1.7045279162048381,1.697804178654352,1.6910862091896848,
1.6843736905561992,1.6776663077656817,1.6709637479564563,
1.6642657002559893,1.6575718556458066,1.6508819068285556,
1.6441955480970476,1.6375124752051218,1.630832385240175,
1.6241549764972054,1.6174799483542242,1.6108070011488855,
1.6041358360561988,1.5974661549671767,1.590797660368287,
1.5841300552215667,1.577463042845267,1.5707963267948966,
1.5641296107445262,1.5574625983682264,1.5507949932215062,
1.5441264986226162,1.5374568175335945,1.5307856524409076,
1.524112705235569,1.5174376770925875,1.5107602683496182,
1.5040801783846713,1.4973971054927455,1.4907107467612375,
1.4840207979439866,1.4773269533338036,1.4706289056333368,
1.4639263458241114,1.4572189630335937,1.4505064444001083,
1.443788474935441,1.4370647373849552,1.4303349120850406,
1.4235986768177096,1.4168557066621414,1.4101056738429858,
1.4033482475752073,1.3965830939052568,1.3898098755483488,
1.3830282517216042,1.3762378779728133,1.3694384060045657,
1.3626294834934756,1.3558107539042206,1.3489818562981022,
1.342142425135814,1.3352920900740968,1.3284304757559333,
1.3215572015939288,1.3146718815464933,1.3077741238864278,
1.300863530961493,1.2939396989465175,1.2870022175865687,
1.2800506699306966,1.2730846320557205,1.266103672779499,
1.2591073533630983,1.2520952272012253,1.2450668395002664,
1.2380217269432199,1.2309594173407745,1.2238794292677349,
1.2167812716839381,1.2096644435387625,1.2025284333582564,
1.1953727188138572,1.188196766271599,1.1810000303206358,
1.1737819532798148,1.1665419646809578,1.1592794807274085,
1.1519939037262947,1.14468462149285,1.1373510067250105,
1.129992416346375,1.1226081908154653,1.115197653399073,
1.1077601094073037,1.1002948453877395,1.092801128275944,
1.0852782044993052,1.0777252990309627,1.0701416143903082,
1.0625263295862497,1.054878598999098,1.0471975511965979,
1.0394822876792176,1.0317318815493957,1.0239453760989525,
1.016121783308369,1.0082600822510408,1.0003592173949747,
0.9924180967936925,0.9844355901573045,0.9764105267938343,
0.9683416934099058,0.9602278317587973,0.9520676361226454,
0.9438597506142169,0.9356027662821208,0.9272952180016121,
0.9189355811311872,0.910522267912985,0.9020536235925247,
0.8935279222305016,0.8849433621761856,0.8762980611683404,
0.8675900510254493,0.8588172718823291,0.8499775659248068,
0.8410686705679301,0.8320882110160643,0.8230336921349758,
0.8139024895564898,0.8046918399252371,0.7953988301841433,
0.7860203857802704,0.7765532576550023,0.7669940078618663,
0.757338993630836,0.7475843496690208,0.7377259684532484,
0.7277594782289629,0.7176802183806048,0.7074832117793429,
0.697163133642232,0.6867142763495899,0.6761305095606613,
0.6654052348364471,0.654531333816334,0.6435011087932843,
0.6323062142795062,0.6209375778355871,0.6093853080307949,
0.5976385868831388,0.5856855434571507,0.5735131044230966,
0.5611068162303983,0.5484506320168105,0.5355266543143876,
0.5223148218060486,0.5087925245103698,0.4949341263408953,
0.4807103662525439,0.46608759797883154,0.45102681179626214,
0.435482356724161,0.41940024282185207,0.4027158415806612,
0.38535070113403114,0.3672080205578371,0.34816602127296037,
0.3280678820459501,0.3067057723603831,0.2837941092083272,
0.25892154200622075,0.23145641177862522,0.20033484232311857,
0.16348130637114136,0.11553430028423627,0.0};

    int index = (int) ((x + 1) * 150);
    return arccosMap[index];
}

float get_angle(float x_acc, float y_acc, float z_acc, float z_gyro)
{
    //acc_w and gyro_w are the weighted mean parameters, acc_w + gyro_w must be one
    //angle will be measured in RADIANS
    

    static float gyro_angle_prev = 0;
    
    float z_comp = z_acc < -1.0 ? -1.0 : z_acc > 1.0 ? 1.0 : z_acc; /// fast_sqrt(x_acc * x_acc + y_acc * y_acc + z_acc * z_acc);
    
    float acc_angle = fast_arccos(z_comp);

    //printf("Acc Angle: %d\n", (int) (acc_angle * 100));



    acc_angle = y_acc < 0 ? -acc_angle : acc_angle;

    
    // float gyro_angle = gyro_angle_prev + z_gyro * PI / 180 * PID_PERIOD;
    // gyro_angle_prev = gyro_angle;

    //printf("gyro angle: %d\n", (int) gyro_angle);
    //printf("acc angle: %d\n", (int) acc_angle);


    //return GYRO_W * gyro_angle + ACC_W * acc_angle;



    return acc_angle;

}

void updateAngle(float x_acc, float y_acc, float z_acc, float z_gyro)
{
    float new_angle = get_angle(x_acc, y_acc, z_acc, z_gyro);
    pid.post(new_angle);
}

void updateFromPID(float raw_angle_rad) {

    float norm_angle = raw_angle_rad / PI;

    float error = -norm_angle;



            
    static float prev_error       = 0.0f;
    static float integral         = 0.0f;
    static float duty_cycle_prev       = 0.0f; 
    static float prev_der         = 0.0f;
    float mag = 0;
    float delta_derivative  = (error - prev_error) * PID_FREQ;  
    float delta_e = error * PID_PERIOD;
    delta_derivative = ALPHA * delta_derivative + (1 - ALPHA) * prev_der;
    if (fabs(error) > ANGLE_TOLERANCE)
        integral = (1 - ALPHA) * integral + delta_e;
    float duty_cycle = Kp * error
               + Ki * integral
               + Kd * delta_derivative;

    printf("Increment:%d\n", (int) ((duty_cycle - duty_cycle_prev) * 1000));

    if((duty_cycle - duty_cycle_prev) > PID_THRESH)
        duty_cycle += PID_THRESH;

    else if((duty_cycle_prev - duty_cycle) > PID_THRESH)
        duty_cycle -= PID_THRESH;

    duty_cycle_prev = duty_cycle;
    prev_der = delta_derivative;
    

    if      (duty_cycle >  1.0f) duty_cycle =  1.0f;
    else if (duty_cycle < -1.0f) duty_cycle = -1.0f;


    if (fabs(norm_angle) < ANGLE_TOLERANCE) {
        drive_motor(STOP);
        duty_cycle = 0.0f;
        PWMA.write(0.0f);
        PWMB.write(0.0f);
        integral = 0.0f;
    } else {
        mag = fabs(duty_cycle);
        if (duty_cycle >= 0.0f) {
            drive_motor(FORWARD);
        } else {
            drive_motor(BACKWARD);
        }
        
        PWMA.write(mag);
        PWMB.write(mag);
    }

    //printf("PWM: %d\n", (int) (duty_cycle * 100));
    prev_error      = error;
}


int main()
{

    
    float duty_cycle = 0.45;
    int period_micros = 50; //20kHz
    setup_pwm(PWMA, duty_cycle, period_micros);
    setup_pwm(PWMB, duty_cycle, period_micros);
    motor_state init_state = FORWARD;
    drive_motor(init_state);


    int acc_addr = 45;
    int gyro_addr = 51;

    float ACCEL_SENS = 16384.0; // Sensitivity in LSB / g
    float GYRO_SENS = 131.0; // Sensitivity in LSB / (deg / s) 
    slave_sel.write(0); 

    int* data_read;
    float x_acc,y_acc,z_acc, x_gyro, y_gyro, z_gyro;
    myLed = 1;

    float curr_angle = 0;

    Thread pid_thread;


    Thread event_thread;
    event_thread.start(callback(&queue, &EventQueue::dispatch_forever));
    

    imu_wakeup();

    while (true) {
            data_read = imu_read(acc_addr);
            x_acc = data_read[0] / ACCEL_SENS;
            y_acc = data_read[1] / ACCEL_SENS;
            z_acc = data_read[2] / ACCEL_SENS;
            
            data_read = imu_read(gyro_addr);
            x_gyro = data_read[0] / GYRO_SENS;
            y_gyro = data_read[1] / GYRO_SENS;
            z_gyro = data_read[2] / GYRO_SENS;

            // printf(
            //     "ACC (X,Y,Z): %d, %d, %d\n"
            //     "GYRO (X, Y, Z): %d, %d, %d\n",
            //     (int) x_acc, (int) y_acc, (int) z_acc,
            //     (int) x_gyro, (int) y_gyro, (int) z_gyro
            // );
            // fflush(stdout);

            updateAngleEvent.post(x_acc, y_acc, z_acc, z_gyro);
            ThisThread::sleep_for(chrono::milliseconds((int)(PID_PERIOD * 1000)));


    }
}

 
