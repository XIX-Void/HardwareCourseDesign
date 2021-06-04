#include "I2Cdev.h"
#include<U8g2lib.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include<stdio.h>
#include<stdlib.h>
#include "Wire.h"
#include<time.h>

using namespace std;

#define walls 1
#define nothing 0

#define down 1
#define right 2
#define left 4
#define up 8

#define rows 16
#define cols 32
#define MAXSIZE 512

//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

MPU6050 mpu;
float x,y,z;

//made for ball's coordinate
struct coordinate
{
    float x,y;
};

struct block{
	int row,column,direction;
    block(){
        row = 0;
        column = 0;
        direction = down;
    }
	block(int _row,int _column,int _direction){
		row = _row;
		column = _column;
		direction = _direction;
	}
};

struct point {
	int x;
	int y;
}start;

class Stack {
private:
	int count;
	block myBlock[MAXSIZE];

public:
	Stack(){};
	void push_back(const block& object);
	int pop();
	const int& getTop()const;
	int size()const;
	bool isEmpty()const;
    int begin()const;
    block getBlock(int rank);
    void erase(int number);
};

void Stack::push_back(const block& object) {
    myBlock[count] = object;
    count++;
}

int Stack::size()const {
	return count;
}

block Stack::getBlock(int rank){
    return myBlock[rank];
}

int Stack::begin()const{
    return 0;
}

void Stack::erase(int number){
    int temp = number;
    while(number <= count){
        myBlock[number] = myBlock[number + 1];
        number ++;
    }
    count--;
}


#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 6  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 4 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
int temp_y = 0;
int temp_z = 0;

Stack myStack = Stack();

int maze[rows][cols];
int x_num=1,y_num=1;

void initialize(){
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            maze[i][j] = walls;
            //printf("%d",maze[i][j]);
        }
        //printf("\n");
    }
    maze[x_num][y_num] = nothing;
}

void FindBlock() {
	//找出与当前位置相邻的墙
	if(x_num+1<=rows && maze[x_num+1][y_num] == walls) {//down
		myStack.push_back(block(x_num+1,y_num,down));
	}
	if(y_num+1<=cols && maze[x_num][y_num+1] == walls) {//right
		myStack.push_back(block(x_num,y_num+1,right));
	}
	if(x_num-1>=1 && maze[x_num-1][y_num] == walls) {//up
		myStack.push_back(block(x_num-1,y_num,up));
	}
	if(y_num-1>=1 && maze[x_num][y_num-1] == walls) {//left
		myStack.push_back(block(x_num,y_num-1,left));
	}
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//iiiiiiiiiiiiiiiiiiiiiiiiiiiiii
    initialize();
    srand((unsigned)time(NULL));
    FindBlock();
    /*
    while(myStack.size()){
        int blockSize = myStack.size();
        int randnum = rand() % blockSize;
        block selectBlock = myStack.getBlock(randnum);
        x_num = selectBlock.row;
        y_num = selectBlock.column;
        switch (selectBlock.direction){
        case down:{
            x_num++;
            break;
            }
        case right:{
            y_num++;
            break;
            }
        case left:{
            y_num--;
            break;
            }
        case up:{
            x_num--;
            break;
            }
        }
        if(maze[x_num][y_num] == walls){
            maze[selectBlock.row][selectBlock.column] = maze[x_num][y_num] = nothing;
            FindBlock();
        }else{

        }
        myStack.erase(myStack.begin() + randnum);
    }
    */
//iiiiiiiiiiiiiiiiiiiiiiii






    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nS_end any character to begin DMP programming and demo: "));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);





    u8g2.setBusClock(100000);
    u8g2.begin();
    u8g2.setFlipMode(1);




    coordinate ball1;
    ball1.x = 0;
    ball1.y = 0;
}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            x = ypr[0] * 180/M_PI;
            y = ypr[1] * 180/M_PI;
            z = ypr[2] * 180/M_PI;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(y);
            Serial.print("\t");
            Serial.println(z);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        u8g2.clearBuffer();                   // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
        if(-5 < y && y < 5){
            temp_y = temp_y;
        }else if(y <= -5){
            temp_y = temp_y + y;
        }else{
            temp_y = temp_y - y;
        }
        if(-5 < z && z < 5){
            temp_z = temp_z;
        }else if(z <= -5){
            temp_z = temp_z - z;
        }else{
            temp_z = temp_z + z;
        }
        u8g2.drawDisc(60 + temp_y,30 + temp_z,10,U8G2_DRAW_ALL);
            for(int i = 0; i<= 128; i++){
        for(int j = 0; j<= 64; j++){
            if(maze[i][j] = 1){
                u8g2.drawPixel(i,j);
            }
        }
    }
        u8g2.sendBuffer();                    // transfer internal memory to the display
    }
}