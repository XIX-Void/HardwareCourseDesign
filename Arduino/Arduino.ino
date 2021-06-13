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

#define rows 31
#define cols 15
#define MAXSIZE 256

#define XNUM 1
#define YNUM 1

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1309_128X64_NONAME2_2_4W_SW_SPI u8g2(U8G2_R0,SCL,SDA,10,9);

MPU6050 mpu;

//made for ball's coordinate
struct coordinate
{
    float y = 6.0;
    float z = 6.0;
}ball,axis;

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


class Stack {
private:
	int count;
	block myBlock[MAXSIZE];

public:
	Stack(){};
	void push_back(const block object);
	int pop();
	const int& getTop()const;
	int size()const;
	bool isEmpty()const;
    int begin()const;
    block getBlock(int rank);
    void erase(int number);
};

void Stack::push_back(const block object) {
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

Stack myStack = Stack();

boolean button_1,button_2,button_x1,button_x2 = false;
boolean universe_reset = true;
int maze[rows][cols];
int game_time;
int x_num=XNUM,y_num=YNUM;
int x = 0;

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
	if(x_num+2<rows && maze[x_num+1][y_num] == walls) {//down
		myStack.push_back(block(x_num+1,y_num,down));
	}
	if(y_num+2<cols && maze[x_num][y_num+1] == walls) {//right
		myStack.push_back(block(x_num,y_num+1,right));
	}
	if(x_num-2>=0 && maze[x_num-1][y_num] == walls) {//up
		myStack.push_back(block(x_num-1,y_num,up));
	}
	if(y_num-2>=0 && maze[x_num][y_num-1] == walls) {//left
		myStack.push_back(block(x_num,y_num-1,left));
	}
}
void(* resetFunc) (void) = 0;


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
/*
//maze generation
    initialize();
    srand(analogRead(9) * analogRead(10));
    //srand((unsigned)time(NULL));
    FindBlock();
    
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
//maze generation
*/

    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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


    u8g2.setBusClock(800000);
    u8g2.begin();
    //u8g2.setFlipMode(1);


}


void loop() {
    // first thing is generating maze
    //maze generation
    if(universe_reset){
        initialize();
        srand(analogRead(9) * analogRead(10));
        //srand((unsigned)time(NULL));
        FindBlock();
        
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
        universe_reset = false; //universe generate completed,set false
    }
//maze generation


    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            x      = ypr[0] * 180/M_PI;
            axis.y = ypr[1] * 180/M_PI;
            axis.z = ypr[2] * 180/M_PI;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.println(x);

        u8g2.clearBuffer();                   // clear the internal memory
        //u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font


        if(ball.y >= 4 * (rows - 2) && ball.z >= 4 * (cols - 2)){
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(0,28,"Game clear!");
            u8g2.setCursor(0,52);
            u8g2.print(game_time);
            u8g2.drawStr(0,52,"      Second!");
            u8g2.sendBuffer();
            if(axis.z >= 45){
                button_1 = true;
                Serial.println(F("button1"));
            }
            if(axis.z <= -45){
                button_2 = true;
                Serial.println(F("button2"));
            }
            if(button_1 && button_2){
                Serial.println(F("ok"));
                //resetFunc();
                ball.y = ball.z = 6.0;
                universe_reset = true;
                button_1 = false;
                button_2 = false;
                u8g2.clearBuffer();
                u8g2.drawStr(10,28,"Wait 3s!");
                u8g2.sendBuffer();
                delay(3000);
            }
        }else{
            //ball moving
            if(axis.y < -0.8){ //go left
                if(maze[(int)ball.y/4 - 1][(int)ball.z/4] == walls){
                    Serial.println("judgement 1 --- ");
                }else{
                    ball.y = ball.y + axis.y / 4;
                }
            }else if(axis.y > 0.8){ //go right
                if(maze[(int)ball.y/4 + 1][(int)ball.z/4] == walls){
                    Serial.println("judgement 2 --- ");
                }else{
                    ball.y = ball.y + axis.y / 4;
                }
            }
            if(axis.z < -0.8){ //go down
                if(maze[(int)ball.y/4][(int)ball.z/4 + 1] == walls){
                    Serial.println("judgement 3 --- ");
                }else{
                    ball.z = ball.z - axis.z / 4;
                }
            }else if(axis.z > 0.8){ //go up
                if(maze[(int)ball.y/4][(int)ball.z/4 - 1] == walls){
                    Serial.println("judgement 4 --- ");
                }else{
                    ball.z = ball.z - axis.z / 4;
                }
            }

            /*
            Serial.print(ball.y);
            Serial.print("   ");
            Serial.print((int)ball.y/4);
            Serial.print("   ");
            Serial.print("   ");
            Serial.print(ball.z);
            Serial.print("   ");
            Serial.println((int)ball.z/4);
            */

            u8g2.drawBox(ball.y,ball.z, 3 ,3);




            //maze drawing
            for(int i = 0; i< rows * 4; i++){
                for(int j = 0; j< cols * 4; j++){
                    if(maze[i / 4][j / 4] == walls){
                        u8g2.drawPixel(i,j);
                    }
                }
            }
            u8g2.drawCircle(4 * XNUM + 2 , 4 * YNUM + 2 ,2,U8G2_DRAW_ALL);//start point
            u8g2.drawCircle(4 * (rows - 2) + 2 , 4 * (cols - 2) + 2, 2 , U8G2_DRAW_ALL);//end point
            u8g2.sendBuffer();                    // transfer internal memory to the display
            game_time = millis() / 1000;
        }
    }
}