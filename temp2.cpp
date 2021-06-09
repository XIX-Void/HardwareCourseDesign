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

struct Stack {
	int count;
	block myBlock[MAXSIZE];


	void push_back(const block object){
        myBlock[count] = object;
        count++;
    }


	int size()const{
	    return count;
    }


    int begin()const{
        return 0;
    }

    block getBlock(int rank){
        return myBlock[rank];
    }   

    void erase(int number){
        int temp = number;
        while(number <= count){
            myBlock[number] = myBlock[number + 1];
            number ++;
        }
        count--;
    }   

};



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

Stack myStack;

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
    Serial.begin(115200);
    Wire.begin();

//iiiiiiiiiiiiiiiiiiiiiiiiiiiiii

    initialize();
    Serial.println(F("initiallize succe"));
    srand((unsigned)time(NULL));
    Serial.println(F("srand succe"));
    FindBlock();

//下面的while还没有经过测试，但是注释掉while之后也仍然会出现setup变成loop的情况...
    
    while(myStack.size()){
        //Serial.println(F("test"));
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

//iiiiiiiiiiiiiiiiiiiiiiii

 
    u8g2.setBusClock(100000);
    u8g2.begin();
    u8g2.setFlipMode(1);
    coordinate ball1;
    ball1.x = 0;
    ball1.y = 0;
    Serial.println(F("finish"));
}


void loop() {
    delay(1000);
        Serial.println(F("test for loop begin")); 
        u8g2.clearBuffer();                   // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
        u8g2.drawDisc(60,30,10,U8G2_DRAW_ALL);
        for(int i = 0; i<= 15; i++){
            for(int j = 0; j<= 31; j++){
                if(maze[i][j] = 1){
                    u8g2.drawPixel(i,j);
                }
            }
        }
        u8g2.sendBuffer();     
        Serial.println(F("test for loop finish")); 
}