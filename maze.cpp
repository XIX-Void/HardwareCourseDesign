#include<stdio.h>
#include<stdlib.h>
#include<time.h>
using namespace std;

#define walls 1
#define nothing 0

#define down 1
#define right 2
#define left 4
#define up 8

#define rows 8
#define cols 16
#define MAXSIZE 8192
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
}

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

int main(){
    initialize();
    printf("\n");
    srand((unsigned)time(NULL));
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
    
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            if(i == start.x + 1 && j == start.y + 1){
                printf("%s","\xa7\xb0");
            }else if(maze[i][j] == nothing){
                printf(" ");
            }else if(maze[i][j] == walls){
                printf("%s","\xa8\x80");
            }
        }
        printf("\n");
    }

    int maze_fin[rows][cols];
    for(int i = 1; i<rows; i++){
        for(int j = 1; j<cols; j++){
            maze_fin[i][j] = maze[i-1][j-1];
        }
    }


    return 0;
}
