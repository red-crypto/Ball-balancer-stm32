#include "pattern.h"
#include "main.h"
extern TIM_HandleTypeDef htim3;

int timeII;
void PID(double,double,double);
void SetMoveTo(double,double,double,int);
void linePattern(double rx, double ry, int wait, int num){
    	for(int i=0;i<num;i++){
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<wait){
    			PID(108,rx,ry);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<wait){
    			PID(108,-10,0);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<wait){
    			PID(108,-rx,-ry);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<wait){
    			PID(108,-10,0);
    		}
    	}
    }
void tripattern(){
		timeII=__HAL_TIM_GET_COUNTER(&htim3);
		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<10000){
			PID(108,-10,0);
		}
    	for(int i=0;i<5;i++){

    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<2000){
    			PID(108,-30,0);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<2000){
    			PID(108,20,50);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<2000){
    			PID(108,20,0);
    		}
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<2000){
    			PID(108,20,-50);
    		}

    	}
    }
void bunnyhop(){
    	for(int i=0;i<10;i++){
    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<1000){
    			PID(108,-10,0);
    		}

    			SetMoveTo(80,0,0,50);//height 80
    			SetMoveTo(108,0,0,50);//height 108

    		timeII=__HAL_TIM_GET_COUNTER(&htim3);
    		while(__HAL_TIM_GET_COUNTER(&htim3)-timeII<1000){
    			PID(108,-10,0);
    		}

    	}
    }
