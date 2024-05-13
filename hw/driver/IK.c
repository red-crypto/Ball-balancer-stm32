/*
 * IK.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */



#include "IK.h"
#include "my_touch.h"
#include "uart.h"


//PID variables lasts kp =0.408, ki = 0.000001, kd =3.2
const double kp =0.408, ki = 0.000001, kd =3.2;   //kp =0.4, ki = 0.000001, kd =3.2; for 256,20us  kp =0.408, ki = 0.000002, kd =2.2; for 256,5us                             //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 };  //PID terms for X and Y directions
float out[2];
static float xx = 0, yy = 0;
int currentPos[3]= { 0 , 0 , 0 }, targetPos[3]= { 0 , 0 , 0 }, direction[3]= { 0 , 0 , 0 },distanceToGo[3] = { 0 , 0 , 0 };
int flag[3]={ 0 , 0 , 0 };
int pos[3]={ 0 , 0 , 0 };
int j=0;
double angToStep = 142.2; //200/360*microsteps , ex)16->8.89 32->17.78 128->71.1 256->142.2
double angOrig = 206.66;

//Accel/Decel variables
int n=0;
float cn=0;
float c0=0;
float stepInterval[3]= {0, 0, 0};
int intStepInterval[3] = {0, 0, 0};
float speed=1000;
float accel=30000;
float stepAngle=0;

//Calculation Variables
float theta;
float ix=0;
float iy=0;
float Psix;
float Psiy;

float Psiz;

float ux;
float uy;
float uz;
float vx;
float vy;
float vz;

float l1;
float l2;
float b;
float p;
float a1;
float a2;
float a3;

float O7x;
float O7y;

float O4x;
float O5x;
float O6x;
float O4y;
float O5y;
float O6y;
float O4z;
float O5z;
float O6z;
float A1;
float A2;
float A3;
float B1;
float B2;
float B3;
float C1;
float C2;
float C3;

float cal_theta(int leg, float O7z, float pax, float pay){
	   ix=pax;
	   iy=pay;
	   float k1,k2=0;
	   k1=-0.2;// psi angle / width 20 / 155
	   k2=0.133;// psi angle / width 20 / 100
	   //rotate the angle
	   float xxx=0.5*(k2*iy-sqrt(3)*k1*ix);
	   float yyy=0.5*(-k1*ix-sqrt(3)*k2*iy);

	   Psix=xxx*3.14/180;
	   Psiy=-yyy*3.14/180;

	   Psiz=atan2(-sin(Psix)*sin(Psiy),(cos(Psix)+cos(Psiy)));

	   ux=cos(Psiy)*cos(Psiz);
	   uy=cos(Psiz)*sin(Psix)*sin(Psiy)+cos(Psix)*sin(Psiz);
	   uz=sin(Psix)*sin(Psiz)-cos(Psix)*cos(Psiz)*sin(Psiy);

	   vx=-cos(Psiy)*sin(Psiz);
	   vy=cos(Psix)*cos(Psiz) - sin(Psix)*sin(Psiy)*sin(Psiz);
	   vz=sin(Psix)*cos(Psiz) + cos(Psix)*sin(Psiy)*sin(Psiz);

	   l1=44.45; // link1
	   l2=93.2; // link2
	   b=50.8; // base plate length
	   p=79.375; // moving plate length

	   a1=0*3.14/180;
	   a2=120*3.14/180;
	   a3=240*3.14/180;

	   O7x=(p*(ux-vy))/2;
	   O7y=-uy*p;

	   O4x=O7x+p*ux;
	   O5x=O7x-p*ux/2+sqrt(3)*p*vx/2;
	   O6x=O7x-p*ux/2-sqrt(3)*p*vx/2;
	   O4y=O7y+p*uy;
	   O5y=O7y-p*uy/2+sqrt(3)*p*vy/2;
	   O6y=O7y-p*uy/2-sqrt(3)*p*vy/2;
	   O4z=O7z+p*uz;
	   O5z=O7z-p*uz/2+sqrt(3)*p*vz/2;
	   O6z=O7z-p*uz/2-sqrt(3)*p*vz/2;

	  //cos(2x)=cos^2x-sinx;
	  //cos^2x=cos2x+1 /2

	 	  	  //theta for another asw of eqs.
//	  double thetam1=2*atan2((-B1-sqrt(pow(A1,2)+pow(B1,2)-pow(C1,2))),(C1-A1))*180/3.14;
//	  double thetam2=2*atan2((-B2-sqrt(pow(A2,2)+pow(B2,2)-pow(C2,2))),(C2-A2))*180/3.14;
//	  double thetam3=2*atan2((-B3-sqrt(pow(A3,2)+pow(B3,2)-pow(C3,2))),(C3-A3))*180/3.14;

	  switch(leg) {
	  case 0:

		   A1=2*l1*cos(a1)*(b*cos(a1)-O4x);
		   B1=2*l1*O4z*(cos(2*a1)+1)/2;
		   C1=pow(O4x,2)-2*b*O4x*cos(a1)+(cos(2*a1)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O4z,2));

		  theta=2*atan2((-B1+sqrt(pow(A1,2)+pow(B1,2)-pow(C1,2))),(C1-A1))*180/3.14;
		  theta=180+theta;
		  break;
	  case 1:
		  A2=2*l1*cos(a2)*(b*cos(a2)-O5x);
		  B2=2*l1*O5z*(cos(2*a2)+1)/2;
		  C2=pow(O5x,2)-2*b*O5x*cos(a2)+(cos(2*a2)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O5z,2));
		  theta=2*atan2((-B2+sqrt(pow(A2,2)+pow(B2,2)-pow(C2,2))),(C2-A2))*180/3.14;
		  theta=180+theta;
		  break;
	  case 2:
		  A3=2*l1*cos(a2)*(b*cos(a2)-O6x);
		  B3=2*l1*O6z*(cos(2*a3)+1)/2;
		  C3=pow(O6x,2)-2*b*O6x*cos(a3)+(cos(2*a3)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O6z,2));
		  theta=2*atan2((-B3+sqrt(pow(A3,2)+pow(B3,2)-pow(C3,2))),(C3-A3))*180/3.14;
		  theta=180+theta;
		  break;

	  }

	  return theta;


  }


  void step(int i, int spd, int mst){

	  switch(i){
	  case 0:
  			if(direction[i]==0){

				currentPos[i]+=1;

				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_DIR_Pin, 0);
				   // Generate a pulse on the STEP pin
				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 1);
				   delay_us(spd); // 200ns setup time
				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 0);


  			}
  			else{

				currentPos[i]-=1;

				HAL_GPIO_WritePin(GPIOB, MOTOR_A_DIR_Pin, 1);
				   // Generate a pulse on the STEP pin
				HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 1);
				delay_us(spd);// 200ns setup time
				HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 0);

  			}
  			break;
	  case 1:
			if(direction[i]==0){

				currentPos[i]+=1;

		  		   HAL_GPIO_WritePin(MOTOR_B_DIR_GPIO_Port, MOTOR_B_DIR_Pin, 0);
		  		   // Generate a pulse on the STEP pin
		  		   HAL_GPIO_WritePin(MOTOR_B_STEP_GPIO_Port, MOTOR_B_STEP_Pin, 1);
		  		   delay_us(spd); // 200ns setup time
		  		   HAL_GPIO_WritePin(MOTOR_B_STEP_GPIO_Port, MOTOR_B_STEP_Pin, 0);


			}
			else{

				currentPos[i]-=1;

	  			   HAL_GPIO_WritePin(MOTOR_B_DIR_GPIO_Port, MOTOR_B_DIR_Pin, 1);
	  			   // Generate a pulse on the STEP pin
	  			   HAL_GPIO_WritePin(MOTOR_B_STEP_GPIO_Port, MOTOR_B_STEP_Pin, 1);
	  			   delay_us(spd); // 200ns setup time
	  			   HAL_GPIO_WritePin(MOTOR_B_STEP_GPIO_Port, MOTOR_B_STEP_Pin, 0);



			}
		  break;
	  case 2:
			if(direction[i]==0){

				currentPos[i]+=1;

		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_DIR_Pin, 0);
		  		   // Generate a pulse on the STEP pin
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 1);
		  		   delay_us(spd); // 200ns setup time
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 0);

			}
			else{

				currentPos[i]-=1;

	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_DIR_Pin, 1);
	  			   // Generate a pulse on the STEP pin
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 1);
	  			   delay_us(spd); // 200ns setup time
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 0);


			}
		  break;
	  }


  }



  void SetMoveTo(double hz, double nx, double ny,int timeOut){
	  float ahz=(float)hz;
	  float anx=(float)nx;
	  float any=(float)ny;

		stepAngle= 0.0001226; //1.8deg -> rad / 256steps
		c0 = 0.676 * sqrt((2.0*stepAngle) / accel) * 1000000; // Equation 15

  		for(int i=0 ; i<3; i++){
  			float pTheta=cal_theta(i, ahz, anx, any);
  			uartPrintf(1,"motor shaft angle: %f\n",pTheta);
  			pos[i]=round((angOrig - pTheta) * angToStep);

  			//moveTo()
  			targetPos[i] = pos[i];
  		}
  		//run();
  			//runSpeed();
  	  int timeInt=mills();//60000->1ms
  			while(mills()-timeInt<timeOut)
  			{
				//computeNewSpeed()
				distanceToGo[j]=targetPos[j]-currentPos[j];

				if(distanceToGo[j] == 0)
				{
					flag[j]=1;

				}
				else if(distanceToGo[j]>0)
				{
					direction[j]=0;
					flag[j]=0;
				}
				else
				{
					direction[j]=1;
					flag[j]=0;
				}

  				if(flag[j]!=1)
  				{
  				//time=micros()
  				//if time-pretime>=stepinterval
				step(j,20,256); // default : 20us (datasheet : 20ns)

				//////////////////////////////////////accel/decl

//				if(n==0)
//				{
//					cn=c0;
//
//				}
//				else
//				{
//					cn=cn-((2*cn)/((4*n)+1));
//				}
//
//				stepInterval[j] = cn; //6ms->3ms->2.5ms->...
//
//				intStepInterval[j] = (int)(stepInterval[j]);
//				n++;
//				delay_us(intStepInterval[j]);

				//////////////////////////////////////////

  				}
  				else
  				{
  					//distance==target
  				}

				j++;
				if(j>2)
				{
					j=0;
				}

  			}
  			//reset flags

  			flag[0]=0;
  			flag[1]=0;
  			flag[2]=0;

  }

    void PID(double height,double setPointX, double setPointY){
  	if(TouchRead(&xx, &yy)){
  		//mapping x,y
      if(yy<250)yy=250;
      if(yy>3700)yy=3700;
      if(xx<600)xx=600;
      if(xx>3500)xx=3500;
      xx=(xx-600)/(3500-600)* 100;
      yy=(yy-250)/(3700-250) * 155;
      uartPrintf(1,"X: %f Y: %f \r\n",xx-50,yy-77.5);
      for (int i = 0; i < 2; i++) {

      	      errorPrev[i] = error[i];

                                                                      //sets previous error
        error[i] = (i==0)?(xx-50 - setPointX):(yy-77.5 - setPointY);
        	  	  	  	  	  //sets error aka X or Y ball position

        integr[i] += (error[i] + errorPrev[i]);                                                        //calculates the integral of the error (proportional but not equal to the true integral of the error)
        deriv[i] = (error[i] - errorPrev[i]);       //calcuates the derivative of the error (proportional but not equal to the true derivative of the error)
        deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                                //checks if the derivative is a real number or infinite
        out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];
        out[i]= (float)out[i];

      }

      SetMoveTo(height, out[0],out[1],10);//height 108

  	}


}



