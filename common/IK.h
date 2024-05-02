/*
 * IK.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_COMMON_IK_H_
#define SRC_COMMON_IK_H_



#include "hw_def.h"



float cal_theta(int leg, float O7z, float pax, float pay);
void step(int i, int spd, int mst);
void SetMoveTo(double hz, double nx, double ny,int timeOut);
void PID(double height,double setPointX, double setPointY);



#endif /* SRC_COMMON_IK_H_ */
