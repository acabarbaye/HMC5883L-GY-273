/*
 * types.h
 *
 *  Created on: 20 Sep 2017
 *      Author: adrcab01
 */

#ifndef PROJECT_TYPES_H_
#define PROJECT_TYPES_H_


// Utility
typedef bool rtStatus_t;
typedef int int_t;
#ifndef SUCCEEDED
#define SUCCEEDED  1
#endif
#ifndef FAILED
#define FAILED  0
#endif
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795f
#endif
#ifndef M_2PI
#define M_2PI  6.283185307179586476925286766559f
#endif
#ifndef I2C_ACKNOWLEDGEMENT
#define I2C_ACKNOWLEDGEMENT 0
#endif
#ifndef CONVERT_RAD_TO_DEG
#define CONVERT_RAD_TO_DEG(angle)  (angle * 57.295779513082320876798154814105f)
#endif
#ifndef CONVERT_DEG_TO_RAD
#define CONVERT_DEG_TO_RAD(angle)  (angle * 0.01745329251994329576923690768489f)
#endif

#ifndef CHECK_SUCCESS
#define CHECK_SUCCESS(method_result,successful_result) if(method_result != successful_result){ return FAILED;}
#endif


#endif /* HMC5883L_TYPES_H_ */
