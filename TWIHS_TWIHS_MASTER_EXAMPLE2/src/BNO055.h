/*
 * BNO055.h
 *
 * Created: 29.03.2017 9:52:28
 *  Author: Kopranov_IN
 */ 
#ifndef BNO055_H_
#define BNO055_H_

#include "asf.h"

#define ADDRESS_BNO055	0x29	//
#define NDOF_MODE		0xFC//режим встроенной обработки данных с трех сенсоров
#define AMG_MODE		0xF7//режим трех сенсоров
//register setting
#define OPR_MODE		0x3D//выбор режима работы модуля
#define Page_ID			0x07//выбор страницы регистров page0 or page1
#define UNIT_SEL		0x3B//выбор формата данных
#define ACC_Config		0x08//

//register read measurment data
#define EUL_Pitch_MSB	0x1F//тангаж
#define	EUL_Pitch_LSB	0x1E
#define	EUL_Roll_MSB	0x1D//крен
#define	EUL_Roll_LSB	0x1C
#define	EUL_Heading_MSB	0x1B//курс
#define	EUL_Heading_LSB	0x1A
#define GYR_DATA_Z_MSB	0x19//
#define GYR_DATA_Z_LSB	0x18//
#define GYR_DATA_Y_MSB	0x17//
#define GYR_DATA_Y_LSB	0x16//
#define GYR_DATA_X_MSB	0x15//
#define GYR_DATA_X_LSB	0x14//
#define ACC_DATA_X_LSB	0x08//
#define TEMP_SOURCE		0x40//выбор источника температуры
#define Accel_TEMP_SOURCE 0x00//источник температуры акселерометр
#define	TEMP			0x34//температура
#define UNIT_SEL		0x3B//настройка единиц измерения
#define Temperature_C	~(1<<4)//градусы Цельсия	
#define TWIHS_CLK		400000




#endif
