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
#define NDOF_MODE		0xFC//����� ���������� ��������� ������ � ���� ��������
#define AMG_MODE		0xF7//����� ���� ��������
//register setting
#define OPR_MODE		0x3D//����� ������ ������ ������
#define Page_ID			0x07//����� �������� ��������� page0 or page1
#define UNIT_SEL		0x3B//����� ������� ������
#define ACC_Config		0x08//

//register read measurment data
#define EUL_Pitch_MSB	0x1F//������
#define	EUL_Pitch_LSB	0x1E
#define	EUL_Roll_MSB	0x1D//����
#define	EUL_Roll_LSB	0x1C
#define	EUL_Heading_MSB	0x1B//����
#define	EUL_Heading_LSB	0x1A
#define GYR_DATA_Z_MSB	0x19//
#define GYR_DATA_Z_LSB	0x18//
#define GYR_DATA_Y_MSB	0x17//
#define GYR_DATA_Y_LSB	0x16//
#define GYR_DATA_X_MSB	0x15//
#define GYR_DATA_X_LSB	0x14//
#define ACC_DATA_X_LSB	0x08//
#define TEMP_SOURCE		0x40//����� ��������� �����������
#define Accel_TEMP_SOURCE 0x00//�������� ����������� ������������
#define	TEMP			0x34//�����������
#define UNIT_SEL		0x3B//��������� ������ ���������
#define Temperature_C	~(1<<4)//������� �������	
#define TWIHS_CLK		400000




#endif
