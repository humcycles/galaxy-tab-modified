/* 
 *
 * Sain touch driver
 *
 * Copyright (C) 2009 Sain, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#ifndef SAIN_REG_HEADER
#define SAIN_REG_HEADER

#define	TOUCH_UPGRADE_MODULE	0
#define	TOUCH_ONESHOT_UPGRADE	1 // kmj_df09


#define SAIN_REST_CMD			0x00
#define SAIN_WAKEUP_CMD			0x01

#define SAIN_IDLE_CMD			0x04
#define SAIN_SLEEP_CMD			0x05

#define	SAIN_CLEAR_INT_STATUS_CMD	0x03	//st1001 only
#define	SAIN_CALIBRATE_CMD		0x06
#define	SAIN_SAVE_STATUS_CMD		0x07
#define	SAIN_RECALL_FACTORY_CMD		0x0f

// 0x10~12
#define SAIN_TOUCH_MODE			0x10
#define SAIN_SENSITIVITY  		0x11			//
#define SAIN_FIR_COEFFICIENT		0x12
#define SAIN_CHIP_REVISION		0x13
#define SAIN_EEPROM_INFO		0x14

// 0x20~21
#define SAIN_TOTAL_NUMBER_OF_X		0x20
#define SAIN_TOTAL_NUMBER_OF_Y		0x21

#define	SAIN_X_RESOLUTION		0x28
#define	SAIN_Y_RESOLUTION		0x29

// 0x30~33
#define SAIN_CALIBRATION_REF 		0x30
#define SAIN_CALIBRATION_DEFAULT_N 	0x31
#define SAIN_NUMBER_OF_CALIBRATION 	0x32
#define SAIN_CALIBRATION_ACCURACY	0x33


// 0x50~53
#define SAIN_FINGER_COEF_X_GAIN		0x50
#define SAIN_FINGER_COEF_X_GAIN_EDGE1	0x51
#define SAIN_FINGER_COEF_X_GAIN_EDGE2	0x52
#define SAIN_FINGER_COEF_X_OFFSET	0x53

// 0x58~5B
#define SAIN_FINGER_COEF_Y_GAIN		0x58
#define SAIN_FINGER_COEF_Y_GAIN_EDGE1	0x59
#define SAIN_FINGER_COEF_Y_GAIN_EDGE2	0x5a
#define SAIN_FINGER_COEF_Y_OFFSET	0x5b

// 0x60~63
#define SAIN_STYLUS_COEF_X_GAIN		0x60			
#define SAIN_STYLUS_COEF_X_GAIN_EDGE1	0x61
#define SAIN_STYLUS_COEF_X_GAIN_EDGE2	0x62
#define SAIN_STYLUS_COEF_X_OFFSET	0x63

// 0x68~6B
#define SAIN_STYLUS_COEF_Y_GAIN		0x68			
#define SAIN_STYLUS_COEF_Y_GAIN_EDGE1	0x69
#define SAIN_STYLUS_COEF_Y_GAIN_EDGE2	0x6a
#define SAIN_STYLUS_COEF_Y_OFFSET	0x6b

#define	SAIN_POINT_STATUS_REG		0x80
#define	SAIN_ONE_FINGER_GESTURE_STATUS_REG		0x98	//one finger
#define	SAIN_TWO_FINGER_GESTURE_STATUS_REG		0x99	//two finger
#define	SAIN_ICON_STATUS_REG		0x9a	//icon event - four icon

#define	SAIN_RAWDATA_REG		0x9F	//raw data 320byte

// 0xA0~A5
#define	SAIN_HOLD_THRESHOLD		0xa0
#define	SAIN_REACTION_TIME		0xa1
#define SAIN_PALM_REJECT_THRESHHOLD	0xa2
#define SAIN_NOISE_REJECT_THRESHHOLD	0xa3
#define SAIN_STYLUS_EDGE_COEF		0xa4
#define SAIN_STYLUS_HW_THRESHHOLD 	0xa5


// 0xB0~BF
#define SAIN_HOLD_EVENT_TIME		0xb0 
#define SAIN_LONG_HOLD_EVENT_TIME	0xb1	// about 9 sec
#define SAIN_X_FLICK_DISTANCE		0xb2
#define SAIN_Y_FLICK_DISTANCE		0xb3
#define SAIN_FLICK_TIME_THRESHHOLD	0xb4
#define SAIN_PINCH_X_THRESHHOLD		0xb5
#define SAIN_PINCH_Y_THRESHHOLD		0xb6

#define SAIN_FIRMWARE_VERSION		0xc9

#define	SAIN_ERASE_FLASH		0xc9
#define	SAIN_WRITE_FLASH		0xc8
#define	SAIN_READ_FLASH			0xca


//0xF0
#define	SAIN_INT_ENABLE_FLAG		0xf0
//---------------------------------------------------------------------

// Interrupt & status register flag bit
//-------------------------------------------------
#define	BIT_PT_CNT_CHANGE			0
#define	BIT_DOWN				1
#define	BIT_MOVE				2
#define	BIT_UP					3
#define	BIT_HOLD				4
#define	BIT_LONG_HOLD				5
#define	BIT_ONE_FINGER_GETSTURE			6
#define	BIT_TWO_FINGER_GETSTURE			7
#define	BIT_WEIGHT_CHANGE			8
#define	BIT_PT_NO_CHANGE			9
#define	BIT_REJECT				10
#define	BIT_PT_EXIST				11		// status register only
#define	BIT_ICON1_DOWN				12
#define	BIT_ICON2_DOWN				13
#define	BIT_ICON1_UP				14
#define	BIT_ICON2_UP				15
//-------------------------------------------------
// 4 icon
#define	RESERVED_0					12
#define	RESERVED_1					13
#define	RESERVED_2					14
#define	BIT_ICON_EVENT				15

// 4 icon
#define	BIT_O_ICON0_DOWN				0
#define	BIT_O_ICON1_DOWN				1
#define	BIT_O_ICON2_DOWN				2
#define	BIT_O_ICON3_DOWN				3
#define	BIT_O_ICON4_DOWN				4
#define	BIT_O_ICON5_DOWN				5
#define	BIT_O_ICON6_DOWN				6
#define	BIT_O_ICON7_DOWN				7

#define	BIT_O_ICON0_UP				8
#define	BIT_O_ICON1_UP				9
#define	BIT_O_ICON2_UP				10
#define	BIT_O_ICON3_UP				11
#define	BIT_O_ICON4_UP				12
#define	BIT_O_ICON5_UP				13
#define	BIT_O_ICON6_UP				14
#define	BIT_O_ICON7_UP				15


#define	BT412_ID				0x000a	
#define	SAIN_TS_SERIES				0


#define	sain_bit_set(val,n)		((val) &=~(1<<(n)), (val) |=(1<<(n)))
#define	sain_bit_clr(val,n)		((val) &=~(1<<(n)))
#define	sain_bit_test(val,n)		((val) & (1<<(n)))



#endif //SAIN_REG_HEADER


