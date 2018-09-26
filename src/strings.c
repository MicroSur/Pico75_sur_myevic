#include <stdint.h>

#define __string__ const uint8_t __attribute__ ((section (".strings")))

//__string__ String_ON 		[] = { 0xAA, 0xA9, 0 }; // "ON"
__string__ String_No 		[] = { 0xA9, 0x90, 0 }; // "No"
__string__ String_On 		[] = { 0xBB, 0 }; //{ 0xAA, 0x8F, 0 }; // "On"
__string__ String_Key 		[] = { 0xA6, 0x86, 0x9A, 0 }; // "Key"
__string__ String_OFF 		[] = { 0xAA, 0xA1, 0xA1, 0 }; // "OFF"
__string__ String_OffMod	[] = { 0x76, 0x87, 0x87, 0x74, 0x90, 0x85, 0 }; // OffMod
__string__ String_Sleep    	[] = { 0xAE, 0x8D, 0x86, 0x86, 0x91, 0 };
__string__ String_Mode    	[] = { 0xA8, 0x90, 0x85, 0x86, 0 };
__string__ String_Low 		[] = { 0xA7, 0x90, 0x98, 0 }; // "Low"
__string__ String_Off 		[] = { 0xBA, 0 }; //[] = { 0xAA, 0x87, 0x87, 0 }; // "Off"
__string__ String_Lock 		[] = { 0xA7, 0x90, 0x84, 0x8C, 0 }; // "Lock"
__string__ String_UnLock 	[] = { 0xB0, 0x8F, 0xA7, 0x90, 0x84, 0x8C, 0 }; // "UnLock"
__string__ String_Protection    [] = { 0xAB, 0x93, 0x90, 0x95, 0x86, 0x84, 0x95, 0x8A, 0x90, 0x8F, 0	}; // "Protection"
__string__ String_Version 	[] = { 0xB1, 0x86, 0x93, 0x94, 0x8A, 0x90, 0x8F, 0 }; // "Version"
__string__ String_Device 	[] = { 0x9F, 0x86, 0x97, 0x8A, 0x84, 0x86, 0 }; // "Device"
__string__ String_TooHot 	[] = { 0xA3, 0x90, 0x95, 0 }; // "Hot"
__string__ String_Stealth 	[] = { 0xAE, 0x95, 0x86, 0x82, 0x8D, 0x95, 0x89, 0 }; // "Stealth"
__string__ String_Temp 		[] = { 0xAF, 0x86, 0x8E, 0x91, 0 }; // "Temp"
__string__ String_Battery 	[] = { 0x9D, 0x82, 0x95, 0x95, 0x86, 0x93, 0x9A, 0 }; // "Battery"
__string__ String_Atomizer	[] = { 0x9C, 0x95, 0x90, 0x8E, 0x8A, 0x9B, 0x86, 0x93, 0 }; // "Atomizer"
//__string__ String_Found 	[] = { 0xA1, 0x90, 0x96, 0x8F, 0x85, 0 }; // "Found"
__string__ String_Short 	[] = { 0xAE, 0x89, 0x90, 0x93, 0x95, 0 }; // "Short"
__string__ String_LongFire 	[] = { 0xA7, 0x90, 0x8F, 0x88, 0xBC, 0xA1, 0x8A, 0x93, 0x86, 0 };
__string__ String_NewCoil 	[] = { 0xA9, 0x86, 0x98, 0 }; // "New"
__string__ String_SameCoil	[] = { 0xAA, 0x8D, 0x85, 0 }; // Old
//__string__ String_Right 	[] = { 0xAD, 0x8A, 0x88, 0x89, 0x95, 0	}; // "Right"
//__string__ String_Left 		[] = { 0xA7, 0x86, 0x87, 0x95, 0 }; // "Left"
__string__ String_Logo 		[] = { 0xA7, 0x90, 0x88, 0x90, 0 }; // "Logo"
__string__ String_Game 		[] = { 0xA1, 0x8D, 0x82, 0x91, 0x91, 0x9A, 0 }; // "Game" Flappy
__string__ String_Tetris 	[] = { 0xAF, 0x86, 0x95, 0x93, 0x8A, 0x94, 0 }; // "Tetris" 
__string__ String_Easy 		[] = { 0xA0, 0x82, 0x94, 0x9A, 0 }; // "Easy"
__string__ String_Normal 	[] = { 0xA9, 0x90, 0x93, 0x8E, 0x82, 0x8D, 0 }; // "Normal"
__string__ String_Hard 		[] = { 0xA3, 0x82, 0x93, 0x85, 0 }; // "Hard"
__string__ String_Exit 		[] = { 0xA0, 0x99, 0x8A, 0x95, 0 }; // "Exit"
//__string__ String_Back 		[] = { 0x9D, 0x82, 0x84, 0x8C, 0 }; // "Back"
__string__ String_Back 		[] = { 0xCF, 0 }; // "< -"
//__string__ String_BF_s 		[] = { 0x69, 0x6D, 0	}; // "BF" (small)
__string__ String_TCR 		[] = { 0xAF, 0x9E, 0xAD, 0 }; // "TCR"
__string__ String_PWR_s 	[] = { 0xE4, 0 }; //[] = { 0x77, 0x7E, 0x79, 0 }; // "PWR" (small)
__string__ String_AMP_s 	[] = { 0xE5, 0 }; //[] = { 0x68, 0x74, 0x77, 0 }; // "AMP" (small)
__string__ String_MAX_s 	[] = { 0xA8, 0x9C, 0xB3, 0 }; //big now //{ 0x74, 0x68, 0x7F, 0 }; // "MAX" (small)
//__string__ String_MIN_s 	[] = { 0x74, 0x70, 0x75, 0 }; // "MIN" (small)
__string__ String_Weak 		[] = { 0xB2, 0x86, 0x82, 0x8C, 0 }; // "Weak"
__string__ String_TEMP 		[] = { 0xAF, 0xA0, 0xA8, 0xAB, 0 }; // "TEMP"
//__string__ String_TCRSet 	[] = { 0xAF, 0x9E, 0xAD, 0xBC, 0xAE, 0x86, 0x95, 0 }; // "TCR Set"
__string__ String_POWER 	[] = { 0xAB, 0xBD, 0xAA, 0xBD, 0xB2, 0xBD, 0xA0, 0xBD, 0xAD, 0 }; // "P O W E R"
__string__ String_BYPASS 	[] = { 0xBD, 0xBD, 0x9D, 0xB4, 0xAB, 0x9C, 0xAE, 0xAE, 0xBD, 0 }; // "  BYPASS "
__string__ String_VOLT_s 	[] = { 0xE6, 0 }; //[] = { 0x7D, 0x76, 0x73, 0x7B, 0 }; // "VOLT" (small)
__string__ String_COIL_s 	[] = { 0xE9, 0 }; //[] = { 0x6A, 0x76, 0x70, 0x73, 0 }; // "COIL" (small)
__string__ String_TIME_s 	[] = { 0xE7, 0 }; //[] = { 0x7B, 0x70, 0x74, 0x6C, 0 }; // "TIME" (small)
__string__ String_PUFF_s 	[] = { 0xE8, 0 }; //[] = { 0x77, 0x7C, 0x6D, 0x6D, 0 }; // "PUFF" (small)
__string__ String_BATT_s	[] = { 0xEA, 0 }; //[] = { 0x69, 0x68, 0x7B, 0x7B, 0 };
__string__ String_VOUT_s	[] = { 0xE6, 0 }; //[] = { 0x7D, 0x76, 0x7C, 0x7B, 0 };
__string__ String_TEMP_s	[] = { 0xF4, 0 }; //[] = { 0x7B, 0x6C, 0x74, 0x77, 0 };
//__string__ String_BOARD_s	[] = { 0xF2, 0 }; //[] = { 0x69, 0x76, 0x68, 0x79, 0x6B, 0 };
__string__ String_RES_s		[] = { 0xEB, 0 }; //[] = { 0x79, 0x6C, 0x7A, 0 };
__string__ String_LIQ_s		[] = { 0xF3, 0 }; //[] = { 0x73, 0x70, 0x78, 0 };
__string__ String_SMART 	[] = { 0xAE, 0xA8, 0x9C, 0xAD, 0xAF, 0 }; //used in menu too
__string__ String_Contrast	[] = { 0x9E, 0x90, 0x8F, 0x95, 0x93, 0x82, 0x94, 0x95, 0 };
//__string__ String_LongFireTo	[] = { 0xA7, 0x90, 0x8F, 0x88, 0xBC, 0xA1, 0x8A, 0x93, 0x86, 0 };
__string__ String_Edit		[] = { 0xA0, 0x85, 0x8A, 0x95, 0 };
__string__ String_ClkSpeed	[] = { 0xAE, 0x91, 0x86, 0x86, 0x85, 0 };
__string__ String_Menus		[] = { 0xA8, 0x86, 0x8F, 0x96, 0x94, 0 };
__string__ String_Modes		[] = { 0xA8, 0x90, 0x85, 0x86, 0x94, 0 };
//__string__ String_TEMP_NI_s	[] = { 0x7B, 0x6C, 0x74, 0x77, 0xBD, 0x75, 0x70, 0x00 };
//__string__ String_TEMP_TI_s	[] = { 0x7B, 0x6C, 0x74, 0x77, 0xBD, 0x7B, 0x70, 0x00 };
//__string__ String_TEMP_SS_s	[] = { 0x7B, 0x6C, 0x74, 0x77, 0xBD, 0x7A, 0x7A, 0x00 };
//__string__ String_TCR_s		[] = { 0x7B, 0x6A, 0x79, 0x00 };
//__string__ String_POWER_s	[] = { 0x77, 0x76, 0x7E, 0x6C, 0x79, 0x00 };
//__string__ String_BYPASS_s	[] = { 0x69, 0x80, 0x77, 0x68, 0x7A, 0x7A, 0x00 };
//__string__ String_SMART_s	[] = { 0x7A, 0x74, 0x68, 0x79, 0x7B, 0x00 };
__string__ String_Coils		[] = { 0x9E, 0x90, 0x8A, 0x8D, 0x94, 0 };
__string__ String_Zero_All	[] = { 0xB5, 0x86, 0x93, 0x90, 0xBC, 0x9C, 0x8D, 0x8D, 0 };
__string__ String_Miscs		[] = { 0xA8, 0x8A, 0x94, 0x84, 0x94, 0 };
//__string__ String_DateTime	[] = { 0x9F, 0x82, 0x95, 0x86, 0xBD, 0xD6, 0xBD, 0xAF, 0x8A, 0x8E, 0x86, 0 };
//__string__ String_Cancel	[] = { 0x9E, 0x82, 0x8F, 0x84, 0x86, 0x8D, 0 };
__string__ String_Save		[] = { 0xAE, 0x82, 0x97, 0x86, 0 };
__string__ String_Clock		[] = { 0x9E, 0x8D, 0x90, 0x84, 0x8C, 0 };
__string__ String_3D		[] = { 0x0E, 0x9F, 0 };
__string__ String_Box		[] = { 0x9D, 0x90, 0x99, 0 };//{ 0x9E, 0x96, 0x83, 0x86, 0 };cube
__string__ String_Qix		[] = { 0xAC, 0x8A, 0x99, 0 };
__string__ String_None		[] = { 0xA9, 0x90, 0x8F, 0x86, 0 };
__string__ String_ClkAdjust	[] = { 0x9C, 0x85, 0x8B, 0x96, 0x94, 0x95, 0 };
__string__ String_Screen	[] = { 0xAE, 0x84, 0x93, 0x86, 0x86, 0x8F, 0 };
__string__ String_Min		[] = { 0xA8, 0x8A, 0x8F, 0 };
__string__ String_Expert	[] = { 0xA0, 0x99, 0x91, 0x86, 0x93, 0x95, 0 };
__string__ String_USB		[] = { 0xDA, 0 }; //[] = { 0xB0, 0xAE, 0x9D, 0 };
__string__ String_HID		[] = { 0xA3, 0xA4, 0x9F, 0 };
//__string__ String_COM		[] = { 0x9E, 0xAA, 0xA8, 0 };
__string__ String_UCH_s		[] = { 0xF5, 0 };
//__string__ String_DBG		[] = { 0x9F, 0x9D, 0xA2, 0 };
__string__ String_X32		[] = { 0xCA, 0 }; //[] = { 0xB3, 0x0E, 0x0D,0 };
//__string__ String_PCT		[] = { 0xAB, 0x9E, 0xAF,0 };
__string__ String_Saver		[] = { 0xAE, 0x82, 0x97, 0x86, 0x93, 0 };
__string__ String_Preheat 	[] = { 0xAB, 0x93, 0x86, 0x89, 0x86, 0x82, 0x95, 0 };
__string__ String_Time		[] = { 0xAF, 0x8A, 0x8E, 0x86, 0 };
__string__ String_Pwr		[] = { 0xAB, 0x98, 0x93, 0 };
__string__ String_Manage	[] = { 0xA8, 0x82, 0x8F, 0x82, 0x88, 0x86, 0 };
__string__ String_Unit		[] = { 0xB0, 0x8F, 0x8A, 0x95,0 };
__string__ String_Main		[] = { 0xA8, 0x82, 0x8A, 0x8F,0 };
__string__ String_Interface	[] = { 0xA4, 0x8F, 0x95, 0x86, 0x93, 0x87, 0x82, 0x84, 0x86 ,0 };
__string__ String_FiFlip	[] = { 0xA1, 0x8A, 0xA1, 0x8D, 0x8A, 0x91, 0 };
__string__ String_VVLite	[] = { 0xB1, 0xB1, 0xA7, 0x8A, 0x95, 0x86, 0 };
__string__ String_1Watt		[] = { 0x0C, 0xB2, 0x82, 0x95, 0x95 ,0 };
__string__ String_1C5F		[] = { 0x0C, 0x9E, 0x10, 0xA1, 0 };
__string__ String_Hide		[] = { 0xA3, 0x8A, 0x85, 0x86, 0 };
//__string__ String_Date		[] = { 0x9F, 0x82, 0x95, 0x86, 0 };
__string__ String_Set		[] = { 0xAE, 0x86, 0x95, 0 };
//__string__ String_SetTime	[] = { 0xAE, 0x86, 0x95, 0xBD, 0xAF, 0x8A, 0x8E, 0x86, 0 };
//__string__ String_SetDate	[] = { 0xAE, 0x86, 0x95, 0xBD, 0x9F, 0x82, 0x95, 0x86, 0 };
__string__ String_WakeMP	[] = { 0xB2, 0x82, 0x8C, 0x86, 0xBD, 0xD5, 0xD4, 0 };
__string__ String_2			[] = { 0x0D, 0 };
__string__ String_3			[] = { 0x0E, 0 };
__string__ String_4			[] = { 0x0F, 0 };
__string__ String_5			[] = { 0x10, 0 };
__string__ String_OnOff		[] = { 0xAA, 0x8F, 0xD6, 0xBD, 0xAA, 0x87, 0x87, 0 };
//__string__ String_ModePlus	[] = { 0xA8, 0x90, 0x85, 0x86, 0xBD, 0xD4, 0xD4, 0 };
__string__ String_Clicks	[] = { 0x9E, 0x8D, 0x8A, 0x84, 0x8C, 0x94, 0 };
__string__ String_PPwr		[] = { 0xAB, 0xAB, 0x98, 0x93, 0 };
//__string__ String_BAT		[] = { 0x9D, 0x9C, 0xAF, 0 }; //BATT_s
__string__ String_GEN		[] = { 0xA2, 0xA0, 0xA9, 0 };
//__string__ String_25R		[] = { 0x0D, 0x10, 0xAD, 0 };
//__string__ String_HG2		[] = { 0xA3, 0xA2, 0x0D, 0 };
//__string__ String_LGH		[] = { 0xA7, 0xA2, 0xA3, 0 };
//__string__ String_HE4		[] = { 0xA3, 0xA0, 0x0F, 0 };
//__string__ String_30Q		[] = { 0x0E, 0x0B, 0xAC, 0 };
//__string__ String_VT4		[] = { 0xB1, 0xAF, 0x0F, 0 };
//__string__ String_VT5		[] = { 0xB1, 0xAF, 0x10, 0 };
//__string__ String_VT6		[] = { 0xB1, 0xAF, 0x11, 0 };
//__string__ String_SVT		[] = { 0xAE, 0xB1, 0xAF, 0 };
__string__ String_CUS		[] = { 0x9E, 0xB0, 0xAE, 0 };
__string__ String_Vaping	[] = { 0xB1, 0x82, 0x91, 0x8A, 0x8F, 0x88, 0 };
__string__ String_Prot		[] = { 0xAB, 0x93, 0x90, 0x95, 0xC1, 0 };
__string__ String_Build		[] = { 0x9D, 0x96, 0x8A, 0x8D, 0x85, 0 };
//__string__ String_myevic	[] = { 0x8E, 0x9A, 0x86, 0x97, 0x8A, 0x84, 0 };
__string__ String_Snow		[] = { 0xAE, 0x8F, 0x90, 0x98, 0 };
__string__ String_Fmt		[] = { 0xA1, 0x8E, 0x95, 0 };
__string__ String_DMY1		[] = { 0x9F, 0xC1, 0xA8, 0xC1, 0xB4, 0 };
__string__ String_MDY		[] = { 0xA8, 0xD6, 0x9F, 0xD6, 0xB4, 0 };
__string__ String_DMY2		[] = { 0x9F, 0xD6, 0xA8, 0xD6, 0xB4, 0 };
__string__ String_YMD		[] = { 0xB4, 0xD9, 0xA8, 0xD9, 0x9F, 0 };
__string__ String_Dial		[] = { 0x9F, 0x8A, 0x82, 0x8D, 0 };
__string__ String_Invert	[] = { 0xA4, 0x8F, 0x97, 0x86, 0x93, 0x95, 0 };
//__string__ String_mld		[] = { 0xF6, 0 }; //[] = { 0x8E, 0x8D, 0xD6, 0x85, 0 };
//__string__ String_ml		[] = { 0xF5, 0 }; //[] = { 0x8E, 0x8D, 0 };
__string__ String_SHR		[] = { 0xAE, 0xA3, 0xAD, 0 };
__string__ String_Tetra		[] = { 0xAF, 0x86, 0x95, 0x93, 0x82, 0 };
__string__ String_Vaped		[] = { 0xB1, 0x82, 0x91, 0x86, 0x85, 0 };
//__string__ String_BVO		[] = { 0x9D, 0xB1, 0xAA, 0 };
__string__ String_mlkJ		[] = { 0x8E, 0x8D, 0xD6, 0x8C, 0xA5, 0 };
__string__ String_Check		[] = { 0x9E, 0x89, 0x86, 0x84, 0x8C, 0 };
//__string__ String_Adapter	[] = { 0x9C, 0x85, 0x82, 0x91, 0x95, 0x86, 0x93, 0 };
__string__ String_Charge	[] = { 0x9E, 0x89, 0x82, 0x93, 0x88, 0x86, 0 };
__string__ String_Error		[] = { 0xA0, 0x93, 0x93, 0x90, 0x93, 0 };
__string__ String_Imbalanced    [] = { 0xA4, 0x8E, 0x83, 0x82, 0x8D, 0x82, 0x8F, 0x84, 0x86, 0x85, 0 };
__string__ String_Batteries	[] = { 0x9D, 0x82, 0x95, 0x95, 0x86, 0x93, 0x8A, 0x86, 0x94, 0 };
__string__ String_LSL		[] = { 0xA7, 0xAE, 0xA7, 0 };
//__string__ String_Show		[] = { 0xAE, 0x89, 0x90, 0x98, 0 };
//__string__ String_Where		[] = { 0xB2, 0x89, 0x86, 0x93, 0x86, 0 };
__string__ String_Top		[] = { 0xAF, 0x90, 0x91, 0 };
__string__ String_Mid		[] = { 0xA8, 0x8A, 0x85, 0 };
__string__ String_Size		[] = { 0xAE, 0x8A, 0x9B, 0x86, 0 };
__string__ String_hms		[] = { 0x89, 0x8E, 0x94, 0 };
__string__ String_HM		[] = { 0xA3, 0xA8, 0 };
//__string__ String_BALANCE_s	[] = { 0x69, 0x68, 0x73, 0x68, 0x75, 0x6A, 0x6C, 0 };
//__string__ String_BAL_s		[] = { 0x69, 0x68, 0x73, 0xC1, 0 };
__string__ String_Octa		[] = { 0xAA, 0x84, 0x95, 0x82, 0 };
__string__ String_Dodeca	[] = { 0x9F, 0x90, 0x85, 0x86, 0x84, 0 };
__string__ String_Isoca		[] = { 0xA4, 0x84, 0x90, 0x94 , 0 }; //Icos
__string__ String_Square	[] = { 0xAE, 0x92, 0x96, 0x82, 0x93, 0x86, 0 };
__string__ String_TIE   	[] = { 0xAF, 0xA4, 0xA0, 0 };
__string__ String_Quartz   	[] = { 0xAC, 0x96, 0x82, 0x93, 0x95, 0x9B, 0 };
__string__ String_Spinner   	[] = { 0xAE, 0x91, 0x8A, 0x8F, 0 }; //0x8F, 0x86, 0x93, 0 };
__string__ String_B1		[] = { 0x9D, 0x0C, 0 };
__string__ String_B2		[] = { 0x9D, 0x0D, 0 };
__string__ String_B3		[] = { 0x9D, 0x0E, 0 };
__string__ String_B4		[] = { 0x9D, 0x0F, 0 };
__string__ String_M1		[] = { 0xA8, 0x0C, 0 };
__string__ String_M2		[] = { 0xA8, 0x0D, 0 };
__string__ String_M3		[] = { 0xA8, 0x0E, 0 };
__string__ String_DEF		[] = { 0xAF, 0xA1, 0xAD, 0 }; // TFR // { 0x9F, 0xA0, 0xA1, 0 };
//__string__ String_UCH		[] = { 0xB0, 0x9E, 0xA3, 0 }; //UCH_s
__string__ String_Algo		[] = { 0x9C, 0x8D, 0x88, 0x90, 0 };
//__string__ String_Auto		[] = { 0x9C, 0x96, 0x95, 0x90, 0 };
__string__ String_Sweet		[] = { 0xAE, 0x98, 0x86, 0x86, 0x95, 0 };
__string__ String_Boost		[] = { 0x9D, 0x90, 0x90, 0x94, 0x95, 0 };
__string__ String_PID		[] = { 0xAB, 0xA4, 0x9F, 0 };
__string__ String_P			[] = { 0xAB, 0 };
__string__ String_I			[] = { 0xA4, 0 };
__string__ String_D			[] = { 0x9F, 0 };
//__string__ String_Yes		[] = { 0xB4, 0x86, 0x94, 0 };
__string__ String_Led		[] = { 0xA7, 0x86, 0x85, 0 };
__string__ String_Red		[] = { 0xAD, 0 };
__string__ String_Green		[] = { 0xA2, 0 };
__string__ String_Blue		[] = { 0x9D, 0xD6, 0xB4,0 }; // B/Y
__string__ String_Delay		[] = { 0x9F, 0x86, 0x8D, 0x82, 0x9A, 0 };
__string__ String_Profile	[] = { 0xAB, 0x93, 0x90, 0x87, 0x8A, 0x8D, 0x86, 0 };
__string__ String_NI 		[] = { 0xA9, 0xA4, 0 }; // "NI"
__string__ String_TI 		[] = { 0xAF, 0xA4, 0 }; // "TI"
__string__ String_SS 		[] = { 0xAE, 0xAE, 0 }; // "SS"
__string__ String_TC 		[] = { 0xAF, 0x9E, 0 };
__string__ String_PW 		[] = { 0xAB, 0xB2, 0 };
__string__ String_BY 		[] = { 0x9D, 0xB4, 0 };
__string__ String_SM 		[] = { 0xAE, 0xA8, 0 };
//__string__ String_ProfPlus	[] = { 0xAB, 0x93, 0x90, 0x87, 0xC1, 0xBD, 0xD4, 0xD4, 0 };
__string__ String_Curve		[] = { 0x9E, 0x96, 0x93, 0x97, 0x86, 0 };
__string__ String_Enable	[] = { 0xA0, 0x8F, 0x82, 0x83, 0x8D, 0x86, 0 };
__string__ String_Reset		[] = { 0xAD, 0x86, 0x94, 0x86, 0x95, 0 };
__string__ String_Splash	[] = { 0xAE, 0x91, 0x8D, 0x82, 0x94, 0x89, 0 };   

__string__ String_Sunday	[] = { 0xAE, 0xB0, 0xA9, 0 }; //{ 0xAE, 0x7C, 0x75, 0x6B, 0x68, 0x80, 0 };
__string__ String_Monday	[] = { 0xA8, 0xAA, 0xA9, 0 }; //{ 0xA8, 0x76, 0x75, 0x6B, 0x68, 0x80, 0 };
__string__ String_Tuesday	[] = { 0xAF, 0xB0, 0xA0, 0 }; //{ 0xAF, 0x7C, 0x6C, 0x7A, 0x6B, 0x68, 0x80, 0 };
__string__ String_Wednesday	[] = { 0xB2, 0xA0, 0x9F, 0 }; //{ 0xB2, 0x6C, 0x6B, 0x75, 0x6C, 0x7A, 0x6B, 0x68, 0x80, 0 };
__string__ String_Thursday	[] = { 0xAF, 0xA3, 0xB0, 0 }; //{ 0xAF, 0x6F, 0x7C, 0x79, 0x7A, 0x6B, 0x68, 0x80, 0 };
__string__ String_Friday	[] = { 0xA1, 0xAD, 0xA4, 0 }; //{ 0xA1, 0x79, 0x70, 0x6B, 0x68, 0x80, 0 };
__string__ String_Saturday	[] = { 0xAE, 0x9C, 0xAF, 0 }; //{ 0xAE, 0x68, 0x7B, 0x7C, 0x79, 0x6B, 0x68, 0x80, 0 };

__string__ String_Survival	[] = { 0xAE, 0x96, 0x93, 0x97, 0x8A, 0x97, 0x82, 0x8D, 0 };
__string__ String_PuffsOff	[] = { 0xAB, 0x96, 0x87, 0x87, 0xAA, 0x87, 0x87, 0 }; //PuffOff
__string__ String_VapeTimeOff	[] = { 0xAF, 0x8E, 0x93, 0xAA, 0x87, 0x87, 0 }; // TmrOff //{ 0xAF, 0x8A, 0x8E, 0xAA, 0x87, 0x87, 0 }; //TimOff
__string__ String_HoldFi	[] = { 0xA3, 0x90, 0x8D, 0x85, 0xA1, 0x8A, 0 };
__string__ String_FireScrDur	[] = { 0xA1, 0x8A, 0x93, 0x86, 0xAE, 0x84, 0x93, 0 }; 
__string__ String_Percent	[] = { 0xC2, 0 }; 
__string__ String_V             [] = { 0xBE, 0 }; 
//__string__ String_Flash         [] = { 0xA1, 0x8D, 0x82, 0x94, 0x89, 0 }; ->Save
__string__ String_Ok            [] = { 0xAA, 0x8C, 0 };
__string__ String_SwapMP        [] = { 0xAE, 0x98, 0x82, 0x91, 0xD5, 0xD4, 0 };
__string__ String_SME           [] = { 0xAE, 0xA8, 0xA0, 0 };
__string__ String_Cold          [] = { 0x9E, 0x90, 0x8D, 0x85, 0 };
__string__ String_New           [] = { 0xA9, 0x86, 0x98, 0 };
__string__ String_AutoFi        [] = { 0x9C, 0x96, 0x95, 0x90, 0xA1, 0x8A, 0 };
__string__ String_ATime         [] = { 0x9C, 0xAF, 0x8A, 0x8E, 0x86, 0 };
__string__ String_NewZC         [] = { 0x9D, 0x82, 0x95, 0x95, 0xB5, 0x9E, 0 }; //BatZC
__string__ String_ZeroCnts      [] = { 0xB5, 0x86, 0x93, 0x90, 0x9E, 0x8F, 0x95, 0x94, 0 };
__string__ String_Repeat        [] = { 0xAD, 0x86, 0x91, 0x86, 0x82, 0x95, 0 };
__string__ String_UI            [] = { 0xB0, 0xA4, 0 };
__string__ String_FMP           [] = { 0xA1, 0xD5, 0xD4, 0 }; //F<>
__string__ String_All           [] = { 0x9C, 0x8D, 0x8D, 0 };
//__string__ String_Puff24        [] = { 0xEE, 0 };
__string__ String_Puff24        [] = { 0xAB, 0x96, 0x87, 0x87, 0xB5, 0x9F, 0 }; //PuffZD
__string__ String_ResetCoil     [] = { 0xD0, 0x9E, 0x90, 0x8A, 0x8D, 0 }; //reset coil
