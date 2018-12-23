/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
//
#include "esense_board_api.h"
#include "Wire.h"
//#include "driver/i2c.h"

//
// Address of the esense board (0x44).
#define  esense_board_address  0x44


/*
Send a general command  to the e-sense Board.
*/


void Esense_Board_Send_Command(int command)
{
	Wire.beginTransmission(esense_board_address); // transmit to device esense_board_address
	Wire.write(0x01); 
	Wire.write(command); 
	Wire.write(0x17); // sends
	Wire.endTransmission(); // stop transmitting
}


/*
void Esense_Board_Get_Data(int command, int *data)
Get a Data from a desired command to the e-sense Board .
*/
void Esense_Board_Get_Data(int command, int *data)
{
    int i=0, buffer[3] = {0x00, 0x00, 0x00};

	Esense_Board_Send_Command(command);
    //
	//delay(200);

	Wire.requestFrom(esense_board_address, 3);    // request 3 bytes from slave device #2
 
  while(Wire.available())    // slave may send less than requested
  { 
    buffer[i] = Wire.read();    // receive a byte as character
    i++;
  }
  *data = buffer[1];
}


/*
void Esense_Board_Program(void)
used to program the esense board with the bootloader.
*/
/*void Esense_Board_Program(void)
{
    Esense_Board_Send_Command(0x62);
}*/



/*
void Esense_Board_Init(void)
Initialize the value of the measured currents
*/
void Esense_Board_Init(void)
{
    Esense_Board_Send_Command(0x01);
}


/*
void Esense_Board_Do_Measurement(void)
Start a measurement in the esense board.
*/
void Esense_Board_Do_Measurement(void)
{
    Esense_Board_Send_Command(0x02);
}



/*
void Esense_Board_Stop_Emission(void)
Stop the emission of the electric field in the esense board.
*/
void Esense_Board_Stop_Emission(void)
{
    Esense_Board_Send_Command(0x03);
}
///////////////////////////////////////////////////

///////////////////////////////////////////////////

/*
void Esense_Board_Stop_Emission(void)
Start the emission of the electric field in the esense board.
*/
void Esense_Board_Start_Emission(void)
{
    Esense_Board_Send_Command(0x04);
}


/*
void Esense_Board_Increase_Amplitude(void)
Increase the amplitude of the emitting electric field in the esense board.
*/
void Esense_Board_Increase_Amplitude(void)
{
    Esense_Board_Send_Command(0x05);
}


/*
void Esense_Board_Decrease_Amplitude(void)
Decrease the amplitude of the emitting electric field in the esense board.
*/
void Esense_Board_Decrease_Amplitude(void)
{
    Esense_Board_Send_Command(0x06);
}


/*
void Esense_Board_Get_Angle(int *data)
Get the measure angle computed with the measured current
*/
void Esense_Board_Get_Distance_Lat(int *data)
{
    Esense_Board_Get_Data(0x07, data);
}

/*
void Esense_Board_Get_Distance(int *data)
Get the measure distance computed with the measured current
*/
void Esense_Board_Get_Distance_Front(int *data)
{
    Esense_Board_Get_Data(0x08, data);
}
//////////////////////////////////////////////////////
void Esense_Board_Measure_Frequency(void)
{
     Esense_Board_Send_Command(0x09);
   
}
void Esense_Board_FrequencyMax(void)
{
     Esense_Board_Send_Command(0x91);
   
}
void Esense_Board_FrequencyM(void)
{
     Esense_Board_Send_Command(0x92);
   
}
void Esense_Board_FrequencyParl(void)
{
     Esense_Board_Send_Command(0x93);
   
}
void Esense_Board_FrequencyF(void)
{
     Esense_Board_Send_Command(0x94);
   
}
void Esense_Board_FrequencyH(void)
{
     Esense_Board_Send_Command(0x95);
   
}
void Esense_Board_FrequencyB(void)
{
     Esense_Board_Send_Command(0x96);
   
}

void Esense_Board_Gain_Frequency(int *data)
{
    Esense_Board_Get_Data(0x0A, data);
}
void Esense_Board_Gain_Amplitude(int *data)
{
    Esense_Board_Get_Data(0x0B, data);
}
void Esense_Board_Gbf_On_Off(int *data)
{
    Esense_Board_Get_Data(0x0C, data);
}
void Esense_Board_Detection_Or_Not(int *data)
{
    Esense_Board_Get_Data(0x0D, data);
}
void Esense_Board_Autoset_Amplitude(void)
{
    Esense_Board_Send_Command(0x0E);
}
void Esense_Board_Increase_Frequency(void)
{
    Esense_Board_Send_Command(0x0F);
}
void Esense_Board_Decrease_Frequency(void)
{
    Esense_Board_Send_Command(0x15);
}
//
void Esense_board_get_I10(int *data)
{
    Esense_Board_Get_Data(0x10, data);
}
void Esense_board_get_I11(int *data)
{
    Esense_Board_Get_Data(0x11, data);
}
void Esense_board_get_I12(int *data)
{
    Esense_Board_Get_Data(0x12, data);
}
void Esense_board_get_I13(int *data)
{
    Esense_Board_Get_Data(0x13, data);
}

void Esense_board_get_I20(int *data)
{
    Esense_Board_Get_Data(0x20, data);
}
void Esense_board_get_I21(int *data)
{
    Esense_Board_Get_Data(0x21, data);
}
void Esense_board_get_I22(int *data)
{
    Esense_Board_Get_Data(0x22, data);
}
void Esense_board_get_I23(int *data)
{
    Esense_Board_Get_Data(0x23, data);
}

void Esense_board_get_I30(int *data)
{
    Esense_Board_Get_Data(0x30, data);
}
void Esense_board_get_I31(int *data)
{
    Esense_Board_Get_Data(0x31, data);
}
void Esense_board_get_I32(int *data)
{
    Esense_Board_Get_Data(0x32, data);
}
void Esense_board_get_I33(int *data)
{
    Esense_Board_Get_Data(0x33, data);
}

void Esense_board_get_I40(int *data)
{
    Esense_Board_Get_Data(0x40, data);
}
void Esense_board_get_I41(int *data)
{
    Esense_Board_Get_Data(0x41, data);
}
void Esense_board_get_I42(int *data)
{
    Esense_Board_Get_Data(0x42, data);
}
void Esense_board_get_I43(int *data)
{
    Esense_Board_Get_Data(0x43, data);
}
void Esense_board_get_V(int *data)
{
    Esense_Board_Get_Data(0x50, data);
}
void Esense_Board_get_W(int *data)
{
    Esense_Board_Get_Data(0x51, data);
}
void Esense_board_get_Vert(int *data)
{
    Esense_Board_Get_Data(0x52, data);
}

/* [] END OF FILE */
