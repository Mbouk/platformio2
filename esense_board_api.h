#ifndef ESENSE_BOARD_H
#define ESENSE_BOARD_H

////
//// Address of the esense board (0x44).
//int  esense_board_address=0x44;

void Esense_Board_Send_Command(int command);

/*
void ESENSE::Esense_Board_Get_Data(int command, int *data)
Get a Data from a desired command to the e-sense Board .
*/
void Esense_Board_Get_Data(int command, int *data);
//
// The ESENSE class. Provides an interface to ESENSE Board.
//
//class ESENSE
//{
//	public:
//		
    //    used to program the esense board with the bootloader.*/
    // void Esense_Board_Program(void);
	
    //    Initialize the value of the measured currents */
    void Esense_Board_Init(void);
    
    //    Start a measurement in the esense board. */
    void Esense_Board_Do_Measurement(void);

    //    Stop the emission of the electric field in the esense board.*/
    void Esense_Board_Stop_Emission(void);

    //	Start the emission of the electric field in the esense board. */
    void Esense_Board_Start_Emission(void);

    //Increase the amplitude of the emitting electric field in the esense board. */
    void Esense_Board_Increase_Amplitude(void);

    //    Decrease the amplitude of the emitting electric field in the esense board */
    void Esense_Board_Decrease_Amplitude(void);
    
    //    Get the measure angle computed with the measured current*/
    void Esense_Board_Get_Distance_Lat(int *data);
    
    // Get the measure distance computed with the measured current */
    void Esense_Board_Get_Distance_Front(int *data);
    
    //Get the measure of frequency of the emitting electric field in the esense board.
    
    void Esense_Board_Measure_Frequency(void);
    void Esense_Board_FrequencyMax(void);

  	void Esense_Board_FrequencyM(void);
  
  	void Esense_Board_FrequencyParl(void);
  
  	void Esense_Board_FrequencyF(void);
  	void Esense_Board_FrequencyH(void);
  	void Esense_Board_FrequencyB(void);

    //Increase the frequency of the emitting electric field in the esense board.
    void Esense_Board_Increase_Frequency(void);
    
    //Decrease the frequency of the emitting electric field in the esense board.
    void Esense_Board_Decrease_Frequency(void);
    
    //Get the gain of frequency of the emitting electric field in the esense board.
    void Esense_Board_Gain_Frequency(int *data);
    
    //Get the gain of amplitude of the emitting electric field in the esense board.
    void Esense_Board_Gain_Amplitude(int *data);
    
    //Get GBF status : turn off or turn on the emitting electric field in the esense board.
    void Esense_Board_Gbf_On_Off(int *data);
    
    //Get detection status : 0 nothing or 1 something.
    void Esense_Board_Detection_Or_Not(int *data);
    
    // Start an autoset of amplitude of the emitting electric field in the esense board..
    void Esense_Board_Autoset_Amplitude(void);
    
    void Esense_board_get_I10(int *data);
    void Esense_board_get_I11(int *data);
    void Esense_board_get_I12(int *data);
    void Esense_board_get_I13(int *data);
    void Esense_board_get_I20(int *data);
    void Esense_board_get_I21(int *data);
    void Esense_board_get_I22(int *data);
    void Esense_board_get_I23(int *data);
    void Esense_board_get_I30(int *data);
    void Esense_board_get_I31(int *data);
    void Esense_board_get_I32(int *data);
    void Esense_board_get_I33(int *data);
    void Esense_board_get_I40(int *data);
    void Esense_board_get_I41(int *data);
    void Esense_board_get_I42(int *data);
    void Esense_board_get_I43(int *data);
  
	  void Esense_board_get_V(int *data);
    void Esense_Board_get_W(int *data);
    void Esense_board_get_Vert(int *data);
//}
#endif
