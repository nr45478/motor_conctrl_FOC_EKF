/**********************************
   
**********************************/
#include "main.h"
#include "adc.h"
#include "UpperComputer.h"

#define SEND_BUFFER_SIZE 20  // 16ؖޚ˽ߝ + 4ؖޚβ˽ߝ
uint8_t send_buffer[SEND_BUFFER_SIZE];
volatile uint8_t dma_tx_busy = 0;


float tempFloat[4];                    //֨ӥքʱҤ
uint8_t tempData[16];                    //֨ӥքԫˤBuffer (3+1)*4

int32_t ia_test,ib_test,ic_test;

double Ia,Ib,Ic;
float Ia_test,Ib_test,Ic_test;
float Vbus;
uint16_t ADC1ConvertedValue[5];
uint16_t i = 0;
uint32_t A_offset,B_offset;
uint8_t get_offset_flag = 0;
uint8_t get_offset_sample_cnt = 0;
u8 oled_display_sample_freq = 0;
u8 speed_close_loop_flag;
float Iq_ref;
float EKF_Hz;

float theta_add;
float theta,myref=0.00001;

float motor_direction = 1.0f;

extern float Rs;
extern float Ls;
extern float flux;
void send_PC(float wave1,float wave2,float wave3,float wave4);

void get_offset(uint32_t *a_offset,uint32_t *b_offset)
{
  if(get_offset_sample_cnt<128)
  {
    *a_offset += ADC1->JDR2;
    *b_offset += ADC1->JDR3;
    get_offset_sample_cnt++;
  }
  else
  {
    *a_offset >>= 7;
    *b_offset >>= 7;
    get_offset_sample_cnt=0;
    TIM_CtrlPWMOutputs(PWM_TIM,DISABLE);
    get_offset_flag = 2;
  }
}

void motor_run(void)
{
  float vbus_temp;
  double ia_temp,ib_temp;
  vbus_temp = (float)(ADC1->JDR1);                                //փսĸП֧ѹ adcתۻֵ
  ia_temp = (int16_t)((int16_t)A_offset - (int16_t)ADC1->JDR2);   //փսAР֧· adcתۻֵ
  ib_temp = (int16_t)((int16_t)B_offset - (int16_t)ADC1->JDR3);   //փսBР֧· adcתۻֵ
  Vbus = vbus_temp*VBUS_CONVERSION_FACTOR;                        //ͨڽ֧ѹתۻӲؓèͨڽؖѹ֧بփսéёadcתۻֵ תۯΪ ֦ʵ֧ѹ
  Ia = ia_temp*SAMPLE_CURR_CON_FACTOR;                            //ͨڽ֧·תۻӲؓèͨڽӉҹ֧بۍՋ̣؅ճѶ˽փսéёadcӉҹֵתۯΪ֦ʵ֧·ֵ
  Ib = ib_temp*SAMPLE_CURR_CON_FACTOR;                            //ͨڽ֧·תۻӲؓèͨڽӉҹ֧بۍՋ̣؅ճѶ˽փսéёadcӉҹֵתۯΪ֦ʵ֧·ֵ
  Ic = -Ia-Ib;                                                    //ܹԚܹ׻ܴز֧·֨ÉìٹߝABР֧·ȥ݆̣CР֧·
  Ia_test = Ia;
  Ib_test = Ib;
  Ic_test = Ic;
  
  int_test2 = ADC1ConvertedValue[0];
  
  if(speed_close_loop_flag==0)         //̙׈۷ҕ۷Ȑۻ࠘׆ì֧ٕܺǴ֯ʱ̙׈۷һҕ۷
  {                                    //ҢȒ֧·ӎ߼ֵۺýնݓè؀ԥܷéì̙׈կսһֵ֨
    if((Iq_ref<MOTOR_STARTUP_CURRENT*motor_direction)) //̙׈Ȑɫҕ۷
    {                                  //֧·۷՚֧ܺՋѐڽԌאȫԌҕ۷
      Iq_ref += 0.001f;              //އ׈՚֧ٕܺǴ֯ʱߍҕ۷ՋѐìϞѨǿΏìփӦԚߨ׻üËҨض   0.00003f; 
    }                                  //״̬ڛӢǷ֍̙єŜ҈ތۃ
    else
    {
      speed_close_loop_flag=1;
    }
  }
  else
  {
    if(speed_close_loop_flag==1)
    {
      if(Iq_ref>(MOTOR_STARTUP_CURRENT*motor_direction/2.0f))
      {
        Iq_ref -= 0.001f;
      }
      else
      {
        speed_close_loop_flag=2;
      }
    }
  }
  
  
  float_test3 = Speed_Ref*2.0f*PI;
  
//ȐۻԐِܲϞِ׽ʽՋѐìҲߍˇѡձՓϞِ״̬ڛӢǷ׽ʽփսއ׈ۍ̙׈хϢ۹ˇՓԐِ׽ʽփսއ׈ۍ̙׈хϢ  
#ifdef  HALL_FOC_SELECT            //ͨڽ͵ݾҠӫѡձԐِFOCՋѐì  

  if((hall_speed*2.0f*PI)>SPEED_LOOP_CLOSE_RAD_S)     //Ԑِ׽ʽՋѐìܴ׻ԫِǷփսއ׈ۍ̙׈хϢ
  {
    FOC_Input.Id_ref = 0.0f; //hall
    Speed_Fdk = hall_speed*2.0f*PI;
    FOC_Input.Iq_ref = Speed_Pid_Out;
  }
  else
  {
    FOC_Input.Id_ref = 0.0f; //hall
    FOC_Input.Iq_ref = Iq_ref;
    Speed_Pid.I_Sum = Iq_ref;
  }
  FOC_Input.theta = hall_angle;
  FOC_Input.speed_fdk = hall_speed*2.0f*PI;
  
#endif
  
#ifdef  SENSORLESS_FOC_SELECT            //ͨڽ͵ݾҠӫѡձϞِFOCՋѐ
  
  //if(FOC_Output.EKF[2]>SPEED_LOOP_CLOSE_RAD_S)      //Ϟِ׽ʽՋѐì״̬ڛӢǷփսއ׈ۍ̙׈хϢ
	if (fabs(FOC_Output.EKF[2]) > fabs(SPEED_LOOP_CLOSE_RAD_S))
  {
    FOC_Input.Id_ref = 0.0f; //less
    Speed_Fdk = FOC_Output.EKF[2];
    FOC_Input.Iq_ref = Speed_Pid_Out;
  }
  else
  {
    FOC_Input.Id_ref = 0.0f;  //less
    FOC_Input.Iq_ref = Iq_ref;
    Speed_Pid.I_Sum = Iq_ref;
  }
  FOC_Input.theta = FOC_Output.EKF[3]+myref;
  FOC_Input.speed_fdk = FOC_Output.EKF[2];
  
#endif  

  
  
  EKF_Hz = FOC_Output.EKF[2]/(2.0f*PI);
  FOC_Input.Id_ref = 0;//0.0f; //less              
  FOC_Input.Tpwm = PWM_TIM_PULSE_TPWM;         //FOCՋѐگ˽ѨҪԃսքˤɫхϢ
  FOC_Input.Udc = Vbus;
  FOC_Input.Rs = Rs;
  FOC_Input.Ls = Ls;
  FOC_Input.flux = flux;
  
  FOC_Input.ia = Ia;
  FOC_Input.ib = Ib;
  FOC_Input.ic = Ic;           
  foc_algorithm_step();       //ֻٶFOCՋѐگ˽èѼ(Ϟِ״̬ڛӢǷì֧·۷ìSVPWMìظҪҤۻì֧ܺӎ˽ʶҰé
  
  if(motor_start_stop==1)
  {
    PWM_TIM->CCR1 = (u16)(FOC_Output.Tcmp1);     //ͨڽSVPWMփսքռࠕ҈سֵٸ֨ʱǷք݄զǷ
    PWM_TIM->CCR2 = (u16)(FOC_Output.Tcmp2);
    PWM_TIM->CCR3 = (u16)(FOC_Output.Tcmp3);
  }
  else
  {
    PWM_TIM->CCR1 = PWM_TIM_PULSE>>1;
    PWM_TIM->CCR2 = PWM_TIM_PULSE>>1;
    PWM_TIM->CCR3 = PWM_TIM_PULSE>>1;
  }
  
  drv8301_protection(); 
  //communication_task();//ԭѦʏλܺɎϱ
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	//ՋԦўلʏԫҨю˽ߝܰȐۻʏλܺìʏλܺʹԃǤאһٶȫעˍ­һٶ
	
	//vofa֚ɽ׽ʏλܺ  ֧·aì֧·bìíаҨìתؓλ׃
	//send_PC(FOC_Input.ia,FOC_Input.ib,FOC_Output.EKF[2],FOC_Output.EKF[3]);
	
	
	//ؔߪעʏλܺ   ֧·aì֧·bìíаҨìתؓλ׃ìӎ߼̙׈ìEFK״!̙׈
	//SendWaveformData(FOC_Input.ia,FOC_Input.ib,FOC_Output.Tcmp1,FOC_Output.EKF[3],Speed_Ref*60,Speed_Fdk*60/6.28318548F);
	

	////////////////////////////////////////////////////////////////////////////////////////////////
	
  oled_display_sample_freq++;
  if(oled_display_sample_freq == 10)         //֧·ѥؔոOLEDДʾǁ ģŢʾҨǷДʾҨю٦Ŝ
  {
    if(display_data_flag==0)
    {
      display_data_buff[display_data_buff_cnt]= (s8)(FOC_Output.EKF[3]*15.0f);
      display_data_buff_cnt++;
      if(display_data_buff_cnt==127)
      {
        display_data_buff_cnt=0;
        display_data_flag=1;
      }
    }
    oled_display_sample_freq=0;
  }
}


void ADC_IRQHandler(void)
{
  
  if((SAMPLE_ADC->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
  {
    if(get_offset_flag==2)
    {
      hall_angle += hall_angle_add;
      if(hall_angle<0.0f)
      {
        hall_angle += 2.0f*PI;
      }
      else if(hall_angle>(2.0f*PI))
      {
        hall_angle -= 2.0f*PI;
      }
      motor_run();
      
    }
    else
    {
      if(get_offset_flag==1)
      {
        get_offset(&A_offset,&B_offset);
      }
    }
    ADC_ClearFlag(SAMPLE_ADC, ADC_FLAG_JEOC);
  }
}

////עٸvofaʏλܺ
//void send_PC(float wave1,float wave2,float wave3,float wave4)
//{
//	tempFloat[0] = wave1;    //(float)תԉء֣˽
//	tempFloat[1] = wave2;
//	tempFloat[2] = wave3;
//	tempFloat[3] = wave4;
//	memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));//ͨڽ߽Ѵё˽ߝטтֻm
//	 
//	//1	
//	USART_SendData(USART2, tempData[0]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[1]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[2]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[3]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//  //2	
//	USART_SendData(USART2, tempData[4]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[5]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[6]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[7]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//	//3	
//	USART_SendData(USART2, tempData[8]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[9]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[10]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[11]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//	//4	
//	USART_SendData(USART2, tempData[12]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[13]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[14]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[15]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));

//	//β	
//	USART_SendData(USART2, 0x00);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x00);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x80);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x7F);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//}

//// ע̍ٸVOFAʏλܺ
//void send_PC(float wave1, float wave2, float wave3, float wave4)
//{
//    //static uint8_t wave_index = 0;  // ޲̬Ҥ݇¼ձǰكע̍ńٶҨю
//    
//    // ݫ4ٶҨю˽ߝզɫ˽ة
//    tempFloat[0] = wave1;
//    tempFloat[1] = wave2;
//    tempFloat[2] = wave3;
//    tempFloat[3] = wave4;
//    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));
//	
//		for(uint8_t i=0;i<16;i++){

//					USART_SendData(USART2, tempData[i]);
//					while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//		}
//    
//    // ע̍β˽ߝèڌ֨4ٶؖޚé
//    USART_SendData(USART2, 0x00); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x00); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x80); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x7F); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//    
//    // ټт̷ӽìѭ۷ע̍0,1,2,3
//    //wave_index = (wave_index + 1) % 2;
//}


// ўلsend_PCگ˽
void send_PC(float wave1, float wave2, float wave3, float wave4)
{
    // ɧڻDMAֽ՚ԫˤì͸ڽѾՎע̍ӔҜĢԥͻ
    if(dma_tx_busy) {
        return;
    }
    
    // ݫ4ٶҨю˽ߝզɫ˽ة
    tempFloat[0] = wave1;
    tempFloat[1] = wave2;
    tempFloat[2] = wave3;
    tempFloat[3] = wave4;
    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));
    
    // ׼Ѹע̍ۺԥȸú16ؖޚҨю˽ߝ + 4ؖޚβ˽ߝ
    memcpy(send_buffer, tempData, 16);
    send_buffer[16] = 0x00;
    send_buffer[17] = 0x00;
    send_buffer[18] = 0x80;
    send_buffer[19] = 0x7F;
    
    // Ƥ׃ҢǴ֯DMAԫˤ
    DMA_Cmd(DMA1_Stream6, DISABLE);
    
    // ȥԽ̹ԐDMAҪ־
    DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);
    
    // ʨ׃DMAӎ˽
    USART2_TX_DMA_STREAM->M0AR = (uint32_t)send_buffer;
    USART2_TX_DMA_STREAM->NDTR = SEND_BUFFER_SIZE;
    
    // ʹŜDMAԫˤΪԉא׏
    DMA_ITConfig(USART2_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
    
    // Ǵ֯DMAԫˤ
    dma_tx_busy = 1;
    DMA_Cmd(USART2_TX_DMA_STREAM, ENABLE);
    
    // ʹŜUSART2քDMAԫˤ
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

// DMAԫˤΪԉא׏Ԧmگ˽
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        
        // ޻ԃUSART2քDMAԫˤ
        USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
        
        // ޻ԃDMA
        DMA_Cmd(DMA1_Stream6, DISABLE);
        
        // ȥԽæҪ־
        dma_tx_busy = 0;
    }
}
