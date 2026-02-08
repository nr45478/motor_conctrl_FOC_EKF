/**********************************
      
**********************************/
#include "main.h"
#include "foc_algorithm.h"
#include "position_pid.h"
#include "position_sensor.h"
#include "control_mode.h"

real32_T D_PI_I = 1282.8F;
real32_T D_PI_KB = 15.0F;
real32_T D_PI_LOW_LIMIT = -24.0F;
real32_T D_PI_P = 2.199F;
real32_T D_PI_UP_LIMIT = 24.0F;
real32_T Q_PI_I = 1282.8F;
real32_T Q_PI_KB = 15.0F;
real32_T Q_PI_LOW_LIMIT = -24.0F;
real32_T Q_PI_P = 2.199F;
real32_T Q_PI_UP_LIMIT = 24.0F;


FOC_INTERFACE_STATES_DEF FOC_Interface_states;


FOC_INPUT_DEF FOC_Input;


FOC_OUTPUT_DEF FOC_Output;


RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void stm32_ekf_Start_wrapper(real_T *xD);
  extern void stm32_ekf_Outputs_wrapper(const real32_T *u,
                                        real32_T *y,
                                        const real_T *xD);
  extern void stm32_ekf_Update_wrapper(const real32_T *u,
                                       real32_T *y,
                                       real_T *xD);
  extern void stm32_ekf_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void L_identification_Start_wrapper(real_T *xD);
  extern void L_identification_Outputs_wrapper(const real32_T *u,
                                               real32_T *y,
                                               const real_T *xD);
  extern void L_identification_Update_wrapper(const real32_T *u,
                                              real32_T *y,
                                              real_T *xD);
  extern void L_identification_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void R_flux_identification_Start_wrapper(real_T *xD);
  extern void R_flux_identification_Outputs_wrapper(const real32_T *u,
                                                    real32_T *y,
                                                    const real_T *xD);
  extern void R_flux_identification_Update_wrapper(const real32_T *u,
                                                   real32_T *y,
                                                   real_T *xD);
  extern void R_flux_identification_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

extern float float_test1;
extern float float_test2;

CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
VOLTAGE_DQ_DEF Voltage_DQ;
CURRENT_PID_DEF Current_D_PID;
CURRENT_PID_DEF Current_Q_PID;

/***************************************
¹¦ÄÜ£ºClark±ä»»
ÐÎ²Î£ºÈýÏàµçÁ÷ÒÔ¼°alpha_betaµçÁ÷
ËµÃ÷£ºÓÉÈýÏà»¥²î120¶È±ä»»µ½Á½Ïà»¥²î90¶È
***************************************/
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
  Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * 0.5F) * 2.0F / 3.0F;
  Current_alpha_beta_temp->Ibeta = (Current_abc_temp.Ib - Current_abc_temp.Ic) * 0.866025388F * 2.0F / 3.0F;
}
/***************************************
¹¦ÄÜ£ºSVPWM¼ÆËã
ÐÎ²Î£ºalpha_betaµçÑ¹ÒÔ¼°Ä¸ÏßµçÑ¹¡¢¶¨Ê±Æ÷ÖÜÆÚ
ËµÃ÷£º¸ù¾Ýalpha_betaµçÑ¹¼ÆËãÈýÏàÕ¼¿Õ±È
***************************************/
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp)
{
  int32_T sector;
  real32_T Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
  sector = 0;
  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
  if (v_alpha_beta_temp.Vbeta > 0.0F) {
    sector = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 4;
  }
  
  switch (sector) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > Tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
    break;
  }
  
  FOC_Output.Tcmp1 = Tcmp1;
  FOC_Output.Tcmp2 = Tcmp2;
  FOC_Output.Tcmp3 = Tcmp3;
}

/***************************************
¹¦ÄÜ£ºCOS_SINÖµ¼ÆËã
ÐÎ²Î£º½Ç¶ÈÒÔ¼°COS_SIN½á¹¹Ìå
ËµÃ÷£ºCOS_SINÖµ¼ÆËã
***************************************/
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp)
{
  cos_sin_temp->Cos = arm_cos_f32(angle_temp);
  cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}
/***************************************
¹¦ÄÜ£ºPARK±ä»»
ÐÎ²Î£ºalpha_betaµçÁ÷¡¢COS_SINÖµ¡¢DQÖáµçÁ÷
ËµÃ÷£º½»Á÷±äÖ±Á÷
***************************************/
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp)
{
  current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
  current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}
/***************************************
¹¦ÄÜ£º·´PARK±ä»»
ÐÎ²Î£ºDQÖáµçÑ¹¡¢COS_SINÖµ¡¢alpha_betaµçÑ¹
ËµÃ÷£ºÖ±Á÷±ä½»Á÷
***************************************/
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}

/***************************************
¹¦ÄÜ£ºµçÁ÷»·PID
ÐÎ²Î£ºµçÁ÷²Î¿¼¡¢µçÁ÷·´À¡¡¢µçÑ¹Êä³ö¡¢PID½á¹¹Ìå
ËµÃ÷£º¸ù¾ÝµçÁ÷Îó²îÈ¥µ÷½ÚµçÁ÷Êä³ö
***************************************/
void Current_PID_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,CURRENT_PID_DEF* current_pid_temp)
{
  real32_T error;
  real32_T temp;
  error = ref_temp - fdb_temp;
  temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;
  if (temp > current_pid_temp->Max_Output) 
  {
    *out_temp = current_pid_temp->Max_Output;
  } 
  else if (temp < current_pid_temp->Min_Output) 
  {
    *out_temp = current_pid_temp->Min_Output;
  } 
  else 
  {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) *0.0001f;
}



void foc_algorithm_step(void)
{

  Current_Iabc.Ia = FOC_Input.ia;         //ÈýÏàµçÁ÷¸³Öµ
  Current_Iabc.Ib = FOC_Input.ib;
  Current_Iabc.Ic = FOC_Input.ic;
  
  Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK ±ä»»
  Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //ÓÉ½Ç¶È¼ÆËã park±ä»»ºÍ ·´park±ä»»µÄ COS SINÖµ
  Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park±ä»»£¬ÓÉIalpha Ibeta Óë½Ç¶ÈÐÅÏ¢£¬È¥¼ÆËãId Iq  // ÓÉ½»Á÷ÐÅÏ¢×ª»¯ÎªÖ±Á÷ÐÅÏ¢£¬·½±ãPID¿ØÖÆ 
  Current_PID_Calc(FOC_Input.Id_ref,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);     //DÖáµçÁ÷»·PID  ¸ù¾ÝµçÁ÷²Î¿¼ÓëµçÁ÷·´À¡È¥¼ÆËã Êä³öµçÑ¹
  Current_PID_Calc(FOC_Input.Iq_ref,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID);     //QÖáµçÁ÷»·PID  ¸ù¾ÝµçÁ÷²Î¿¼ÓëµçÁ÷·´À¡È¥¼ÆËã Êä³öµçÑ¹
  Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //·´park±ä»»  Í¨¹ýµçÁ÷»·µÃµ½µÄdqÖáµçÑ¹ÐÅÏ¢½áºÏ½Ç¶ÈÐÅÏ¢£¬È¥°ÑÖ±Á÷ÐÅÏ¢×ª»¯Îª½»Á÷ÐÅÏ¢ÓÃÓÚSVPWMµÄÊäÈë

  FOC_Interface_states.EKF_Interface[0] = Voltage_Alpha_Beta.Valpha;   //À©Õ¹¿¨¶ûÂü¹À¼Æ×ª×ÓÎ»ÖÃÓëËÙ¶ÈÐèÒªµÄÊäÈëÐÅÏ¢
  FOC_Interface_states.EKF_Interface[1] = Voltage_Alpha_Beta.Vbeta;    //×´Ì¬¹Û²âÆ÷ÊäÈë
  FOC_Interface_states.EKF_Interface[2] = Current_Ialpha_beta.Ialpha;
  FOC_Interface_states.EKF_Interface[3] = Current_Ialpha_beta.Ibeta;
  FOC_Interface_states.EKF_Interface[4] = FOC_Input.Rs;
  FOC_Interface_states.EKF_Interface[5] = FOC_Input.Ls;
  FOC_Interface_states.EKF_Interface[6] = FOC_Input.flux;
  

  stm32_ekf_Outputs_wrapper(&FOC_Interface_states.EKF_Interface[0], &FOC_Output.EKF[0],  //À©Õ¹¿¨¶ûÂü¹À¼Æ×ª×ÓÎ»ÖÃÓëËÙ¶ÈµÄÊä³öº¯Êý
                            &FOC_Interface_states.EKF_States[0]);

  FOC_Interface_states.R_flux_Ident_Interface[0] = Current_Idq.Iq;         //µç»úµç×èÓë´ÅÁ´²ÎÊýÊ¶±ðËã·¨µÄÊäÈë
  FOC_Interface_states.R_flux_Ident_Interface[1] = FOC_Input.speed_fdk;
  FOC_Interface_states.R_flux_Ident_Interface[2] = Voltage_DQ.Vq;

  FOC_Interface_states.L_Ident_Interface[0] = -(Current_Idq.Iq * FOC_Input.speed_fdk);//µç»úµç¸Ð²ÎÊýÊ¶±ðËã·¨µÄÊäÈë
  FOC_Interface_states.L_Ident_Interface[1] = Voltage_DQ.Vd;
  

  L_identification_Outputs_wrapper(&FOC_Interface_states.L_Ident_Interface[0],  //µç»úµç¸Ð²ÎÊýÊ¶±ðËã·¨µÄÊä³ö
                                   &FOC_Interface_states.L_Ident_Output, &FOC_Interface_states.L_Ident_States);
  

  R_flux_identification_Outputs_wrapper(&FOC_Interface_states.R_flux_Ident_Interface[0],//µç»úµç×èÓë´ÅÁ´²ÎÊýÊ¶±ðËã·¨µÄÊä³ö
                                        &FOC_Interface_states.R_flux_Ident_Output[0], &FOC_Interface_states.R_flux_Ident_States);
  
  
	#define DEAD_TIME_COMP   -0.0005f   // æ ¹æ®å®žé™…è°ƒï¼Œä¸€èˆ¬0.2~1.0V

	float Ia_sign = (Current_Iabc.Ia > 0.02f) ? 1.0f : 
									(Current_Iabc.Ia < -0.02f) ? -1.0f : 0.0f;
	float Ib_sign = (Current_Iabc.Ib > 0.02f) ? 1.0f : 
									(Current_Iabc.Ib < -0.02f) ? -1.0f : 0.0f;
	float Ic_sign = (Current_Iabc.Ic > 0.02f) ? 1.0f : 
									(Current_Iabc.Ic < -0.02f) ? -1.0f : 0.0f;

	// è¡¥å¿åˆ°Î±Î²è½´
	Voltage_Alpha_Beta.Valpha += DEAD_TIME_COMP * 
			(2.0f*Ia_sign - Ib_sign - Ic_sign) / 3.0f;
	Voltage_Alpha_Beta.Vbeta += DEAD_TIME_COMP * 
			(Ib_sign - Ic_sign) * 0.577f;  // 1/sqrt(3)
  SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,FOC_Input.Tpwm);       //SVPWM ¼ÆËãÄ£¿é
 

  stm32_ekf_Update_wrapper(&FOC_Interface_states.EKF_Interface[0], &FOC_Output.EKF[0],   //À©Õ¹¿¨¶ûÂüÂË²¨Ëã·¨µÄ¼ÆËã
                           &FOC_Interface_states.EKF_States[0]);                         //Ò²¾ÍÊÇÎÞ¸Ð×´Ì¬¹Û²âÆ÷µÄ¼ÆËã
  

  L_identification_Update_wrapper(&FOC_Interface_states.L_Ident_Interface[0],//µç»úµç¸Ð²ÎÊýÊ¶±ðËã·¨µÄ¼ÆËã
                                  &FOC_Interface_states.L_Ident_Output, &FOC_Interface_states.L_Ident_States);
  
 
  R_flux_identification_Update_wrapper(&FOC_Interface_states.R_flux_Ident_Interface[0],//µç»úµç×èÓë´ÅÁ´²ÎÊýÊ¶±ðËã·¨µÄ¼ÆËã
                                       &FOC_Interface_states.R_flux_Ident_Output[0], &FOC_Interface_states.R_flux_Ident_States);
  

  FOC_Output.L_RF[0] = FOC_Interface_states.L_Ident_Output;
  FOC_Output.L_RF[1] = FOC_Interface_states.R_flux_Ident_Output[0];
  FOC_Output.L_RF[2] = FOC_Interface_states.R_flux_Ident_Output[1];
	  
  // ========== ??????6:???????(????)==========
  position_sensor_update();

	
}


void foc_algorithm_initialize(void)
{
  //µçÁ÷»·PID ²ÎÊý ³õÊ¼»¯
  {
  Current_D_PID.P_Gain = D_PI_P;
  Current_D_PID.I_Gain = D_PI_I;
  Current_D_PID.B_Gain = D_PI_KB;
  Current_D_PID.Max_Output = D_PI_UP_LIMIT;
  Current_D_PID.Min_Output = D_PI_LOW_LIMIT;
  Current_D_PID.I_Sum = 0.0f;
  
  Current_Q_PID.P_Gain = Q_PI_P;
  Current_Q_PID.I_Gain = Q_PI_I;
  Current_Q_PID.B_Gain = Q_PI_KB;
  Current_Q_PID.Max_Output = Q_PI_UP_LIMIT;
  Current_Q_PID.Min_Output = Q_PI_LOW_LIMIT;
  Current_Q_PID.I_Sum = 0.0f;
  }
  speed_pid_initialize();  //ËÙ¶È»·PID ²ÎÊý ³õÊ¼»¯
               
  stm32_ekf_Start_wrapper(&FOC_Interface_states.EKF_States[0]);//À©Õ¹¿¨¶ûÂüÂË²¨Ëã·¨ ²ÎÊý³õÊ¼»¯

  L_identification_Start_wrapper(&FOC_Interface_states.L_Ident_States);//µç»úµç¸Ð²ÎÊýÊ¶±ðËã·¨ ²ÎÊý³õÊ¼»¯

  R_flux_identification_Start_wrapper(&FOC_Interface_states.R_flux_Ident_States);//µç»úµç×èÓë´ÅÁ´²ÎÊýÊ¶±ðËã·¨ ²ÎÊý³õÊ¼»¯
  
  //×´Ì¬±äÁ¿³õÊ¼»¯
  {
    real_T initVector[4] = { 0, 0, 0, 0 };
    
    {
      int_T i1;
      real_T *dw_DSTATE = &FOC_Interface_states.EKF_States[0];
      for (i1=0; i1 < 4; i1++) {
        dw_DSTATE[i1] = initVector[i1];
      }
    }
  }
 
  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        FOC_Interface_states.L_Ident_States = initVector[0];
      }
    }
  }
  

  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        FOC_Interface_states.R_flux_Ident_States = initVector[0];
      }
    }
  }
  	{
			// ???????PID???
		position_pid_initialize();
		
		// ????????????(??????)
		position_sensor_init(POS_METHOD_HALL_INTERP);
		
		// ???????????(??????)
		control_mode_init();
	}

}

