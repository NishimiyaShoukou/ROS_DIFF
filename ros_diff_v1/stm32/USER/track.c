#include "track.h"
#include "gpio.h"
#include "contrl.h"
#include "move.h"
#include "chasis.h"
uint8_t Led_buffer[7];  
uint8_t step_max=6;

void Get_Led(void)
{
	 //Led_buffer=0;
	 Led_buffer[rin3]=HAL_GPIO_ReadPin(rin3_GPIO_Port,rin3_Pin);
	 Led_buffer[rin2]=HAL_GPIO_ReadPin(rin2_GPIO_Port,rin2_Pin);
	 Led_buffer[rin1]=HAL_GPIO_ReadPin(rin1_GPIO_Port,rin1_Pin);
	 Led_buffer[mid]=HAL_GPIO_ReadPin(mid_GPIO_Port,mid_Pin);
	 Led_buffer[lin1]=HAL_GPIO_ReadPin(lin1_GPIO_Port,lin1_Pin);
	 Led_buffer[lin2]=HAL_GPIO_ReadPin(lin2_GPIO_Port,lin2_Pin);
	 Led_buffer[lin3]=HAL_GPIO_ReadPin(lin3_GPIO_Port,lin3_Pin);
}


//巡线前进
void track_line(void)
{
	 static uint8_t buf=0,buf1=0,buf2=0;
	 if(Led_buffer[rin3]==1&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,90);
		  
		 
	 }
	 else  if(Led_buffer[rin3]==1&&Led_buffer[rin2]==1&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,80);
		  
		 
	 }
	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==1&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,60);
		  
		 
	 }
	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==1&&Led_buffer[rin1]==1&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,60);
		  
		 
	 }
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==1&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,50);
		  
		 
	 }
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(40,45);
		  
		 
	 }
	 
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==1&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		   if(MAP.cur_step<1)
		  Set_speed(40,40);
		 else if(MAP.cur_step==1)
			Set_speed(20,20);
		 else
			 Set_speed(40,40); 
		  
		 
	 }
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==1&&Led_buffer[lin1]==1&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(45,40);
		  
		 
	 }
	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==1&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
	 {
		  Set_speed(50,40);
		  
		 
	 }
	   else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==1&&Led_buffer[lin2]==1&&Led_buffer[lin3]==0)
	 {
		  Set_speed(60,40);
		  
		 
	 }
	   else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==1&&Led_buffer[lin3]==0)
	 {
		  Set_speed(70,40);
		  
		 
	 }
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==1&&Led_buffer[lin3]==1)
	 {
		  Set_speed(80,40);
		  
		 
	 }
	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==1)
	 {
		  Set_speed(90,40);
		  
		 
	 }
   else if((Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1)||(Led_buffer[rin1]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1)
		 ||(Led_buffer[rin1]==1&&Led_buffer[lin2]==1) ||(Led_buffer[rin2]==1&&Led_buffer[lin1]==1))
	 {
		 
		  //HAL_Delay(1000);
			Decision_cross();
//		 if(Miracle!=START)
//		 {
		  //while((Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1));
			 if(Miracle==START)
			 {
				 if(Drug_task.expect_drug<=2)
	     {
				   if(buf!=(Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1))
					MAP.cur_step+=1;
				 else if(buf1!=Led_buffer[rin1]+Led_buffer[lin1])
					MAP.cur_step+=1;
				 else if(buf2!=Led_buffer[rin2]+Led_buffer[lin1])
					MAP.cur_step+=1;
				 if(MAP.cur_step>=step_max)
					 MAP.cur_step=step_max;
			   }else
			   {  
				    if(buf!=(Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1))
					MAP.cur_step+=1;
				 }
				 
			 }
				 else if(Miracle==RETURN)
					 
				 
				 {
					 
					 Decision_cross();
						 if(Drug_task.expect_drug>2)
				   {
					   
							if(buf!=(Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1))
						MAP.re_step+=1;
					 }
					}
		  
		 
				

		  
			 
	 }
	 else{
		 if(MAP.cur_step<1)
		  Set_speed(40,40);
		 else if(MAP.cur_step==1)
			Set_speed(20,20);
		 else
			 Set_speed(40,40); 
		 
	 }
	 
		 buf=Led_buffer[rin2]+Led_buffer[mid]+1+Led_buffer[lin1];
		 buf1=Led_buffer[rin1]+Led_buffer[lin2];
		 buf2=Led_buffer[rin1]+Led_buffer[lin1];
	
}

void track_line_back(void)
{
	 static uint8_t buf=0;
//		if(Led_buffer[rin3]==1&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,90);  //右轮和左轮
//		  
//		 
//	 }
//	 else  if(Led_buffer[rin3]==1&&Led_buffer[rin2]==1&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,80);
//		  
//		 
//	 }	 
//	
//	 	 if(Led_buffer[rin3]==1&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,90);
//		  
//		 
//	 }
//	 else  if(Led_buffer[rin3]==1&&Led_buffer[rin2]==1&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,80);
//		  
//		 
//	 }
//	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==1&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,60);
//		  
//		 
//	 }
//	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==1&&Led_buffer[rin1]==1&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,60);
//		  
//		 
//	 }
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==1&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,50);
//		  
//		 
//	 }
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,45);
//		  
//		 
//	 }
//	 
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==1&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(40,40);
//		  
//		 
//	 }
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==1&&Led_buffer[lin1]==1&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(45,40);
//		  
//		 
//	 }
//	 else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==1&&Led_buffer[lin2]==0&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(50,40);
//		  
//		 
//	 }
//	   else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==1&&Led_buffer[lin2]==1&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(60,40);
//		  
//		 
//	 }
//	   else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==1&&Led_buffer[lin3]==0)
//	 {
//		   Set_speed_back(70,40);
//		  
//		 
//	 }
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==1&&Led_buffer[lin3]==1)
//	 {
//		   Set_speed_back(80,40);
//		  
//		 
//	 }
//	  else  if(Led_buffer[rin3]==0&&Led_buffer[rin2]==0&&Led_buffer[rin1]==0&&Led_buffer[mid]==0&&Led_buffer[lin1]==0&&Led_buffer[lin2]==0&&Led_buffer[lin3]==1)
//	 {
//		   Set_speed_back(90,40);
//		  
//		 
//	 }
   if((Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1)||(Led_buffer[rin1]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1)||(Led_buffer[rin1]==1&&Led_buffer[lin1]==1))
	 {
		 
		  //HAL_Delay(1000);
			Decision_cross();
		  //while((Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1));
		 if(Drug_task.expect_drug==1||Drug_task.expect_drug==2)
				  MAP.re_step=1;
		 if(buf!=(Led_buffer[rin2]==1&&Led_buffer[mid]==1&&Led_buffer[lin1]==1))
					MAP.cur_step+=1;
//		 else if((Led_buffer[rin2]+Led_buffer[lin1])!=buf)
//		  MAP.re_step+=1;
		 if(MAP.re_step>=step_max)
				 MAP.re_step=step_max;
	 }
//	 else{
//		 
//		  Set_speed_back(50,50);
//		  
//		 
//	 }
	 buf=Led_buffer[rin2]+Led_buffer[mid]+1+Led_buffer[lin1];
	 
	
}



void Decision_cross(void)
{
	if(Miracle==START)
	{
				if(Drug_task.expect_drug==1)
				{
					
					 if(((MAP.cur_step-MAP.begin_step)>=1))
					 {
							 Miracle=PLACE;
						 
					 }
					 else
					 {
							 Move(ML90);
							 Map_Record(ML90);
							// MAP.cur_step+=1;
							
					 }
					// MAP.Route[0]=ML90;
				}
				if(Drug_task.expect_drug==2)
				{
					
					 if(((MAP.cur_step-MAP.begin_step)>=1))
					 {
							Miracle=PLACE;
						 
					 }
					 else
					 {
							Move(MR90);
							Map_Record(MR90);
						//  MAP.cur_step+=1;
						 
					 }
				}
				if(MAP.cur_step>=1)
				{
					 if(Drug_task.curr_state>0&&((MAP.cur_step-MAP.begin_step)>=1))
					 {
							Miracle=PLACE;
					 }
					 else
					 {
						 Move(Car_app); //动作执行判断函数
						 Map_Record(Car_app);
					 }
					
				}
				else
				{
					 Move(MForward);
				}
		}
	  else if(Miracle==RETURN)
		{
			
			    if(MAP.re_step<1)
					{
						Move(MStop);
						HAL_Delay(200);
					 // Move(MBack);
						if(MAP.cur_step>=1)
						Move(MAP.Route[MAP.cur_step-2]);
					}
					else if(MAP.re_step>=1&&MAP.cur_step>=3)
					{
						//if(MAP.cur_step>=1&&MAP.cur_step<=1)
						Move(MStop);
						Go_origin();
						Miracle=WAIT;
					}
					else if(MAP.re_step>=5&&MAP.cur_step>=4&&Drug_task.curr_state==2)
					{
						
							Move(MStop);
						  Go_origin();
						  Miracle=WAIT;
						
					}
//					else
//					{
//						 if(MAP.cur_step>=5)
//						 Move(MAP.Route[MAP.cur_step-1]);										
//					   
//					}
//			  if(MAP.re_step>=1)
//				{
//				      
//						 Move(MAP.Route[MAP.cur_step]);
//					   
////						 Map_Record(Car_app);
//					   
//				}
			  
			
			
		}

}



