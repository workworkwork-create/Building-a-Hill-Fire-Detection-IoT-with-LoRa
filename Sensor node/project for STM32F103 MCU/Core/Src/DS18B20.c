#include "DS18B20.h"

static uint8_t DS18B20_Presence(void);
static void DS18B20_Mode_IPU(void);
static void DS18B20_Mode_Out_PP(void);
static void DS18B20_Rst(void);
static uint8_t DS18B20_ReadBit(void);
static uint8_t DS18B20_ReadByte(void);
static void DS18B20_WriteByte(uint8_t dat);
static void DS18B20_SkipRom(void);
static void DS18B20_MatchRom(void);
void DS18B20_Dout_HIGH(void);
void DS18B20_Dout_LOW(void);

#define DS18B20_Dout_PORT GPIOC 
#define DS18B20_Dout_PIN 0x20	LL_GPIO_PIN_5
#define DS18B20_Dout_GPIO_CLK_ENABLE()     LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC)
//#define DS18B20_Dout_LOW()   do{DS18B20_Dout_PORT->BRR = DS18B20_Dout_PIN  ;}while(0)//HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_RESET)
//#define DS18B20_Dout_HIGH()  do{DS18B20_Dout_PORT->BSRR = DS18B20_Dout_PIN  ;}while(0)//HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_RESET)//HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_SET)
#define DS18B20_Data_IN()   ((DS18B20_Dout_PORT->IDR & 0x20)!=0x00)//HAL_GPIO_ReadPin(DS18B20_Dout_PORT,DS18B20_Dout_PIN)
//#define DS18B20_Data_IN()  LL_GPIO_GetPinPull(DS18B20_Dout_PORT,DS18B20_Dout_PIN)
void DS18B20_Dout_LOW()
{
	DS18B20_Dout_PORT->BRR = 0x20  ;
}
void DS18B20_Dout_HIGH() 
{
	DS18B20_Dout_PORT->BSRR = 0x20  ;

}


static void DS18B20_Delay(uint16_t time)
{
	uint16_t i;

  while(time)
  {   
     for (i = 0; i <10; i++)
    {
      __NOP();
    }
		

    time--;
  }
	
}
uint8_t DS18B20_Init(void)
{
	LL_GPIO_InitTypeDef LL_GPIO_InitStructure;
	
	LL_GPIO_StructInit(&LL_GPIO_InitStructure);
	
  DS18B20_Dout_GPIO_CLK_ENABLE();
  
	LL_GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_InitStructure.Pin = LL_GPIO_PIN_5;
	LL_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	LL_GPIO_Init(GPIOC, &LL_GPIO_InitStructure);
	
  DS18B20_Mode_Out_PP();
        
  DS18B20_Dout_HIGH();
        
  DS18B20_Rst();
  return DS18B20_Presence();
}

static void DS18B20_Mode_IPU(void)
{
  //DS18B20_Dout_GPIO_CLK_ENABLE();

	//LL_GPIO_SetPinMode(DS18B20_Dout_PORT, DS18B20_Dout_PIN, LL_GPIO_MODE_INPUT);
	//LL_GPIO_SetPinPull(DS18B20_Dout_PORT, DS18B20_Dout_PIN, LL_GPIO_PULL_UP);
	
	GPIOC->CRL = ((GPIOC->CRL)&0xFF0FFFFF)|0x00400000;
	GPIOC->BSRR = 0x20;		// pull up
	//DS18B20_Delay(1);
	//LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);
	//GPIOC->BSRR = 0x20;
}

static void DS18B20_Mode_Out_PP(void)
{
  //DS18B20_Dout_GPIO_CLK_ENABLE();
	//GPIOC->CRL = ((GPIOC->CRL)&0xFF0FFFFF)|0x00100000;

	//DS18B20_Dout_GPIO_CLK_ENABLE();
  //LL_GPIO_SetPinMode( DS18B20_Dout_PORT, DS18B20_Dout_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	//LL_GPIO_SetPinSpeed(DS18B20_Dout_PORT,DS18B20_Dout_PIN,LL_GPIO_SPEED_FREQ_LOW);
	
	
	//GPIOC->BSRR = 0x20;		// pull up
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	
	//GPIOC->CRL = ((GPIOC->CRL)&0xFF0FFFFF)|0x00100000;
}

static void DS18B20_Rst(void)
{
		DS18B20_Mode_Out_PP();
		
		DS18B20_Dout_LOW();

		DS18B20_Delay(750);		//750
		
		DS18B20_Dout_HIGH();
		
		DS18B20_Delay(15);
}

static uint8_t DS18B20_Presence(void)
{
		uint8_t pulse_time = 0;
		
		DS18B20_Mode_IPU();
	
		DS18B20_Delay(1);
	 
		while( DS18B20_Data_IN() && pulse_time<100 )
		{
						pulse_time++;
						DS18B20_Delay(1);
		}        
		if( pulse_time >=100 )
						return 1;
		else
						pulse_time = 0;
		while( !DS18B20_Data_IN() && pulse_time<240 )
		{
						pulse_time++;
						DS18B20_Delay(1);
		}        
		if( pulse_time >=240 )
						return 1;
		else
						return 0;
}
static uint8_t DS18B20_ReadBit(void)
{
		uint8_t dat;
		
		DS18B20_Mode_Out_PP();
		DS18B20_Dout_LOW();
		DS18B20_Delay(10);		//10
		
		DS18B20_Mode_IPU();
	
		DS18B20_Delay(1);
		
		if( DS18B20_Data_IN() == SET )
						dat = 1;
		else
						dat = 0;
		
		DS18B20_Delay(45);
		
		return dat;
}
static uint8_t DS18B20_ReadByte(void)
{
		uint8_t i, j, dat = 0;        
		
		for(i=0; i<8; i++)
		{
						j = DS18B20_ReadBit();               
						dat = (dat) | (j<<i);
		}
		
		return dat;
}
static void DS18B20_WriteByte(uint8_t dat)
{
		uint8_t i, testb;
	DS18B20_Dout_HIGH();
		DS18B20_Mode_Out_PP();
		
		for( i=0; i<8; i++ )
		{
						testb = dat&0x01;
						dat = dat>>1;               
						if (testb)
						{                        
										DS18B20_Dout_LOW();
										DS18B20_Delay(8);
										
										DS18B20_Dout_HIGH();
										DS18B20_Delay(58);
						}               
						else
						{                        
										DS18B20_Dout_LOW();
										DS18B20_Delay(80);
										
										DS18B20_Dout_HIGH();               
										DS18B20_Delay(2);
						}
		}
}

static void DS18B20_SkipRom ( void )
{
        DS18B20_Rst();                  
        DS18B20_Presence();                 
        DS18B20_WriteByte(0XCC);                    
}
static void DS18B20_MatchRom ( void )
{
        DS18B20_Rst();                  
        DS18B20_Presence();                 
        DS18B20_WriteByte(0X55);                       
}
float DS18B20_GetTemp_SkipRom ( void )
{
        uint8_t tpmsb, tplsb;
        short s_tem;
        float f_tem;
        
        
        DS18B20_SkipRom ();
        DS18B20_WriteByte(0X44);                                
        
        
        DS18B20_SkipRom ();
				DS18B20_WriteByte(0XBE);                                
        
        tplsb = DS18B20_ReadByte();                 
        tpmsb = DS18B20_ReadByte();
        
				//printf("[%02X %02X]\r\n", tpmsb, tplsb);
        
        s_tem = tpmsb<<8;
        s_tem = s_tem | tplsb;
        
        //if( s_tem < 0 )                
                f_tem = s_tem * 0.0625;        
        //else
        //        f_tem = s_tem * 0.30625;
				
        return f_tem;         
}
void DS18B20_ReadId ( uint8_t * ds18b20_id )
{
        uint8_t uc;
               
        DS18B20_WriteByte(0x33);       
        
        for ( uc = 0; uc < 8; uc ++ )
          ds18b20_id [ uc ] = DS18B20_ReadByte();        
}
float DS18B20_GetTemp_MatchRom ( uint8_t * ds18b20_id )
{
        uint8_t tpmsb, tplsb, i;
        short s_tem;
        float f_tem;
        
        
        DS18B20_MatchRom ();            
        
				for(i=0;i<8;i++)
				{
                DS18B20_WriteByte ( ds18b20_id [ i ] );        
        }
        DS18B20_WriteByte(0X44);                            

        DS18B20_MatchRom ();           
        
        for(i=0;i<8;i++)
				{
                DS18B20_WriteByte ( ds18b20_id [ i ] );        
        }
        DS18B20_WriteByte(0XBE);                                
        
        tplsb = DS18B20_ReadByte();                 
        tpmsb = DS18B20_ReadByte();
        
        
        s_tem = tpmsb<<8;
        s_tem = s_tem | tplsb;
        
        if( s_tem > 0 )                
                f_tem = (s_tem+1) * 0.0625;        
        else
                f_tem = s_tem * 0.0625;
        
        return (s_tem+1)*0.625;                 
}
