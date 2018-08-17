#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_it.h"
#include "stdio.h"
#include "uart1.h"
#include "stm8s_gpio.h"
#include "spi.h"
#include "time4.h"
#include <string.h>
#include "problock.h"

//#define DEBUG


#define BLOCK_MAX       50             
#define UART_RX_LEN     BLOCK_MAX

const uint32_t SwitchPin[] = {KEY1,KEY2,KEY3,KEY4};

volatile static uint8_t gs_u8UartRxBuffer[UART_RX_LEN];
volatile static int rxBufferIndex = 0;
volatile static int nCount = 0;
volatile static EventPos nPos = IDEL_POS;
volatile static uint8_t gs_u8RxStartFlg = 0;

static ST_ProTxBlock stProParam;
static uint8_t gs_u8PortNum = 0;
static uint8_t gs_u8GetIdFlg = 0;


void uart1_init(uint32_t BaudRate)
{
  //BaudRate=9600,start_bit=1,data_bit=8,parity_bit=none,stop_bit=1,enable TX and RX
  UART1_Init(BaudRate, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
  //enable RX interrupt 
  UART1_ITConfig(UART1_IT_RXNE, ENABLE);
}

void UART1_SendByte(uint8_t dat)
{
  while( (UART1_GetFlagStatus(UART1_FLAG_TXE)==RESET) );

  UART1_SendData8(dat);
	
}

void UART1_Send(uint8_t* pbuf, int len)
{
  while(len--)
  {
    UART1_SendByte(*pbuf++);
  }
}

//void spi_init()
//{
//  //f=4MHz,MSB,CPOL=0,CPHA=0
//  SPI_Init(SPI_MODE_MASTER, 4000000, SPI_FIRSTBIT_MSB, SPI_DATA_MODE0);
////  //enable TX interrupt 
////  SPI_ITConfig(SPI_IT_TXE, ENABLE);
//}

/**
  * @brief  Receive a byte from 74HC165.
  * @param  None.
  * @retval The value of the received data.
  */
static uint8_t SoftwareSPI_RX(void)
{
  uint8_t d = 0;

  for(int i=7; i>=0; i--)
  {
    //对于读取165，当SH/LD由低变高电平后，8个输入口的最高位已经在QH上了，即第一位数据是不需要时钟上升沿的（其他数据位需要时钟上升沿才能进入QH）。
    d |= GPIO_DigitalRead(SW_MISO) << i;

    GPIO_DigitalWrite(SW_SCK, LOW);  
    GPIO_DigitalWrite(SW_SCK, HIGH);
  }
  GPIO_DigitalWrite(SW_SCK, LOW);

  return d;
}


static uint8_t getSwitchValFormPin(void)
{
  uint8_t u8val = 0;

  for(int i=0; i<sizeof(SwitchPin); i++)
  {
    if(HIGH == GPIO_DigitalRead(SwitchPin[i]))
    {
      u8val |= (0x01 << i);
    }       
  }
  
  return u8val;
}


static void ProBolck_IO_init()
{
  //SoftwareSPI
  GPIO_Init(SW_SCK, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(SW_MISO, GPIO_MODE_IN_PU_NO_IT); 
  GPIO_Init(CS1, GPIO_MODE_OUT_PP_HIGH_SLOW);
  //CD4052 CD4051
  GPIO_Init(CHA, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(CHB, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(CHC, GPIO_MODE_OUT_PP_LOW_SLOW);
  //init errLed
  GPIO_Init(ERR_LED, GPIO_MODE_OUT_PP_HIGH_SLOW);
  //Switch Key
  GPIO_Init(KEY1, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(KEY2, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(KEY3, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(KEY4, GPIO_MODE_IN_FL_NO_IT);
}


static void SetErrLed(ErrSts sts)
{
  static uint8_t last_d = HIGH;

  switch( sts )
  {
    case ERR_INIT:
      GPIO_DigitalWrite(ERR_LED, LOW);//->ON
      last_d = LOW;
      break;
    case ERR_OK:
      GPIO_DigitalWrite(ERR_LED, HIGH);//->OFF
      last_d = HIGH;
      break;
    case ERR_CNT1://ERROR2:HSE failed
      GPIO_DigitalWrite(ERR_LED, LOW);
      delay_ms(200);
      GPIO_DigitalWrite(ERR_LED, HIGH);
      delay_ms(200);
      GPIO_DigitalWrite(ERR_LED, LOW);
      last_d = LOW;
      break;
    case ERR_CNT2://ERROR3:Receive timeout
      GPIO_DigitalWrite(ERR_LED, !last_d);
      delay_ms(500);
      GPIO_DigitalWrite(ERR_LED, last_d);
      delay_ms(500);
      GPIO_DigitalWrite(ERR_LED, !last_d);
      delay_ms(500);
      GPIO_DigitalWrite(ERR_LED, last_d);   
      break;
    case ERR_LOOP://ERROR1:Invalid parameter
      while(1)
      {
        GPIO_DigitalWrite(ERR_LED, LOW);
        delay_ms(200);
        GPIO_DigitalWrite(ERR_LED, HIGH);
        delay_ms(200);
      }
      break;
    default:
      break;
  }
}

//static uint8_t u8sts[10];
static bool getBlockParam(ST_ProTxBlock *pProParam)
{
  int i;
  int nTotal = 0;
  uint8_t u8Switch = 0;
  uint8_t u8CmdID = 0;
  uint8_t u8temp_val = 0;
  
  nTotal = sizeof(BlockList)/sizeof(ST_BlockObject);

  GPIO_DigitalWrite(CS1, LOW);
  GPIO_DigitalWrite(CS1, HIGH);
  //get CmdID
  u8CmdID = SoftwareSPI_RX();
  //check CmdID
  for(i=0; i<nTotal; i++)
  {
    if(BlockList[i].u8CMD == u8CmdID)
    {
      break;
    }
  }
  
  if(i >= nTotal)
  {
    return FALSE;
  }
  //get Switch
  if(TYPE0 != BlockList[i].swType)
  {
    if( (TYPE3 != BlockList[i].swType) && (TYPE4 != BlockList[i].swType) )
    {
      u8Switch = getSwitchValFormPin();
    }
    else
    {
      u8Switch = SoftwareSPI_RX();
    }
  }

  //check Switch
  switch(BlockList[i].swType)
  {
    case TYPE0:
      break;
    case TYPE1:
      if( (0x01 != u8Switch) && (0x02 != u8Switch) && (0x04 != u8Switch) && (0x08 != u8Switch) )
      {
        return FALSE;
      }
      break;
    case TYPE2:
      if(u8Switch > 0x0F)
      {
        return FALSE;
      }
      break;
    case TYPE3:
      u8temp_val = u8Switch & 0x0F;
      if( (0x00 != u8temp_val) && (0x01 != u8temp_val) && (0x02 != u8temp_val) && (0x04 != u8temp_val) &&
          (0x05 != u8temp_val) && (0x06 != u8temp_val) && (0x08 != u8temp_val) && (0x09 != u8temp_val) &&
          (0x0A != u8temp_val) )
      {
        return FALSE;
      }
      u8temp_val = u8Switch >> 4;
      if( (0x00 != u8temp_val) && (0x01 != u8temp_val) && (0x02 != u8temp_val) && (0x04 != u8temp_val) &&
          (0x05 != u8temp_val) && (0x06 != u8temp_val) && (0x08 != u8temp_val) && (0x09 != u8temp_val) &&
          (0x0A != u8temp_val) )
      {
        return FALSE;
      }
      break;
    case TYPE4:     
      u8temp_val = u8Switch & 0x0F;
      if( (0x01 != u8temp_val) && (0x02 != u8temp_val) && (0x04 != u8temp_val) && (0x08 != u8temp_val) )
      {
        return FALSE;
      }
      u8temp_val = u8Switch >> 4;
      if( (0x01 != u8temp_val) && (0x02 != u8temp_val) && (0x04 != u8temp_val) && (0x08 != u8temp_val) )
      {
        return FALSE;
      }
      break;
  default:
    break;
  }
  //save port num
  gs_u8PortNum = BlockList[i].u8PortNum;
  //set ProParam
  pProParam->u8StartCode = SC_ACK;
  pProParam->u8Len = 2;
  pProParam->u8CmdID = u8CmdID;
  pProParam->u8Switch = u8Switch;

  return TRUE;
}

static void Select_CD405x_CH(uint8_t u8ch)
{
  if(gs_u8PortNum > 4)  //CD4051
  {
    switch( u8ch )
    {
      case 0:     //X0
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, LOW);
        GPIO_DigitalWrite(CHC, LOW);
        break;
      case 1:     //X1
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, LOW);
        GPIO_DigitalWrite(CHC, LOW);
        break;
      case 2:     //X2
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, HIGH);
        GPIO_DigitalWrite(CHC, LOW);
        break;
      case 3:     //X3
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, HIGH);
        GPIO_DigitalWrite(CHC, LOW);
        break;
      case 4:     //X4
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, LOW);
        GPIO_DigitalWrite(CHC, HIGH);
        break;
      case 5:     //X5
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, LOW);
        GPIO_DigitalWrite(CHC, HIGH);
        break;
      case 6:     //X6
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, HIGH);
        GPIO_DigitalWrite(CHC, HIGH);
        break;
      case 7:     //X7
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, HIGH);
        GPIO_DigitalWrite(CHC, HIGH);
        break;
      default:
        break;
    }
  }
  else                  //CD4052                                  
  {
    switch( u8ch )
    {
      case 0:     //X0.Y0
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, LOW);
        break;
      case 1:     //X1,Y1
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, LOW);
        break;
      case 2:     //X2,Y2
        GPIO_DigitalWrite(CHA, LOW);
        GPIO_DigitalWrite(CHB, HIGH);
        break;
      case 3:     //X3,Y3
        GPIO_DigitalWrite(CHA, HIGH);
        GPIO_DigitalWrite(CHB, HIGH);
        break;
      default:
        break;
    }
  }
}


static void STM8s_Reset()
{
  
  uint8_t u8SndBuf[3];
  
  u8SndBuf[0] = SC_REQ;                 //start code
  u8SndBuf[1] = 1;                      //data length
  u8SndBuf[2] = CMD_RESET;              //cmd
  
  for(uint8_t i=1; i<gs_u8PortNum; i++)
  {
    Select_CD405x_CH(i);
    UART1_Send(u8SndBuf, 3);
    delay_ms(10);
  }
  //reset  
  WWDG->CR = 0x80;//WDGA=1
}


static void Select_Send_Next(uint8_t u8ch, uint8_t* pbuf, int len)
{
  uint8_t u8SndBuf[UART_RX_LEN];

  Select_CD405x_CH(u8ch);

  u8SndBuf[0] = SC_REQ;                 //start code
  u8SndBuf[1] = len;                    //data length
  memcpy(&u8SndBuf[2], pbuf, len);      //cmd data
  
  UART1_Send(u8SndBuf, len+2);
}




void main( void )
{
  uint8_t u8HseErrFlg = 0;
  unsigned long startMs = 0;

//  //system clock init
//  CLK_Config(CLK_SOURCE_HSI, CLK_PRESCALER_HSIDIV1);//16MHz

  //system clock init=16Mhz
  if( !CLK_Config(CLK_SOURCE_HSE, CLK_PRESCALER_CPUDIV1) )
  {
    CLK_Config(CLK_SOURCE_HSI, CLK_PRESCALER_HSIDIV1);//16MHz
    u8HseErrFlg = 1;
  }
  //uart1
  uart1_init(9600);
  //TIM4
  TIM4_Init(TIM4_PRESCALER_64, 0xFF);
  //GPIO
  ProBolck_IO_init(); 
  //enable global Interrupts
  enableInterrupts();
  
  delay_ms(5);

  //check block param
  if( FALSE == getBlockParam(&stProParam) )
  {
    SetErrLed(ERR_LOOP);//ERROR1:Invalid parameter
  }
  else if(u8HseErrFlg)
  {
    SetErrLed(ERR_CNT1);//ERROR2:HSE failed
  }
  else
  {
    //turn on ErrLed
    SetErrLed(ERR_INIT);
  }
  
  while(1)
  {
    switch( gs_u8RxStartFlg )
    {
      case 1:
        startMs = millis();
        gs_u8RxStartFlg++;
        break;
      case 2:
        if(millis() - startMs >= 50)//NOTE:timeout=50ms,must < RX_BLOCK_TIMEOUT in problock_Pro_Mini_V6!!!
        {
          gs_u8RxStartFlg = 0;
          nPos = IDEL_POS;
          SetErrLed(ERR_CNT2);//ERROR3:Receive timeout
        }
        break;
      default:
        break;
    }

    
    if(END_POS == nPos)
    {
      switch(gs_u8UartRxBuffer[0])
      {
        case CMD_GET_ID:         
          if( getBlockParam(&stProParam) )
          {
            Select_CD405x_CH(0);
            UART1_Send((uint8_t*)&stProParam, sizeof(stProParam));
            gs_u8GetIdFlg = 1;
          }
          else
          {
            SetErrLed(ERR_LOOP);
          }      
          break;
        case CMD_SET_IN1:
          Select_Send_Next(1, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          //turn off ErrLed
          if(gs_u8GetIdFlg)
          {
            SetErrLed(ERR_OK);
            gs_u8GetIdFlg = 0;
          }
          break;
        case CMD_SET_IN2:
          Select_Send_Next(2, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_SET_IN3:
          Select_Send_Next(3, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_SET_IN4:
          Select_Send_Next(4, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_SET_IN5:
          Select_Send_Next(5, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_SET_IN6:
          Select_Send_Next(6, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_SET_IN7:
          Select_Send_Next(7, (uint8_t*)&gs_u8UartRxBuffer[1], nCount-1);
          break;
        case CMD_RESET:
          STM8s_Reset();
          break;
        default:
          break;
      }
      
      memset((uint8_t*)gs_u8UartRxBuffer, 0, sizeof(gs_u8UartRxBuffer));
      nCount = 0;
      gs_u8RxStartFlg = 0;
      nPos = IDEL_POS;
    }
  }

}

#ifdef DEBUG
uint8_t u8rx_buffer[50];
int n = 0;
#endif


#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)

/**
  * @brief UART1 RX Interrupt routine.
  * @param  None
  * @retval None
  * @note   Originally defined in stm8s_it.c
  */
 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
   
   uint8_t u8dat = 0;
   static int cnt_temp = 0;

   if( UART1_GetFlagStatus(UART1_FLAG_OR) == SET )          //Overrun Error interrupt
   {
     //read UART_DR to clear error flag
     UART1_ReceiveData8();//ERROR4:Overrun interrupt
     nPos = IDEL_POS;
   }
   else if( UART1_GetFlagStatus(UART1_FLAG_RXNE) == SET )  //Receive interrupt
   {
     u8dat = UART1_ReceiveData8();

#ifdef DEBUG
     u8rx_buffer[n++] = u8dat;
     if(n >= sizeof(u8rx_buffer))
     {
       n = 0;
     }
#endif

     switch( nPos )
     {
       case IDEL_POS:
         if(SC_REQ == u8dat)
         {
           nPos = SC_POS;
           rxBufferIndex = 0;
           gs_u8RxStartFlg = 1;
//         gs_u8UartRxBuffer[rxBufferIndex++] = u8dat;
         }
         break;
       case SC_POS:
          nCount = u8dat;
          if(nCount > sizeof(gs_u8UartRxBuffer))
          {
            nCount = sizeof(gs_u8UartRxBuffer);
          }
          if(nCount > 0)
          {
            nPos = LEN_POS;
          }
          else
          {
            nPos = END_POS;
          }
          cnt_temp = nCount;    //-->to avoid Warning[Pa082]: undefined behavior: the order of volatile accesses is undefined in this statement.
//        gs_u8UartRxBuffer[rxBufferIndex++] = u8dat;
          break;
       case LEN_POS:
         gs_u8UartRxBuffer[rxBufferIndex++] = u8dat;
         if(rxBufferIndex >= cnt_temp)
         {
           nPos = END_POS;
         }     
         break;
       case END_POS:  
         break;       
         
       default:
         break;
     }
   } 
   
 }

#endif /*STM8S208 or STM8S207 or STM8S103 or STM8S903 or STM8AF62Ax or STM8AF52Ax */