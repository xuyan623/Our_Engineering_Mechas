#include "bsp_vofa.h"



float vofa_debug[6] = {0};
const uint8_t tail [4] = {0x00, 0x00, 0x80, 0x7f};	//帧尾
uint8_t c_data[4];//数据帧

/*JustFloat*/

void float_turn_u8(float f,uint8_t * c){
	uint8_t x;
	FloatLongType data;
	data.fdata=f;
	
	for(x=0;x<4;x++){
		c[x]=(uint8_t)(data.ldata>>(x*8));
	}

}

void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose)
{
	uint16_t x;
	uint8_t y;
		for(x=0;x<fdata_num;x++){
			float_turn_u8(fdata[x],c_data);
			for(y=0;y<4;y++){
				Usart_choose->DR=c_data[y];
				while((Usart_choose->SR&0X40)==0);
			}
		}
		for(y=0;y<4;y++){
				Usart_choose->DR=tail[y];
			while((Usart_choose->SR&0X40)==0);
		}

}

void UART7_Test(void)
{
    const char *test_str = "UART7 TEST\n";
    
    while(*test_str) {
        // 等待发送缓冲区空
        while((UART7->SR & USART_SR_TXE) == 0);
        
        // 发送字符
        UART7->DR = *test_str++;
    }
    
    // 等待传输完成
    while((UART7->SR & USART_SR_TC) == 0);
}
/*** example:
**
**	float f_vofa_data[i];
**	JustFloat_Send(&f_vofa_data,i,USARTx)
**
***/