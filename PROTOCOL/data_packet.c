#include "data_packet.h"
#include "string.h"
#include "judge_tx_data.h"
#include "judge_rx_data.h"
#include "pc_tx_data.h"
#include "pc_rx_data.h"

/*******************************************************************/
/*裁判系统和小电脑的接收数据和打包*/
/*******************************************************************/

/*获取DMA状态数据*/
static void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id = DMA_GetCurrentMemoryTarget(dma_stream);//DMA_GetCurrentMemoryTarget获取当前内存目标
  
  *remain_cnt = DMA_GetCurrDataCounter(dma_stream);
}

//for debug
int dma_write_len = 0;
int fifo_overflow = 0;
/*
* @ dma_obj ： judge_rx_obj  //初始化时已经把串口接收的数据缓冲区地址赋给了judge_rx_obj的成员buff
*/


/*裁判系统更改尝试*/
//DMA全满中断

/*裁判系统更改尝试*/
void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type,int16_t full_tmp_len)
{
  uint8_t  current_memory_id;
  uint16_t remain_data_counter;

  get_dma_memory_msg(dma_obj->dma_stream, &current_memory_id, &remain_data_counter);//双缓冲模式读取当前所用存储器0/1以及读取出要传输的数据项数目
  uint8_t  *pdata = dma_obj->buff[current_memory_id];
	

//	dma_obj->write_index = dma_obj->buff_size- remain_data_counter;
//	if(it_type==UART_IDLE_IT)
//	{
//		tmp_len=dma_obj->write_index-dma_obj->read_index;
//		fifo_s_puts(dma_obj->data_fifo,&pdata[0],tmp_len);
//		dma_obj->read_index = dma_obj->write_index;
//		
//	}
	if(it_type==UART_DMA_FULL_IT)
	{
//    pdata = dma_obj->buff[!current_memory_id];
//		tmp_len=dma_obj->buff_size-dma_obj->read_index;
    fifo_s_puts(dma_obj->data_fifo,&pdata[0],full_tmp_len);
//		dma_obj->write_index=0;
//		dma_obj->read_index = dma_obj->write_index;
	}
}


static uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  static uint8_t seq;
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  
  p_header->sof          = sof;
  p_header->data_length  = len;
  
  
  if (sof == UP_REG_ID)
  {
    if (seq++ >= 255)
      seq = 0;
    
    p_header->seq = seq;
  }
  else
  {
    p_header->seq = 0;
  }
  
  
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(tx_buf, frame_length);
  
  return tx_buf;
}

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  
  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);
  
  /* use mutex operation */
  if (sof == UP_REG_ID)
  {
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);        //pc_txdata_fifo
  }
  else if (sof == DN_REG_ID)
    fifo_s_puts(&pc_txdata_fifo, tx_buf, frame_length);     //judge_txdata_fifo
  else
    return ;
}

/*
* @p_obj：judge_unpack_obj
* @sof： DN_REG_ID
*/

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof,uint8_t control_type)
{
	
  uint8_t byte = 0;
  
  while ( fifo_used_count(p_obj->data_fifo) )
  {
    byte = fifo_s_get(p_obj->data_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN -CMD_LEN- CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN  + CMD_LEN+p_obj->data_len + CRC_LEN) )// HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN
          {
            if (sof == UP_REG_ID)
            {
							if(control_type==CONTROLLER)
							{
								controller_data_handler(p_obj->protocol_packet);
							}else if(control_type==NORMAL)
							{
								judgement_data_handler(p_obj->protocol_packet);
							}
//              pc_data_handler(p_obj->protocol_packet);
							
            }
            else  //DN_REG_ID
            {
              if(control_type==CONTROLLER)
							{
								controller_data_handler(p_obj->protocol_packet);
							}else if(control_type==NORMAL)
							{
								judgement_data_handler(p_obj->protocol_packet);
							}
            }
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


/************************************************************************/
/*裁判系统和小电脑的发送*/
/************************************************************************/

static void usart_send_data(USART_TypeDef* USARTx,uint8_t *pData, uint16_t Size)
{
  while(Size > 0U)
  {
    Size--;
    while((USARTx->SR & USART_FLAG_TXE) != USART_FLAG_TXE)
    {
    }
    USARTx->DR = (*pData++ & (uint8_t)0xFF);
  }
}

uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{
#if (JUDGE_TX_FIFO_BUFLEN > PC_TX_FIFO_BUFLEN)
  uint8_t  tx_buf[JUDGE_TX_FIFO_BUFLEN];
#else
  uint8_t  tx_buf[PC_TX_FIFO_BUFLEN];
#endif
  
  uint32_t fifo_count = fifo_used_count(pfifo);
  
  if (fifo_count)
  {
    fifo_s_gets(pfifo, tx_buf, fifo_count);
    
    if (sof == UP_REG_ID)
      usart_send_data(USART3, tx_buf, fifo_count);
//    else if (sof == DN_REG_ID)
//      usart_send_data(USART6, tx_buf, fifo_count);//新改
    else
      return 0;
  }
  
  return fifo_count;
}


