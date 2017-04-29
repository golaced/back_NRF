#include  "sbus.h"
uint8_t sbus_data_i[26] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
uint8_t sbus_data[26] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
int16_t channels[18]  = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t  failsafe_status = SBUS_SIGNAL_FAILSAFE;
uint8_t sbus_passthrough = 1;
void sbus_anal_oldx(void) {
    // Read all received data and calculate channel data
    uint8_t i;
    uint8_t sbus_pointer = 0;

				// reset counters
				uint8_t byte_in_sbus = 1;
				uint8_t bit_in_sbus = 0;
				uint8_t ch = 0;
				uint8_t bit_in_channel = 0;
        channels[0]  = ((sbus_data[1]|sbus_data[2]<<8) & 0x07FF); //The first Channel
        channels[1]  = ((sbus_data[2]>>3|sbus_data[3]<<5)  & 0x07FF);
        channels[2]  = ((sbus_data[3]>>6 |sbus_data[4]<<2 |sbus_data[5]<<10)  & 0x07FF);
        channels[3]  = ((sbus_data[5]>>1 |sbus_data[6]<<7) & 0x07FF);
        channels[4]  = ((sbus_data[6]>>4 |sbus_data[7]<<4) & 0x07FF);
        channels[5]  = ((sbus_data[7]>>7 |sbus_data[8]<<1 |sbus_data[9]<<9)   & 0x07FF);
        channels[6]  = ((sbus_data[9]>>2 |sbus_data[10]<<6) & 0x07FF);
        channels[7]  = ((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF);
        channels[8]  = ((sbus_data[12]   |sbus_data[13]<<8) & 0x07FF);
        channels[9]  = ((sbus_data[13]>>3|sbus_data[14]<<5)  & 0x07FF);
        channels[10] = ((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF);
        channels[11] = ((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF);
        channels[12] = ((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF);
        channels[13] = ((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9)  & 0x07FF);
        channels[14] = ((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF);
        channels[15] = ((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF);
        channels[16] = ((sbus_data[23]));
				
				for(i=0;i<17;i++)
				channels[i]+=500;
				// Failsafe
//				failsafe_status = SBUS_SIGNAL_OK;
//				if (sbus_data[23] & (1<<2)) {
//						failsafe_status = SBUS_SIGNAL_LOST;
//				}
//				if (sbus_data[23] & (1<<3)) {
//						failsafe_status = SBUS_SIGNAL_FAILSAFE;
//				}	
		    if(channels[16]==500||channels[16]==503)
					failsafe_status=SBUS_SIGNAL_OK;
				else
					failsafe_status=SBUS_SIGNAL_LOST;
}

void oldx_sbus_rx(u8 com_data)
{
u8 i;
static u8 state;
static uint8_t byteCNT = 0;	
switch(state)
		{
			case 0:
			if(com_data==0x0f)	
			{byteCNT=0;sbus_data_i[byteCNT++]=com_data;state=1;}
				break;
		  case 1:
				sbus_data_i[byteCNT++]=com_data;
 			  if(byteCNT>=25)
				{
						 if(sbus_data_i[24]==0)
						{
							for(i=0;i<25;i++)
							sbus_data[i]=sbus_data_i[i];
							sbus_anal_oldx();
						}
				state=0;
				byteCNT=0;
				}
			  break;
		}
}