#include "include.h"

void sd_task_init(void);//---------------------------------------------------------------------------------------
void SD_Save_Task(float delay);
extern u8 en_save,sdcard_force_out,sd_had_init;

struct _SD{
				u8 init,sd_insert;
				u8 en_save,en_save_force;
        u32 save_time;
        float task_detal;
          };

extern struct _SD sd;