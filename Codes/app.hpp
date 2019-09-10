#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"
#include "parameter.h"



#define MAIN_PRIORITY  TMIN_APP_TPRI + 4
#define REC_PRIORITY   TMIN_APP_TPRI + 2
#define JUD_PRIORITY   TMIN_APP_TPRI + 1
#define OPE_PRIORITY   TMIN_APP_TPRI + 3
#define BT_PRIORITY    TMIN_APP_TPRI + 5

#ifndef STACK_SIZE
#define STACK_SIZE      4096
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY


extern void main_task(intptr_t exinf);
extern void bt_task  (intptr_t exinf);

extern void rec_task (intptr_t exinf);
extern void rec_cyc  (intptr_t exinf);

extern void jud_task (intptr_t exinf);
extern void jud_cyc  (intptr_t exinf);

extern void ope_task (intptr_t exinf);
extern void ope_cyc  (intptr_t exinf);
	

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
