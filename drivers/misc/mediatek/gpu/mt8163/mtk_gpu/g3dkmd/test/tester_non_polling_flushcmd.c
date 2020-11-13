#include "g3dbase_common_define.h" // for define G3DKMD_USE_BUFFERFILE 
#include "g3dkmd_define.h"


#if defined(TEST_NON_POLLING_FLUSHCMD)

#include <linux/types.h>    // for NULL
#include <linux/sched.h>    // for current
#include <linux/delay.h>    // for mdelay

#include "tester_non_polling_flushcmd.h"
#include "g3dkmd_api.h"                  // lock/unlock_g3d()
#include "g3dkmd_file_io.h"              // KMDDPF family
#include "g3dkmd_macro.h"                // VAL_IRQ_*
#include "g3dkmd_engine.h"               // g3dkmdWriteReg(...)



#define TESTER_FALSE 0
#define TESTER_TRUE  1

#define TESTER_OPEN  1
#define TESTER_CLOSE 0


static void tester_trigger( tester_t *obj, const int taskID);

static tester_t tester = {
    .is_fence_opened = TESTER_TRUE,
    .instance        = NULL,
};

void _init_tester(tester_t *pObj)
{
    pObj->is_fence_opened = TESTER_FALSE;
    pObj->instance = pObj;

    atomic_set(&(pObj->cmdCnt), 0);
    atomic_set(&pObj->wqCnt , 0);
    pObj->triggerWhnWqCnt = TRIGGER_WHEN_WAITQUEUE_COUNT;

    pObj->triggerFunc = tester_trigger;

    return ;
}

tester_t* tester_getInstance()
{
    if(!tester.instance) {
        _init_tester(&tester);
    }
    return &tester;
}


int _tester_fence_block_door(tester_t * obj) 
{
    if (!obj->is_fence_opened) {
        return TESTER_CLOSE;
    }

    // someone getting in fence block, so reset the door flag.
    obj->is_fence_opened = TESTER_FALSE;
    return TESTER_OPEN;
}

int _tester_isHtNotFull(tester_t * obj, const int isNotFull, const int taskID)
{
    if (isNotFull) { // command is push into queue
        atomic_inc(&obj->cmdCnt);
//        KMDDPF_ENG( G3DKMD_MSG_NOTICE, "%s: %d cmd is queued.\n",
//                     __FUNCTION__, atomic_read(&obj->cmdCnt));
    } else { 
        atomic_inc(&obj->wqCnt);
        KMDDPF_ENG(G3DKMD_MSG_NOTICE, "a[%d] taskID:%d: %s\n", current->pid, 
                    taskID, (isNotFull)?"Enough":"Not enough");
        KMDDPF_ENG(G3DKMD_MSG_NOTICE, "%s: %d cmd is queued.\n",
                    __FUNCTION__, atomic_read(&obj->cmdCnt));
        KMDDPF_ENG(G3DKMD_MSG_NOTICE, "%s: %d proces is queued.\n",
                    __FUNCTION__, atomic_read(&obj->wqCnt));
        if (atomic_read(&obj->wqCnt) > obj->triggerWhnWqCnt)
        {
            obj->triggerFunc(obj, taskID);

            // reset counters
            atomic_set(&obj->cmdCnt, 0);
            atomic_set(&obj->wqCnt,  0);

            obj->is_fence_opened = TESTER_TRUE;
	    printk("==> is_fence_opened: %d\n", obj->is_fence_opened);
            return 1; // return 1, becaue triggerFunc will update writePtr.
        }
    }

    return isNotFull;
}

void _wake_up_callback(const char * funcName)
{
    //KMDDPF_ENG(G3DKMD_MSG_NOTICE, "[%d][%s] wake up s_flushCmdWq\n", 
    //            current->tgid, funcName);
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "[%s] wake up s_flushCmdWq.\n", funcName);

}

void _tester_wq_release(const int taskID)
{
    //KMDDPF_ENG(G3DKMD_MSG_NOTICE, "[%d] NON-POLLING: released(taskID:%d)\n", 
    //        current->tgid, taskID);
    {
        unsigned int available = 0;
        pG3dExeInst pInst;
        pG3dHtInfo ht;

        lock_g3d();

        pInst = g3dKmdTaskGetInstPtr(taskID);
        ht    = pInst->htInfo;
        available = g3dKmdCheckFreeHtSlots(pInst->hwChannel, pInst->htInfo);


        unlock_g3d();
//*
        KMDDPF_ENG(G3DKMD_MSG_NOTICE, 
                   "NON-POLLING: released(taskID, avaliable, fence, rdPtr, wtPtr, pInst, HwChnl)=(%d, %u, %d, %u, %u,%p, %d)\n",
                   taskID, available, tester_getInstance()->is_fence_opened, 
                   ht->rdPtr, ht->wtPtr, pInst, pInst->hwChannel);
//*/
    }
}


void _tester_wq_singaled(const int taskID)
{
    //KMDDPF_ENG(G3DKMD_MSG_NOTICE, "[%d] NON-POLLING: incoming signal(taskID:%d)\n", 
    //            current->tgid, taskID);
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "NON-POLLING: incoming signal(taskID:%d)\n", taskID);
}

void _tester_wq_timeout(const int taskID)
{
    //KMDDPF_ENG(G3DKMD_MSG_NOTICE, "[%d] NON-POLLING: timeout(taskID:%d)\n", 
    //        current->tgid, taskID));
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "NON-POLLING: timeout(taskID:%d)\n", taskID);
}

//static void g3dKmdIsrTop(unsigned long var);
void g3dKmdInvokeIsrTop(void);

static void tester_trigger(tester_t *obj, const int taskID)
{
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "tester_trigger( taskID: %d.\n", taskID);

#ifndef G3D_HW
    {
    // update write pointer ( simulate HW done the works)
        pG3dExeInst pInst;
        int ret;

        lock_g3d();

        pInst = g3dKmdTaskGetInstPtr(taskID);
        ret   = g3dKmdCheckIfEnoughHtSlots(pInst->hwChannel, pInst->htInfo);

        obj->is_fence_opened = TESTER_OPEN;
        g3dKmdRegUpdateReadPtr(pInst->hwChannel, pInst->htInfo->wtPtr);
        obj->is_fence_opened = TESTER_CLOSE;

        unlock_g3d();

    // set Frame_end interrupt.
        g3dKmdRegWrite(REG_AREG_IRQ_STATUS_MASKED, VAL_IRQ_PA_FRAME_END,
                       MSK_AREG_IRQ_MASKED, SFT_AREG_IRQ_MASK);

    // call interrupt handler
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "===> g3dKmdIsrTop is triggering.\n");
        g3dKmdInvokeIsrTop();
    KMDDPF_ENG(G3DKMD_MSG_NOTICE, "<=== g3dKmdIsrTop is triggered.\n");

    // unset Frame_end interrupt.
    //   ==> IsrTop will automatically reset flags as HW's behaviour.
    }
#endif
}


#endif
