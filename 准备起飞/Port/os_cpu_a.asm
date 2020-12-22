		IMPORT  OSRunning               ; External references
        IMPORT  OSPrioCur
        IMPORT  OSPrioHighRdy
        IMPORT  OSTCBCur
        IMPORT  OSTCBHighRdy
        IMPORT  OSIntNesting
        IMPORT  OSIntExit
        IMPORT  OSTaskSwHook   
			
        EXPORT  OSStartHighRdy               
        EXPORT  OSCtxSw
        EXPORT  OSIntCtxSw
		EXPORT  OS_CPU_SR_Save                                      ; Functions declared in this file
    	EXPORT  OS_CPU_SR_Restore       
        EXPORT  PendSV_Handler
			
NVIC_INT_CTRL   	EQU     0xE000ED04  ; 中断控制寄存器
NVIC_SYSPRI2    	EQU     0xE000ED22  ; 系统优先级寄存器(2)
NVIC_PENDSV_PRI 	EQU         0xF0  ; PendSV中断和系统节拍中断
                                        ; (都为最低，0xff).
NVIC_PENDSVSET  	EQU     0x10000000  ; 触发软件中断的值.

		PRESERVE8 	
			
		AREA    My_Text, CODE, READONLY
        THUMB 
OS_CPU_SR_Save				;关中断
    MRS     R0, PRIMASK  	;读取PRIMASK到R0,R0为返回值 
    CPSID   I				;PRIMASK=1,关中断(NMI和硬件FAULT可以响应)
    BX      LR			    ;返回
	

OS_CPU_SR_Restore			;开中断
    MSR     PRIMASK, R0	   	;读取R0到PRIMASK中,R0为参数
    BX      LR				;返回
	
	
;* 功能描述: 使用调度器运行第一个任务
OSStartHighRdy
        LDR     R4, =NVIC_SYSPRI2      ;设置PendSV的优先级
        LDR     R5, =NVIC_PENDSV_PRI
        STR     R5, [R4]
        MOV     R4, #0                 ;R4 = 0 ，设置PSP = 0
        MSR     PSP, R4
        LDR     R4, =OSRunning         ; OSRunning = TRUE（1）
        MOV     R5, #1
        STRB    R5, [R4]
                                       ;切换到最高优先级的任务
        LDR     R4, =NVIC_INT_CTRL     ;触发PendSV异常 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
        CPSIE   I                      ;开中断
		
OSStartHang
        B       OSStartHang            ;死循环，如果多任务开启失败的话就会进入OSStartHang
		
  ;* 功能描述: 任务级上下文切换     
OSCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL  	;触发PendSV异常 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
		
;* 功能描述: 中断级任务切换
OSIntCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL      ;触发PendSV异常 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
        NOP
		
;* 功能描述: OSPendSV is used to cause a context switch.
PendSV_Handler
    CPSID   I                                                   ; 关中断
    MRS     R0, PSP              								;R0 = PSP堆栈指针 
    CBZ     R0, PendSV_Handler_Nosave		                    ;如果PSP = 0(说明为第一次做任务切换，不需要入栈处理) 就转移到PendSV_Handler_Nosave
	TST 	R14, #0x10								
	IT 		EQ													;这三步是在测试EXC_RETURN的bit4，用来判断是否用FPU，从而决定是否入栈S16-S31
	VSTMDBEQ R0!, {S16-S31} 
    SUBS    R0, R0, #0x20                                       ; 保存剩余的R4-R11
    STM     R0, {R4-R11}
    LDR     R1, =OSTCBCur                                       ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R1]
    STR     R0, [R1]                                            ; R0 is SP of process being switched out
	
PendSV_Handler_Nosave
    PUSH    {R14}                                               ; 保存R14的值，后面要调用函数
    LDR     R0, =OSTaskSwHook                                   ; OSTaskSwHook();
    BLX     R0
    POP     {R14} 												;恢复R14的值
    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]
    LDR     R0, =OSTCBCur                                       ; OSTCBCur  = OSTCBHighRdy，指向当前优先级最高任务
    LDR     R1, =OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]
    LDR     R0, [R2]                                            ; R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    LDM     R0, {R4-R11}                                        ; Restore r4-11 from new process stack
    ADDS    R0, R0, #0x20
	;Is the task using the FPU context? If so, push high vfp registers.
	TST 	R14, #0x10
	IT 		EQ
	VLDMIAEQ R0!, {S16-S31} 
	MSR     PSP, R0                                             ; Load PSP with new process SP
    ORR     LR, LR, #0x04                                       ; Ensure exception return uses process stack
    CPSIE   I
    BX      LR                                                  ; Exception return will restore remaining context
	NOP
    end  
