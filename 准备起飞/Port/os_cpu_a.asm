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
			
NVIC_INT_CTRL   	EQU     0xE000ED04  ; �жϿ��ƼĴ���
NVIC_SYSPRI2    	EQU     0xE000ED22  ; ϵͳ���ȼ��Ĵ���(2)
NVIC_PENDSV_PRI 	EQU         0xF0  ; PendSV�жϺ�ϵͳ�����ж�
                                        ; (��Ϊ��ͣ�0xff).
NVIC_PENDSVSET  	EQU     0x10000000  ; ��������жϵ�ֵ.

		PRESERVE8 	
			
		AREA    My_Text, CODE, READONLY
        THUMB 
OS_CPU_SR_Save				;���ж�
    MRS     R0, PRIMASK  	;��ȡPRIMASK��R0,R0Ϊ����ֵ 
    CPSID   I				;PRIMASK=1,���ж�(NMI��Ӳ��FAULT������Ӧ)
    BX      LR			    ;����
	

OS_CPU_SR_Restore			;���ж�
    MSR     PRIMASK, R0	   	;��ȡR0��PRIMASK��,R0Ϊ����
    BX      LR				;����
	
	
;* ��������: ʹ�õ��������е�һ������
OSStartHighRdy
        LDR     R4, =NVIC_SYSPRI2      ;����PendSV�����ȼ�
        LDR     R5, =NVIC_PENDSV_PRI
        STR     R5, [R4]
        MOV     R4, #0                 ;R4 = 0 ������PSP = 0
        MSR     PSP, R4
        LDR     R4, =OSRunning         ; OSRunning = TRUE��1��
        MOV     R5, #1
        STRB    R5, [R4]
                                       ;�л���������ȼ�������
        LDR     R4, =NVIC_INT_CTRL     ;����PendSV�쳣 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
        CPSIE   I                      ;���ж�
		
OSStartHang
        B       OSStartHang            ;��ѭ���������������ʧ�ܵĻ��ͻ����OSStartHang
		
  ;* ��������: �����������л�     
OSCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL  	;����PendSV�쳣 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
		
;* ��������: �жϼ������л�
OSIntCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL      ;����PendSV�쳣 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
        NOP
		
;* ��������: OSPendSV is used to cause a context switch.
PendSV_Handler
    CPSID   I                                                   ; ���ж�
    MRS     R0, PSP              								;R0 = PSP��ջָ�� 
    CBZ     R0, PendSV_Handler_Nosave		                    ;���PSP = 0(˵��Ϊ��һ���������л�������Ҫ��ջ����) ��ת�Ƶ�PendSV_Handler_Nosave
	TST 	R14, #0x10								
	IT 		EQ													;���������ڲ���EXC_RETURN��bit4�������ж��Ƿ���FPU���Ӷ������Ƿ���ջS16-S31
	VSTMDBEQ R0!, {S16-S31} 
    SUBS    R0, R0, #0x20                                       ; ����ʣ���R4-R11
    STM     R0, {R4-R11}
    LDR     R1, =OSTCBCur                                       ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R1]
    STR     R0, [R1]                                            ; R0 is SP of process being switched out
	
PendSV_Handler_Nosave
    PUSH    {R14}                                               ; ����R14��ֵ������Ҫ���ú���
    LDR     R0, =OSTaskSwHook                                   ; OSTaskSwHook();
    BLX     R0
    POP     {R14} 												;�ָ�R14��ֵ
    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]
    LDR     R0, =OSTCBCur                                       ; OSTCBCur  = OSTCBHighRdy��ָ��ǰ���ȼ��������
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
