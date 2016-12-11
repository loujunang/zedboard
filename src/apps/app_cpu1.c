/*
 * Copyright (c) 2009 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * helloworld.c: simple test application
 */

#include "xparameters.h"
#include <stdio.h>
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "sleep.h"
#include "xpseudo_asm.h"

/**************************** Type Definitions *******************************/

#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC		    XIntc
#define INTC_HANDLER	XIntc_InterruptHandler
#else
#define INTC		    XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */



#define INTC_DEVICE_ID	    XPAR_PS7_SCUGIC_0_DEVICE_ID
#define PL_IRQ_ID           XPS_IRQ_INT_ID

#define IRQ_PCORE_GEN_BASE  XPAR_IRQ_GEN_0_BASEADDR

#define COMM_VAL        (*(volatile unsigned long *)(0xFFFF8000))

#define COMM_TX_FLAG    (*(volatile unsigned long *)(0xFFFF9000))
#define COMM_TX_DATA    (*(volatile unsigned long *)(0xFFFF9004))
#define COMM_RX_FLAG    (*(volatile unsigned long *)(0xFFFF9008))
#define COMM_RX_DATA    (*(volatile unsigned long *)(0xFFFF900C))


/**
 * This typedef contains configuration information for the device driver.
 */
typedef struct {
	u16 DeviceId;		/**< Unique ID of device */
	u32 BaseAddress;	/**< Base address of the device */
} Pl_Config;


/**
 * The driver instance data. The user is required to allocate a
 * variable of this type.
 * A pointer to a variable of this type is then passed to the driver API
 * functions.
 */
typedef struct {
	Pl_Config Config;   /**< Hardware Configuration */
	u32 IsReady;		/**< Device is initialized and ready */
	u32 IsStarted;		/**< Device is running */
} XPlIrq;



void        Xil_L1DCacheFlush(void);
static int  SetupIntrSystem(INTC *IntcInstancePtr, XPlIrq *PeriphInstancePtr, u16 IntrId);
static void DisableIntrSystem(INTC *IntcInstancePtr, u16 IntrId);
static void PlIntrHandler(void *CallBackRef);



extern u32 MMUTable;

/**
 * Assign the driver structures for the interrupt controller and
 * PL Core interrupt source
 */
INTC   IntcInstancePtr;
XPlIrq PlIrqInstancePtr;


/**
 * Modified DCacheFlush to prevent L2 Cache controller access
 */
void MyXil_DCacheFlush(void)
{
	Xil_L1DCacheFlush();
	//Xil_L2CacheFlush();
}

/**
 * Modified SetTlbAttributes to call MyXil_DCacheFlush in order
 * to prevelt L2 Cache controller accesses
 */
void MyXil_SetTlbAttributes(u32 addr, u32 attrib)
{
	u32 *ptr;
	u32 section;

	mtcp(XREG_CP15_INVAL_UTLB_UNLOCKED, 0);
	dsb();

	mtcp(XREG_CP15_INVAL_BRANCH_ARRAY, 0);
	dsb();
	MyXil_DCacheFlush();

	section = addr / 0x100000;
	ptr = &MMUTable + section;
	*ptr = (addr & 0xFFF00000) | attrib;
	dsb();
}

/*
 * Function to send data to OCM which is consumed by Linux Application
 */
void myPutchar(char c) {

	while(COMM_TX_FLAG);	//wait for cpu0 to consume previous value
	COMM_TX_DATA = (volatile unsigned long)c;
	dmb();
	COMM_TX_FLAG = 1;

}

/*
 * Since there is no STDIO assigned to the BSP, outbyte() is not created
 * by libgen. So add outbyte() call to myPutchar() to enable STDOUT functionality
 * for print() and xil_printf()
 */void outbyte(char c) {
	myPutchar(c);
}


int main()
{
	int Status;

	// Disable L1 cache for OCM
	MyXil_SetTlbAttributes(0xFFFF0000,0x04de2);           // S=b0 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0

	//heart beat in OCM for Linux to monitor
    COMM_VAL=0;

    // Initialize driver instance for PL IRQ
    PlIrqInstancePtr.Config.DeviceId = PL_IRQ_ID;
    PlIrqInstancePtr.Config.BaseAddress = IRQ_PCORE_GEN_BASE;
    PlIrqInstancePtr.IsReady = XIL_COMPONENT_IS_READY;
    PlIrqInstancePtr.IsStarted = 0;


	/*
	 * Connect the PL IRQ to the interrupt subsystem so that interrupts
	 * can occur
	 */
	Status = SetupIntrSystem(&IntcInstancePtr,
					&PlIrqInstancePtr,
					PL_IRQ_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// 1sec tick heartbeat visible from linux
    while(1) {
        COMM_VAL+=1;
        sleep(1);
    }


	/*
	 * Disable and disconnect the interrupt system
	 */
	DisableIntrSystem(&IntcInstancePtr, PL_IRQ_ID);



    return 0;
}


/*****************************************************************************/
/**
*
* This function setups the interrupt system such that PL interrupt can occur
* for the peripheral. This function is application specific since the actual
* system may or may not have an interrupt controller. The peripheral device could be
* directly connected to a processor without an interrupt controller. The
* user should modify this function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the instance of the Intc driver.
* @param	PeriphInstancePtr is a pointer to the instance of peripheral driver.
* @param	IntrId is the Interrupt Id of the peripheral interrupt
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
static int SetupIntrSystem(INTC *IntcInstancePtr,
		XPlIrq *PeriphInstancePtr,
				u16 IntrId)
{
	int Status;


	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, IntrId,
				 (Xil_ExceptionHandler)PlIntrHandler,
				 PeriphInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the PL device.
	 */
	XScuGic_Enable(IntcInstancePtr, IntrId);


	/*
	 * Initialize the  exception table
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)INTC_HANDLER,
			 IntcInstancePtr);

	/*
	 * Enable non-critical exceptions
	 */
	Xil_ExceptionEnable();


	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function is the Interrupt handler for the PL Interrupt.
* It is called when the PL creates an interrupt and the interrupt gets serviced.
*
* This function provides an example of how to handle WDT interrupt of the
* WdtTb device.
*
* @param	CallBackRef is a pointer to the callback function.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void PlIntrHandler(void *CallBackRef)
{
	static int count=0;
	XPlIrq *InstancePtr = (XPlIrq *)CallBackRef;

	/*
	 * Clear the interrupt source
	 */
	Xil_Out32(InstancePtr->Config.BaseAddress, 0);

	xil_printf("IRQ clr %d\n", count++);
}



/*****************************************************************************/
/**
*
* This function disables the interrupts that occur
*
* @param	IntcInstancePtr is the pointer to the instance of INTC driver.
* @param	IntrId is the Interrupt Id of the peripheral
*		value from xparameters.h.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void DisableIntrSystem(INTC *IntcInstancePtr, u16 IntrId)
{

	/*
	 * Disconnect and disable the interrupt
	 */
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disconnect(IntcInstancePtr, WdtTbIntrId);
#else
	/* Disconnect the interrupt */
	XScuGic_Disable(IntcInstancePtr, IntrId);
	XScuGic_Disconnect(IntcInstancePtr, IntrId);

#endif

}

