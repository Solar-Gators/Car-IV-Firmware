/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH                          *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2020  SEGGER Microcontroller GmbH                 *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       Current version number will be inserted here                 *
*       when shipment is built.                                      *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File        : FreeRTOSPlugin_ARM.js
Purpose     : Ozone FreeRTOS-awareness JavaScript Plugin for Legacy ARM
---------------------------END-OF-HEADER------------------------------
*/

/*********************************************************************
*
*       Local Functions
*
**********************************************************************
*/

/*********************************************************************
*
*       GetTaskStackGrowDir
*  
* Function description
*   Indicates if the task stack grows in positive (1) or negative (-1) memory direction
*
* Notes
*   (1) The stack grow direction depends on the FreeRTOS-port.
*       Currently, a positive grow direction is only used on the PIC architecture.
*/
function GetTaskStackGrowDir() {    
  return -1;
}

/*********************************************************************
*
*       GetTaskPrioName
*  
* Function description
*   Returns the display text of a task priority
*
* Parameters
*   Priority: task priority (integer)
*/
function GetTaskPrioName(Priority) {
    
  var sName;
  
  if (Priority == 0) {
    sName = "Idle"
  } else if (Priority == 1) {
    sName = "Low"
  } else if (Priority == 2) {
    sName = "Normal";
  } else if (Priority == 3) {
    sName = "High";
  } else if (Priority == 4) {
    sName = "Highest";
  } else {
    return Priority.toString();
  } 
  sName = sName + " (" + Priority + ")";
  return sName;
}

/*********************************************************************
*
*       GetTaskName
*  
* Function description
*   Returns the display text of a task name
*
* Parameters
*   tcb:   task control block (type tskTCB)
*   Addr:  control block memory location
*/
function GetTaskName(tcb, Addr) {
    
   var sTaskName;
   
   sTaskName = Debug.evaluate("(char*)(*(tskTCB*)" + Addr + ").pcTaskName");
   if (sTaskName == undefined) {
     sTaskName = Debug.evaluate("(char*)(*(TCB_t*)" + Addr + ").pcTaskName");
   }
   
   if (tcb.uxTCBNumber != undefined) {
     sTaskName = "#" + tcb.uxTCBNumber + " \"" + sTaskName + "\"";
   }
   return sTaskName;
}

/*********************************************************************
*
*       GetTaskID
*  
* Function description
*   Returns the display text of a task ID
*
* Parameters
*   tcb:   task control block (type tskTCB)
*   Addr:  control block memory location
*/
function GetTaskID(tcb, Addr) { 
   return  "0x" + (tcb.uxTCBNumber == undefined ? Addr.toString(16) : tcb.uxTCBNumber.toString(16));
}

/*********************************************************************
*
*       GetTCB
*  
* Function description
*   Returns the task control block of a task
*
* Parameters
*   Addr: control block memory location
*/
function GetTCB(Addr) { 
  var tcb;
  tcb = Debug.evaluate("*(tskTCB*)" + Addr);
  if (tcb == undefined) {
    tcb = Debug.evaluate("*(TCB_t*)" + Addr);
  }
  return tcb;
}

/*********************************************************************
*
*       GetTaskNotifyStateStr
*  
* Function description
*   Returns the display string of a task notify state
*
* Parameters
*   tcb: task control block (type tskTCB)
*/
function GetTaskNotifyStateStr(tcb) {

   if (tcb.ucNotifyState == undefined) {
     return tcb.eNotifyState == undefined ? "N/A" : tcb.eNotifyState.toString();
   } else {
     return tcb.ucNotifyState.toString()
   }
}

/*********************************************************************
*
*       GetTaskStackInfoStr
*  
* Function description
*   Returns a display text of the format "<free space> / <stack size>"
*
* Parameters
*   tcb: task control block (type tskTCB)
*
* Notes
*   (1) pxEndOfStack is only available when FreeRTOS was compiled with 
*       configRECORD_STACK_HIGH_ADDRESS == 1
*/
function GetTaskStackInfoStr(tcb) {
  var FreeSpace;
  var UsedSpace;
  var Size;
  //                                GrowDir == 0     GrowDir == 1
  //
  // pxStack       |  Low Addr   |  Stack Top     |  Stack Base    |
  //               |             |                |                |
  // pxTopOfStack  |             |  Stack Pointer |  Stack Pointer |       
  //               |             |                |                |
  // pxEndOfStack  |  High Addr  |  Stack Base    |  Stack Top     |
  // 
  if (GetTaskStackGrowDir() == 1) { // stack grows in positive address direction
  
    if (tcb.pxEndOfStack == undefined) {
      UsedSpace  =  tcb.pxTopOfStack - tcb.pxStack;
      FreeSpace  =  undefined;
      Size       =  undefined;  
    } else {
      FreeSpace  =  tcb.pxEndOfStack - tcb.pxTopOfStack;
      UsedSpace  =  tcb.pxTopOfStack - tcb.pxStack;
      Size       =  FreeSpace + UsedSpace;  
    }
  } else {    // stack grows in negative address direction
  
    if (tcb.pxEndOfStack == undefined) {
      FreeSpace  =  tcb.pxTopOfStack - tcb.pxStack;     
      UsedSpace  =  undefined;      
      Size       =  undefined;
    } else {
      FreeSpace  =  tcb.pxTopOfStack - tcb.pxStack;     
      UsedSpace  =  tcb.pxEndOfStack - tcb.pxTopOfStack;      
      Size       =  FreeSpace + UsedSpace;
    }   
  }   
  return FreeSpace + " / " + (Size == undefined ? "N/A" : Size);   
}

/*********************************************************************
*
*       AddTask
*  
* Function description
*   Adds a task to the task window
*
* Parameters
*   Addr:            memory location of the task's control block (TCB)
*   CurrTaskAddr:    memory location of the executing task's control block (TCB)
*   sState:          state of the task (e.g. "executing")
*/
function AddTask(Addr, CurrTaskAddr, sState) {
  var tcb;
  var sTaskName; 
  var sPriority;
  var sRunCnt;
  var sMutexCnt;
  var sTimeout;
  var sStackInfo;
  var sID;
  var sNotifiedValue;
  var sNotifyState; 

  tcb            = GetTCB(Addr);
  sTaskName      = GetTaskName(tcb, Addr);
  sID            = GetTaskID(tcb, Addr);
  sStackInfo     = GetTaskStackInfoStr(tcb);
  sPriority      = GetTaskPrioName(tcb.uxPriority);
  sNotifyState   = GetTaskNotifyStateStr(tcb);
  sTimeout       = (tcb.Timeout          == undefined ? "N/A" : tcb.Timeout.toString());
  sRunCnt        = (tcb.ulRunTimeCounter == undefined ? "N/A" : tcb.ulRunTimeCounter.toString());
  sNotifiedValue = (tcb.ulNotifiedValue  == undefined ? "N/A" : tcb.ulNotifiedValue.toString());
  sMutexCnt      = (tcb.uxMutexesHeld    == undefined ? "N/A" : tcb.uxMutexesHeld.toString());
 
  if (Addr == CurrTaskAddr) {
    sState = "executing";
  } 
  Threads.add(sTaskName, sRunCnt, sPriority, sState, sTimeout, sStackInfo, sID, sMutexCnt, sNotifiedValue, sNotifyState, (Addr == CurrTaskAddr) ? undefined : Addr);
}

/*********************************************************************
*
*       AddList
*  
* Function description
*   Adds all tasks of a task list to the task window
*
* Parameters
*   List:            task list (type xLIST)
*   CurrTaskAddr:    memory location of the executing task's control block (TCB)
*   sState:          common state of all tasks within 'list'
*/
function AddList(List, CurrTaskAddr, sState) {
  var i;
  var Index;
  var Item;
  var TaskAddr;

  if ((List != undefined) && (List.uxNumberOfItems > 0)) {
      
    Index = List.xListEnd.pxNext;
    
    for (i = 0; i < List.uxNumberOfItems; i++) {
        
      Item = Debug.evaluate("*(xLIST_ITEM*)" + Index);

      TaskAddr = Item != 0 ? Item.pvOwner : 0;

      if (TaskAddr != 0) {
        AddTask(TaskAddr, CurrTaskAddr, sState);
      }
      Index = Item.pxNext;

      if (i > 1000) { // infinite loop guard
        break;
      }
    }
  }
}

/*********************************************************************
*
*       API Functions
*
**********************************************************************
*/

/*********************************************************************
*
*       init
*  
* Function description
*   Initializes the task window
*/
function init() {
    
  Threads.clear();
  Threads.newqueue("Task List");
  Threads.setColumns("Name", "Run Count", "Priority", "Status", "Timeout", "Stack Info", "ID", "Mutex Count", "Notified Value", "Notify State");
  Threads.setSortByNumber("Priority");
  Threads.setSortByNumber("Timeout");
  Threads.setSortByNumber("Run Count");
  Threads.setSortByNumber("ID");
  Threads.setSortByNumber("Mutex Count");
  Threads.setSortByNumber("Notified Value");
  Threads.setSortByNumber("Notify State");
  Threads.setColor("Status", "ready", "executing", "blocked");
}

/*********************************************************************
*
*       update
*  
* Function description
*   Updates the task window
*/
function update() {
  var i;
  var pList;
  var List;
  var MaxPriority;
  var CurrTaskAddr;

  Threads.clear();

  if((Debug.evaluate("pxCurrentTCB") == 0) || (Debug.evaluate("pxCurrentTCB") == undefined)) {
    return;
  }
  MaxPriority  = Debug.evaluate("uxTopReadyPriority");  
  CurrTaskAddr = Debug.evaluate("pxCurrentTCB");

  for (i = MaxPriority; i >= 0; i--) {
    List = Debug.evaluate("pxReadyTasksLists[" + i + "]");
    AddList(List, CurrTaskAddr, "ready");
  }

  pList = Debug.evaluate("pxDelayedTaskList");
  if (pList != 0) {
    List = Debug.evaluate("*(xLIST*)" + pList);
    AddList(List, CurrTaskAddr, "blocked");
  }

  pList = Debug.evaluate("pxOverflowDelayedTaskList");
  if (pList != 0) {
    List = Debug.evaluate("*(xLIST*)" + pList);
    AddList(List, CurrTaskAddr, "blocked");
  }

  List = Debug.evaluate("xSuspendedTaskList");
  if (List != 0) {
    AddList(List, CurrTaskAddr, "suspended");
  } 
}

/*********************************************************************
*
*       getregs
*
* Function description
*   Returns the register set of a task.
*   For ARM cores, this function is expected to return the values
*   of registers R0 to R15 and PSR.
*
* Parameters
*   hTask: integer number identifiying the task.
*   Identical to the last parameter supplied to method Threads.add.
*   For convenience, this should be the address of the TCB.
*
* Return Values
*   An array of unsigned integers containing the task’s register values.
*   The array must be sorted according to the logical indexes of the regs.
*   The logical register indexing scheme is defined by the ELF-DWARF ABI.
*
**********************************************************************
*/
function getregs(hTask) { 
  var i;
  var SP;
  var LR;
  var Addr;
  var tcb;
  var aRegs = new Array(17);

  tcb  =  GetTCB(hTask);
  SP   =  tcb.pxTopOfStack;
  Addr =  SP;
  //
  // CPSR
  //
  aRegs[16] = TargetInterface.peekWord(Addr); 
  Addr += 4;
  //
  // R0...R13(pxOriginalTOS)
  //
  for (i = 0; i < 14; i++) {
    aRegs[i] = TargetInterface.peekWord(Addr); 
    Addr += 4;
  }
  //
  // LR
  //
  aRegs[14] = TargetInterface.peekWord(Addr);  
  Addr += 4;
  //
  // PC
  //
  aRegs[15] = TargetInterface.peekWord(Addr); 
  Addr += 4;
  //
  // SP
  //
  aRegs[13] = Addr;
  
  return aRegs;
}

/*********************************************************************
*
*       getContextSwitchAddrs
*
*  Functions description
*    Returns an unsigned integer array containing the base addresses 
*    of all functions that complete a task switch when executed.
*/
function getContextSwitchAddrs() {
  
  var aAddrs;
  var Addr;
  
  Addr = Debug.evaluate("&vTaskSwitchContext");
  
  if (Addr != undefined) {
    aAddrs = new Array(1);
    aAddrs[0] = Addr;
    return aAddrs;
  } else {
    return [];
  }
}

/*********************************************************************
*
*       getOSName()
*
*  Functions description:
*    Returns the name of the RTOS this script supplies support for
*/
function getOSName() {
  return "FreeRTOS";
}
