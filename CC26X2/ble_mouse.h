#ifndef BLE_MOUSE_H
#define BLE_MOUSE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/PIN.h>

#include <bcomdef.h>

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Project Zero.
 */
extern void ProjectZero_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
