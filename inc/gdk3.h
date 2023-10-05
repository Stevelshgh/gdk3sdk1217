/*
沁恒的头文件包含规则极其混乱，丑陋之极，GDK3特别将非常基础没有依赖的一些基本类型
定义在这个头文件中
包含这个文件永远不会产生关联依赖
*/
#ifndef _GD_GDK3_H_
#define _GD_GDK3_H_
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/** 
  * @brief  HAL Lock structures definition  
  */
typedef enum 
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;

#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
  #ifndef __packed
    #define __packed __attribute__((__packed__))
  #endif /* __packed */
#endif /* __GNUC__ */


/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined   (__GNUC__)        /* GNU Compiler */
  #ifndef __ALIGN_END
    #define __ALIGN_END    __attribute__ ((aligned (4)))
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN  
    #define __ALIGN_BEGIN
  #endif /* __ALIGN_BEGIN */
#else
  #ifndef __ALIGN_END
    #define __ALIGN_END
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN      
    #if defined   (__CC_ARM)      /* ARM Compiler */
      #define __ALIGN_BEGIN    __align(4)  
    #elif defined (__ICCARM__)    /* IAR Compiler */
      #define __ALIGN_BEGIN 
    #endif /* __CC_ARM */
  #endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

#endif // _GD_GDK3_H_