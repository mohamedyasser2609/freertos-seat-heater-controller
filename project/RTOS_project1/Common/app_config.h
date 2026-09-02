#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Task Priorities */
#define TASK_PRIORITY_SAFETY          3  /* Reduced from 4 to prevent priority inversion */
#define TASK_PRIORITY_POWER_MGMT      2
#define TASK_PRIORITY_DIAGNOSTIC      2
#define TASK_PRIORITY_SEAT_CONTROL    2
#define TASK_PRIORITY_STATISTICS      1
#define TASK_PRIORITY_DISPLAY         1  /* Lowest priority */

/* Task Stack Sizes - Optimized based on actual usage */
#define TASK_STACK_SIZE_SAFETY         192  /* Safety critical but optimized */
#define TASK_STACK_SIZE_POWER_MGMT     96   /* Simple state management */
#define TASK_STACK_SIZE_DIAGNOSTIC     128  /* Reduced as event handling is simple */
#define TASK_STACK_SIZE_SEAT_CONTROL   128  /* Optimized for ADC and control */
#define TASK_STACK_SIZE_STATISTICS     96   /* Simple data collection */
#define TASK_STACK_SIZE_DISPLAY        384  /* Increased to prevent stack overflow */

#endif /* APP_CONFIG_H */

