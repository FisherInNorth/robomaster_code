# 2022赛季步兵电控代码：云台部分开发者日志
- 2021.12.30 修改人：罗俊宇 
    1. 修改了AHRSMahony算法中采样率，从1000修改到源ins_task工程中的1000，同时修改算法中的PID参数至原参数
    2. 修改了INS_task函数中gyro_update_flag判断条件为(1<< IMU_NOTIFY_SHFITS)，原来是IMU_UPDATE_SHFITS
    3. 增加了了DMA2中断最后一段if()语句中的内容，原本是空的
- 2021.12.31 修改人：罗俊宇
    1. 修改了温控CCR1比较值大于预装载值的bug
    2. 修改了温控pwm的输出定时器tim10的cnt一直不累加的问题，原因是在初始化程序中缺少HAL_TIM_Base_Start(&htim10);导致定时器没有开启
