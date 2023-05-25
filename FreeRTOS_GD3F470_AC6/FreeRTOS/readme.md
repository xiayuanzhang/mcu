使用FreeRTOS主要设置3个东西

1. FreeRTOSConfig.h

参考官网的,或者找其他工程的这个文件进行修改

2. heap.c

FreeRTOS\portable\MemMang 

3. port.c和portmacro.h

FreeRTOS\portable\   中根据编译器和处理器型号选择