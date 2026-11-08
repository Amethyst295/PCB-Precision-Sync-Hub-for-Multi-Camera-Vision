#include <stdint.h>
extern uint32_t _estack,_sidata,_sdata,_edata,_sbss,_ebss;
int main(void); void SystemInit(void);
void Reset_Handler(void){uint32_t*s=&_sidata,*d=&_sdata;while(d<&_edata)*d++=*s++;for(d=&_sbss;d<&_ebss;)*d++=0;SystemInit();main();for(;;);}
void Default_Handler(void){for(;;);}
void TIM2_IRQHandler(void) __attribute__((weak,alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak,alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((weak,alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__((weak,alias("Default_Handler")));
#define D (uint32_t)Default_Handler
__attribute__((section(".isr_vector"),used)) const uint32_t vectors[16+82]={
 [0]=(uint32_t)&_estack,[1]=(uint32_t)Reset_Handler,[2 ... 15]=D,[16 ... 97]=D,
 [16+24]=(uint32_t)TIM1_BRK_TIM9_IRQHandler,[16+27]=(uint32_t)TIM1_CC_IRQHandler,[16+28]=(uint32_t)TIM2_IRQHandler,[16+43]=(uint32_t)TIM8_BRK_TIM12_IRQHandler};
void _exit(int c){(void)c;for(;;);} int _sbrk(int i){(void)i;return -1;}
