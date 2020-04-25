/*            
Trabalho de Conclusão de curso: Som eletrônico de escapamento
Acadêmico Cássio Augusto Rosso
     |                                   |                                  |                                    |    
     |             DAC 8531              |                DAC8411           |             MCP41010               |
     |    1.VDD  -         -  8.GND      |      1.SYNC  -        - 6.VOUT   |   1.CS    -         - 8.VDD(5V)    |
     |    2.VREF -         -  7.DIN      |      2.CLOCK -        - 5.GND    |   2.CLOCK -         - 7.PB0        | 
     |    3.VFB  -         -  6.CLOCK    |      3.DIN   -        - 4.VDD    |   3.DIN   -         - 6.PW0        |
     |    4.OUT  -         -  5.SYNC     |                                  |   4.VSS   -         - 5.PA0        |
     |                                   |                                  |                                    |
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "math.h"
#include "shift.h"
#include "vetor_e30_1793.h"
#include "vetor_lancer_987.h"
#include "vetor_mr2_1632.h"
#include "vetor_k20_1925.h"
#include "car_starting.h"
#include "vetor_shelby_1636.h"

//  DECLARAÇÕES VARIAVEIS GLOBAIS
uint32_t ADC1Value[1];                                           // Variável de captura da informação no canal A/D
unsigned char tempo=23,som=4,volume;
char equalizador='n';
volatile uint32_t millis=0;
uint16_t rpm=1000,rpm_ad;
uint16_t i=0,amostra_filtrada;
const float fir_lowpass[21]={0.0074,0.0094,0.0154,0.0248,0.0366,0.0496,0.0627,0.0745,0.0838,0.0898,0.0919,0.0898,0.0838,0.0745,0.0627,0.0496,0.0366,0.0248,0.0154,0.0094,0.0074}; // Coeficientes FIR passa-baixo
const float fir_highpass[21]={-0.0025,-0.0036,-0.0066,-0.0115,-0.0183,-0.0264,-0.0349,-0.0431,-0.0497,-0.0541,0.9467,-0.0541,-0.0497,-0.0431,-0.0349,-0.0264,-0.0183,-0.0115,-0.0066,-0.0036,-0.0025};        // Coeficientes FIR passa-altas
const float fir_highpass2[11]={-0.0039,-0.0086,-0.0210,-0.0370,-0.0502,0.9408,-0.0502,-0.0370,-0.0210,-0.0086,-0.0039};

//  DEFINIÇÕES UART
#define GPIO_PB1_U1TX 0x00010401
#define GPIO_PB0_U1RX 0x00010001

//  DEFINIÇÕES SSI
#define GPIO_PA2_SSI0CLK 0x00000802 // PA2 como CLOCK
#define GPIO_PA5_SSI0TX  0x00001402 // PA5 como DATA
#define GPIO_PD0_SSI3CLK 0x00030001 // PD0 como CLOCK
#define GPIO_PD1_SSI3FSS 0x00030401 // PD1 como SYNC
#define GPIO_PD3_SSI3TX  0x00030C01 // PD3 como DATA

//  CONFIGURAÇÃO SYSTICK - TIMER MICRO-PROCESSADOR
void SycTickInt(){
  millis++;
}
void Init_SysTickbegin(){
  SysTickPeriodSet(80);           // 80 * 12,5ns(1/80MHz) = 1 micro-segundo
  SysTickIntRegister(SycTickInt);
  SysTickIntEnable();
  SysTickEnable();
}
void Wait(uint32_t time){
	uint32_t temp = millis;
	while( (millis-temp) < time){
	}	
}

//  CONFIGURAÇÃO TIMER0 - TIMER 'A' E 'B' JUNTOS PARA OPERAÇÃO EM 32 BITS GERANDO O DISPARO DO CANAL A/D EM UM TEMPO PRE-DETERMINADO
void Init_Timer0 (void){
SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);    // Timer recarrega-se utomaticamente
TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 80000);      // t(ms) = valor * 12,5ns(clock)
TimerControlTrigger(TIMER0_BASE, TIMER_BOTH, true); // Timer habilitado para disparar canal A/D
TimerEnable(TIMER0_BASE, TIMER_BOTH);               // Habilitação e junção de Timers A e B para 32 bits 
}

//  CONFIGURAÇÕES UART
void Init_UART0 (void) {
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
GPIOPinConfigure(GPIO_PA0_U0RX);
GPIOPinConfigure(GPIO_PA1_U0TX);
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
UARTStdioConfig(0, 460800, 80000000); }

void UART1_IntHandler(void)
{
char x; 
x=UARTCharGet(UART1_BASE);
         switch(x)
            {
            case '0': som = 0;             
            break;
            case '1': som = 1;             
            break;
            case '2': som = 2;
            break;
            case '3': som = 3;             
            break;
            case '4': som = 4;
            break;
            case '5': som = 5;
            break;
            case 'n': equalizador = 'n';
            break;
            case 'a': equalizador = 'a';
            break;
            case 'g': equalizador = 'g';
            break;
            }
UARTIntClear(UART1_BASE,UART_INT_RT);
}

// CONFIGURAÇÕES UART1 - BLUETOOTH
void Init_UART1 (void) 
{
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
GPIOPinConfigure(GPIO_PB0_U1RX);
GPIOPinConfigure(GPIO_PB1_U1TX);
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
UARTStdioConfig(1, 921600, 80000000);
UARTIntEnable(UART1_BASE,UART_INT_RT);
IntEnable(INT_UART1_TM4C123);
IntRegister(INT_UART1_TM4C123,UART1_IntHandler);
}

//  CONFIGURAÇÕES SPI (COMUNICAÇÃO SERIAL COM DAC)
//  A BIBLIOTECA TIVAWARE PERMITE A COMUNICAÇÃO DE NO MÁXIMO ATÉ 16 BITS DE DATA, COMO O DAC8431 POSSUI UM REGISTRADOR INTERNO
//  DE 24 BITS É NECESSÁRIO CONFIGURAR A SPI COM DADOS DE 8 BITS PARA ENVIO DE 3 PACOTES (3*8=24 BITS) E O PINO DE FSS DEVE SER CONFIGURADO 
//  MANUALMENTE EM UM PINO GPIO PARA CORRETO SINCRONISMO
void Init_SSI0 (void){
SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);                                                           // RESETA MÓDULO SSI0
SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);                                                          // HABILITA MÓDULO SSI0                                             
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                                         // HABILITA PORT-A
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));                                                   // ESPERA MÓDULO ESTAR PRONTO
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));                                                  // ESPERA MÓDULO ESTAR PRONTO
GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);	                                             // PA3 CONFIGURADO MANUALMENTE(GPIO) COMO SAÍDA PARA SINCRONISMO DE SPI
GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);                                               // ESCRITA PA3 (PINO DE SINCRONISMO)
GPIOPinConfigure(GPIO_PA2_SSI0CLK);                                                                  // PA2 como CLOCK                                                              // PA3 como SYNC
GPIOPinConfigure(GPIO_PA5_SSI0TX);                                                                   // PA5 como DATA
GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5);                                            // Definições dos pinos como SSI
SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 40000000, 8);  // DAC com 40MHz, 16 bits
SSIEnable(SSI0_BASE);                                                                                // Habilita SPI
}

//  CONFIGURAÇÃO SPI PARA POTENCIOMETRO DIGITAL MCP41010
void Init_SSI3 (void)
{
SysCtlPeripheralReset(SYSCTL_PERIPH_SSI3);                                                           // RESETA MÓDULO SSI0
SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);                                                          // HABILITA MÓDULO SSI0                                             
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                                                         // HABILITA PORT-A
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));                                                   // ESPERA MÓDULO ESTAR PRONTO
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));                                                  // ESPERA MÓDULO ESTAR PRONTO
GPIOPinConfigure(GPIO_PD0_SSI3CLK);                                                                  // PD0 como CLOCK                                                              
GPIOPinConfigure(GPIO_PD1_SSI3FSS);                                                                  // PD1 como SYNC
GPIOPinConfigure(GPIO_PD3_SSI3TX);                                                                   // PD3 como DATA
GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);                               // Definições dos pinos como SSI
SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 10000000, 16);  // POT. com 10MHz, 8 bits
SSIEnable(SSI3_BASE);                                                                                // Habilita SPI
}

//  QUEBRA DE DADOS DE 16 BITS PARA CONJUNTOS DE 8 BITS NECESSÁRIOS PARA A COMUNICAÇÃO COM DAC8431 
//  POIS SEU REGISTRO INTERNO POSSUI 24 BITS E A BIBLIOTECA TIVAWARE COMUNICA-SE ATÉ 16 BITS NECESSITANDO UMA MANIPULAÇÃO DE BITS 
unsigned char byte2_8551(uint16_t data)
{
  unsigned char LSB = (data & 255);
  return LSB;
}
unsigned char byte1_8551(uint16_t data)
{
  unsigned char MSB = ((data >> 8) & 255);
  return MSB;
}
unsigned char byte1_8411 (uint16_t data)
{
  unsigned char byte1 = ((data >> 10) & 255);
  return byte1;
}
unsigned char byte2_8411 (uint16_t data)
{
  unsigned char byte2 = ((data >> 2) & 255);
  return byte2;
}
unsigned char byte3_8411 (uint16_t data)
{
  unsigned char byte3 = ((data << 6)  & 255);
  return byte3;
}

//  ENVIO DE DADOS PARA DAC8411 E DAC8551
void Send_To_DAC8411 (uint16_t data, unsigned char tempo_entre_amostras)
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0);
    SSIDataPut(SSI0_BASE, (byte1_8411(data)));
    SSIDataPut(SSI0_BASE, (byte2_8411(data)));
    SSIDataPut(SSI0_BASE, (byte3_8411(data)));
    while(SSIBusy(SSI0_BASE));
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    Wait(tempo_entre_amostras);
}
void Send_To_DAC8551 (uint16_t data, unsigned char tempo_entre_amostras)
{
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0);
    SSIDataPut(SSI0_BASE, 0x00);
    SSIDataPut(SSI0_BASE, (byte1_8551(data)));
    SSIDataPut(SSI0_BASE, (byte2_8551(data)));
    while(SSIBusy(SSI0_BASE));
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    Wait(tempo_entre_amostras);
}

//  REPRODUÇÃO DE SONS DE MOTORES DE AUTOMÓVEIS

void tempo_e30_1793rpm(void)
{
tempo = ((-0.0039*rpm) + 37.3);
}
void tempo_lancer_987(void)
{
tempo = ((-0.0027*rpm) + 31.4);
}
void tempo_mr2_1632rpm(void)
{
tempo = ((-0.0036*rpm) + 38.71);
}
void tempo_k20_1925(void)
{
tempo = ((-0.003*rpm) + 34.86);
}
void tempo_shelby_1636(void)
{
tempo = ((-0.0026*rpm) + 33.017);
}

//  REPRODUÇÃO SONS

void play_e30_1793rpm(void)
{
  switch(equalizador){
  case 'n':
    for(i = 0; i < 1319; i++)
    {
    Send_To_DAC8411(e30[i],tempo);
    } 
    break;
  case 'g':
    for(i = 20; i < 1319; i++)
{
amostra_filtrada = (fir_lowpass[0]*e30[i] + fir_lowpass[1]*e30[i-1] + fir_lowpass[2]*e30[i-2]+ fir_lowpass[3]*e30[i-3] + fir_lowpass[4]*e30[i-4] + fir_lowpass[5]*e30[i-5] + fir_lowpass[6]*e30[i-6] + fir_lowpass[7]*e30[i-7] + fir_lowpass[8]*e30[i-8] + fir_lowpass[9]*e30[i-9] + fir_lowpass[10]*e30[i-10] + fir_lowpass[11]*e30[i-11] + fir_lowpass[12]*e30[i-12] + fir_lowpass[13]*e30[i-13] + fir_lowpass[14]*e30[i-14] + fir_lowpass[15]*e30[i-15] + fir_lowpass[16]*e30[i-16] + fir_lowpass[17]*e30[i-17] + fir_lowpass[18]*e30[i-18] + fir_lowpass[19]*e30[i-19] + fir_lowpass[20]*e30[i-20]);
Send_To_DAC8411(amostra_filtrada,(tempo-10));
}
    break;
  case 'a':
for(i = 20; i < 1319; i++)
    {
    amostra_filtrada = (fir_highpass[0]*e30[i] + fir_highpass[1]*e30[i-1] + fir_highpass[2]*e30[i-2]+ fir_highpass[3]*e30[i-4] + fir_highpass[4]*e30[i-4] + fir_highpass[5]*e30[i-5] + fir_highpass[6]*e30[i-6] + fir_highpass[7]*e30[i-7] + fir_highpass[8]*e30[i-8] + fir_highpass[9]*e30[i-9] + fir_highpass[10]*e30[i-10] + fir_highpass[11]*e30[i-11] + fir_highpass[12]*e30[i-12] + fir_highpass[13]*e30[i-13] + fir_highpass[14]*e30[i-14] + fir_highpass[15]*e30[i-15] + fir_highpass[16]*e30[i-16] + fir_highpass[17]*e30[i-17] + fir_highpass[18]*e30[i-18] + fir_highpass[19]*e30[i-19] + fir_highpass[20]*e30[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-10));
    } 
    break;
 }}

void play_lancer_987(void)
{
  switch(equalizador){
  case 'n':
    for(i = 0; i < 3233; i++)
    {
    Send_To_DAC8411(lancer_987[i],tempo);
    }
    break;
  case 'g':
    for(i = 20; i < 3233; i++)
    {
    amostra_filtrada = (fir_lowpass[0]*lancer_987[i] + fir_lowpass[1]*lancer_987[i-1] + fir_lowpass[2]*lancer_987[i-2]+ fir_lowpass[3]*lancer_987[i-3] + fir_lowpass[4]*lancer_987[i-4] + fir_lowpass[5]*lancer_987[i-5] + fir_lowpass[6]*lancer_987[i-6] + fir_lowpass[7]*lancer_987[i-7] + fir_lowpass[8]*lancer_987[i-8] + fir_lowpass[9]*lancer_987[i-9] + fir_lowpass[10]*lancer_987[i-10] + fir_lowpass[11]*lancer_987[i-11] + fir_lowpass[12]*lancer_987[i-12] + fir_lowpass[13]*lancer_987[i-13] + fir_lowpass[14]*lancer_987[i-14] + fir_lowpass[15]*lancer_987[i-15] + fir_lowpass[16]*lancer_987[i-16] + fir_lowpass[17]*lancer_987[i-17] + fir_lowpass[18]*lancer_987[i-18] + fir_lowpass[19]*lancer_987[i-19] + fir_lowpass[20]*lancer_987[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    }
    break;
  case 'a':
    for(i = 20; i < 3233; i++)
    {
    amostra_filtrada = (fir_highpass[0]*lancer_987[i] + fir_highpass[1]*lancer_987[i-1] + fir_highpass[2]*lancer_987[i-2]+ fir_highpass[3]*lancer_987[i-4] + fir_highpass[4]*lancer_987[i-4] + fir_highpass[5]*lancer_987[i-5] + fir_highpass[6]*lancer_987[i-6] + fir_highpass[7]*lancer_987[i-7] + fir_highpass[8]*lancer_987[i-8] + fir_highpass[9]*lancer_987[i-9] + fir_highpass[10]*lancer_987[i-10] + fir_highpass[11]*lancer_987[i-11] + fir_highpass[12]*lancer_987[i-12] + fir_highpass[13]*lancer_987[i-13] + fir_highpass[14]*lancer_987[i-14] + fir_highpass[15]*lancer_987[i-15] + fir_highpass[16]*lancer_987[i-16] + fir_highpass[17]*lancer_987[i-17] + fir_highpass[18]*lancer_987[i-18] + fir_highpass[19]*lancer_987[i-19] + fir_highpass[20]*lancer_987[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    } 
    break;
  }}

void play_mr2_1632rpm(void)
{
  switch(equalizador){
   case 'n':
    for(i = 0; i < 1407; i++)
    {
    Send_To_DAC8411(mr2_1632[i],tempo);
    }
    break;
   case 'g':
    for(i = 20; i < 1407; i++)
    {
    amostra_filtrada = (fir_lowpass[0]*mr2_1632[i] + fir_lowpass[1]*mr2_1632[i-1] + fir_lowpass[2]*mr2_1632[i-2]+ fir_lowpass[3]*mr2_1632[i-3] + fir_lowpass[4]*mr2_1632[i-4] + fir_lowpass[5]*mr2_1632[i-5] + fir_lowpass[6]*mr2_1632[i-6] + fir_lowpass[7]*mr2_1632[i-7] + fir_lowpass[8]*mr2_1632[i-8] + fir_lowpass[9]*mr2_1632[i-9] + fir_lowpass[10]*mr2_1632[i-10] + fir_lowpass[11]*mr2_1632[i-11] + fir_lowpass[12]*mr2_1632[i-12] + fir_lowpass[13]*mr2_1632[i-13] + fir_lowpass[14]*mr2_1632[i-14] + fir_lowpass[15]*mr2_1632[i-15] + fir_lowpass[16]*mr2_1632[i-16] + fir_lowpass[17]*mr2_1632[i-17] + fir_lowpass[18]*mr2_1632[i-18] + fir_lowpass[19]*mr2_1632[i-19] + fir_lowpass[20]*mr2_1632[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    }
    break;
  case 'a':
    for(i = 10; i < 1407; i++)
    {
    amostra_filtrada = (fir_highpass2[0]*mr2_1632[i] + fir_highpass2[1]*mr2_1632[i-1] + fir_highpass2[2]*mr2_1632[i-2]+ fir_highpass2[3]*mr2_1632[i-4] + fir_highpass2[4]*mr2_1632[i-4] + fir_highpass2[5]*mr2_1632[i-5] + fir_highpass2[6]*mr2_1632[i-6] + fir_highpass2[7]*mr2_1632[i-7] + fir_highpass2[8]*mr2_1632[i-8] + fir_highpass2[9]*mr2_1632[i-9] + fir_highpass2[10]*mr2_1632[i-10]); 
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    } 
    break;
  }}
void play_k20_1925(void)
{
  switch(equalizador){
   case 'n':
    for(i = 0; i < 2468; i++)
    {
    Send_To_DAC8411(k20_1925[i],tempo);
    }       
    break;
   case 'g':
    for(i = 20; i < 2468; i++)
    {
    amostra_filtrada = (fir_lowpass[0]*k20_1925[i] + fir_lowpass[1]*k20_1925[i-1] + fir_lowpass[2]*k20_1925[i-2]+ fir_lowpass[3]*k20_1925[i-3] + fir_lowpass[4]*k20_1925[i-4] + fir_lowpass[5]*k20_1925[i-5] + fir_lowpass[6]*k20_1925[i-6] + fir_lowpass[7]*k20_1925[i-7] + fir_lowpass[8]*k20_1925[i-8] + fir_lowpass[9]*k20_1925[i-9] + fir_lowpass[10]*k20_1925[i-10] + fir_lowpass[11]*k20_1925[i-11] + fir_lowpass[12]*k20_1925[i-12] + fir_lowpass[13]*k20_1925[i-13] + fir_lowpass[14]*k20_1925[i-14] + fir_lowpass[15]*k20_1925[i-15] + fir_lowpass[16]*k20_1925[i-16] + fir_lowpass[17]*k20_1925[i-17] + fir_lowpass[18]*k20_1925[i-18] + fir_lowpass[19]*k20_1925[i-19] + fir_lowpass[20]*k20_1925[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    }
    break;
   case 'a':
    for(i = 20; i < 2468; i++)
    {
    amostra_filtrada = (fir_highpass[0]*k20_1925[i] + fir_highpass[1]*k20_1925[i-1] + fir_highpass[2]*k20_1925[i-2]+ fir_highpass[3]*k20_1925[i-4] + fir_highpass[4]*k20_1925[i-4] + fir_highpass[5]*k20_1925[i-5] + fir_highpass[6]*k20_1925[i-6] + fir_highpass[7]*k20_1925[i-7] + fir_highpass[8]*k20_1925[i-8] + fir_highpass[9]*k20_1925[i-9] + fir_highpass[10]*k20_1925[i-10] + fir_highpass[11]*k20_1925[i-11] + fir_highpass[12]*k20_1925[i-12] + fir_highpass[13]*k20_1925[i-13] + fir_highpass[14]*k20_1925[i-14] + fir_highpass[15]*k20_1925[i-15] + fir_highpass[16]*k20_1925[i-16] + fir_highpass[17]*k20_1925[i-17] + fir_highpass[18]*k20_1925[i-18] + fir_highpass[19]*k20_1925[i-19] + fir_highpass[20]*k20_1925[i-20]);
    Send_To_DAC8411(amostra_filtrada,(tempo-4));
    } 
    break;
  }}
void play_shelby_1636(void)
{
  switch(equalizador){
   case 'n':
    for(i = 0; i < 5755; i++)
    {
    Send_To_DAC8411(shelby_1636[i],tempo);
    }
    break;
  case 'g':
   for(i = 20; i < 5755; i++)
   {
   amostra_filtrada = (fir_lowpass[0]*shelby_1636[i] + fir_lowpass[1]*shelby_1636[i-1] + fir_lowpass[2]*shelby_1636[i-2]+ fir_lowpass[3]*shelby_1636[i-3] + fir_lowpass[4]*shelby_1636[i-4] + fir_lowpass[5]*shelby_1636[i-5] + fir_lowpass[6]*shelby_1636[i-6] + fir_lowpass[7]*shelby_1636[i-7] + fir_lowpass[8]*shelby_1636[i-8] + fir_lowpass[9]*shelby_1636[i-9] + fir_lowpass[10]*shelby_1636[i-10] + fir_lowpass[11]*shelby_1636[i-11] + fir_lowpass[12]*shelby_1636[i-12] + fir_lowpass[13]*shelby_1636[i-13] + fir_lowpass[14]*shelby_1636[i-14] + fir_lowpass[15]*shelby_1636[i-15] + fir_lowpass[16]*shelby_1636[i-16]+ fir_lowpass[17]*shelby_1636[i-17] + fir_lowpass[18]*shelby_1636[i-18] + fir_lowpass[19]*shelby_1636[i-19] + fir_lowpass[20]*shelby_1636[i-20]);
   Send_To_DAC8411(amostra_filtrada,(tempo-4));
   }
   break;
  case 'a':
   for(i = 20; i < 5755; i++)
   {
   amostra_filtrada = (fir_highpass[0]*shelby_1636[i] + fir_highpass[1]*shelby_1636[i-1] + fir_highpass[2]*shelby_1636[i-2]+ fir_highpass[3]*shelby_1636[i-4] + fir_highpass[4]*shelby_1636[i-4] + fir_highpass[5]*shelby_1636[i-5] + fir_highpass[6]*shelby_1636[i-6] + fir_highpass[7]*shelby_1636[i-7] + fir_highpass[8]*shelby_1636[i-8] + fir_highpass[9]*shelby_1636[i-9] + fir_highpass[10]*shelby_1636[i-10] + fir_highpass[11]*shelby_1636[i-11] + fir_highpass[12]*shelby_1636[i-12] + fir_highpass[13]*shelby_1636[i-13] + fir_highpass[14]*shelby_1636[i-14] + fir_highpass[15]*shelby_1636[i-15] + fir_highpass[16]*shelby_1636[i-16] + fir_highpass[17]*shelby_1636[i-17] + fir_highpass[18]*shelby_1636[i-18] + fir_highpass[19]*shelby_1636[i-19] + fir_highpass[20]*shelby_1636[i-20]);
   Send_To_DAC8411(amostra_filtrada,(tempo-4));
   } 
   break; 
  }}

//  TRATAMENTO DE INTERRUPÇÃO DO CANAL A/D PARA CAPTURA DE TENSÃO NO PEDAL DE ACELERAÇÃO
void ADC1_S2_IntHandler (void)
{
ADCIntClear(ADC1_BASE, 2);
ADCSequenceDataGet(ADC1_BASE, 2, ADC1Value);     // Captura valor no A/D
rpm_ad=ADC1Value[0]*5000/4095 + 1000;            // RPM de 1000 a 6000

if(rpm<rpm_ad)
{
  rpm=rpm+3;
}
else if(rpm>rpm_ad)
{
  rpm=rpm-2;
}
volume = (0.025*rpm) + 105;
SSIDataPut(SSI3_BASE, (volume+4352));

  switch(som)
    {
    case 0: break;
    case 1: tempo_mr2_1632rpm (); break;
    case 2: tempo_k20_1925    (); break; 
    case 3: tempo_e30_1793rpm (); break;
    case 4: tempo_shelby_1636 (); break;
    case 5: tempo_lancer_987  (); break;
    default: break;
    }
}

// CONFIGURAÇÃO DO CANAL A/D  ADC1 PARA OBTENÇÃO DA TENSÃO NO PEDAL DE ACELERAÇÃO
void Init_AD (void){
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);                                        // PB4 A/D
ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_TIMER, 0);                           // Disparo do A/D por Timer
ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH10 | ADC_CTL_IE | ADC_CTL_END); // ADC1, Sequenciador 2, com interrupção
ADCHardwareOversampleConfigure(ADC1_BASE, 4); 
IntEnable(INT_ADC1SS2_TM4C123);
ADCIntEnable(ADC1_BASE, 2);
IntRegister(INT_ADC1SS2_TM4C123,ADC1_S2_IntHandler);

}

// FUNÇÃO MAIN
void main(void)
{
SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); // Clock microprocessador [200MHz/2,5 = 80MHz]

/*SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
IntEnable(INT_GPIOF_TM4C123);
IntRegister(INT_GPIOF_TM4C123, GPIO_F_IntHandler);*/

// Inicialização de periféricos
Init_UART1();
Init_Timer0();
Init_AD();
Init_SSI0();
Init_SSI3();
Init_SysTickbegin();

// Som inicial de ignição do carro
for(i = 0; i < 58319; i++)
 {
  Send_To_DAC8411(inicio[i],21);
 }

ADCSequenceEnable(ADC1_BASE, 2);  // Habilitação de módulo A/D

while(true)
{
  switch(som)
  {
  case 0 : break;
  case 1 : play_mr2_1632rpm (); break;
  case 2 : play_k20_1925    (); break;
  case 3 : play_e30_1793rpm (); break;
  case 4 : play_shelby_1636 (); break;
  case 5 : play_lancer_987  (); break;
  default: break;
  }
}
}

