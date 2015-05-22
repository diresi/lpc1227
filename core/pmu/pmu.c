/**************************************************************************/
/*!
    @file     pmu.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Controls the power management features of the LPC1114, allowing you
    to enter sleep/deep-sleep or deep power-down mode.

    For examples of how to enter either mode, see the comments for the
    functions pmuSleep(), pmuDeepSleep() and pmuPowerDown().
	
    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "pmu.h"
#include "core/gpio/gpio.h"
#include "core/cpu/cpu.h"
#include "core/timer32/timer32.h"

#ifdef CFG_CHIBI
  #include "drivers/rf/chibi/chb_drvr.h"
#endif

#ifdef CFG_SDCARD
  #include "core/ssp/ssp.h"
#endif

#define PMU_WDTCLOCKSPEED_HZ  7812

// Pointer for ROM access and power profiles
#if CFG_PMU_USEPOWERPROFILES == 1
  static unsigned int command[5], result[5];
  const ROM ** rom = (const ROM **) 0x1FFF1FF8;
#endif

void pmuSetupHW(void);
void pmuRestoreHW(void);

/**************************************************************************/
/*!
    Wakeup interrupt handler
*/
/**************************************************************************/
void WAKEUP_IRQHandler(void)
{
  uint32_t regVal;

  // Reconfigure system clock/PLL
  cpuPllSetup(CPU_MULTIPLIER_3);

  // Clear match bit on timer
  TMR_TMR32B0EMR = 0;

  // Clear pending bits
  SCB_STARTRSRP0CLR = SCB_STARTRSRP0CLR_MASK;

  // Clear SLEEPDEEP bit
  SCB_SCR &= ~SCB_SCR_SLEEPDEEP;

  // Disable the deep sleep timer
  TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;

  /* This handler takes care of all the port pins if they
  are configured as wakeup source. */
  regVal = SCB_STARTSRP0;
  if (regVal != 0)
  {
    SCB_STARTRSRP0CLR = regVal;
  }

  // Perform custom wakeup tasks
  pmuRestoreHW();

  __asm volatile ("NOP");

  return;
}

/**************************************************************************/
/*!
    Setup the clock for the watchdog timer.  The default setting is 7.8kHz.
*/
/**************************************************************************/
static void pmuWDTClockInit (void)
{
  /* Enable WDT clock */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_WDTOSC);

  /* Configure watchdog clock */
  /* Freq. = 0.5MHz, div = 64: WDT_OSC = 7.8125kHz  */
  /* Make sure this value is also reflected in PMU_WDTCLOCKSPEED_HZ */
  SCB_WDTOSCCTRL = SCB_WDTOSCCTRL_FREQSEL_0_5MHZ |
                   SCB_WDTOSCCTRL_DIVSEL_DIV64;

  // Switch main clock to WDT output
  SCB_MAINCLKSEL = SCB_MAINCLKSEL_SOURCE_WDTOSC;
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;       // Update clock source
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_DISABLE;      // Toggle update register once
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;

  // Wait until the clock is updated
  while (!(SCB_MAINCLKUEN & SCB_MAINCLKUEN_UPDATE));
}

/**************************************************************************/
/*!
    @brief Initialises the power management unit
*/
/**************************************************************************/
void pmuInit( void )
{
  /* Enable all clocks, even those turned off at power up. */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_WDTOSC_MASK |
                    SCB_PDRUNCFG_SYSOSC_MASK |
                    SCB_PDRUNCFG_ADC_MASK);

  // Setup the appropriate power profile if requested
  #if CFG_PMU_USEPOWERPROFILES == 1
    // Make sure that the clock for ROM is enabled?
    SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_ROM;
    if((*rom)->pPWRD == (void *)0xFFFFFFFF)
    {
      // Ooops ... this device doesn't support power profiles (LPC1114/301, etc.)
      return;
    }
    command[0] = CFG_CPU_CCLK / 1000000;
    command[1] = CFG_PMU_POWERPROFILE;
    command[2] = CFG_CPU_CCLK / 1000000;
    (*rom)->pPWRD->set_power(command, result);
  #endif

  return;
}

/**************************************************************************/
/*!
    @brief Puts select peripherals in sleep mode.

    This function will put the device into sleep mode.  Most gpio pins
    can be used to wake the device up, but the pins must first be
    configured for this in pmuInit.

    @section Example
    @code
    // Configure wakeup sources before going into sleep/deep-sleep.
    // By default, pin 0.1 is configured as wakeup source (falling edge)
    pmuInit();

    // Enter sleep mode
    pmuSleep();
    @endcode
*/
/**************************************************************************/
void pmuSleep()
{
  SCB_PDAWAKECFG = SCB_PDRUNCFG;
  __asm volatile ("WFI");
  return;
}

/**************************************************************************/
/*!
    @brief  Puts the device in deep-sleep mode, and alternately switches
            to the WDTOSC if a timed wakeup is required.

    The device can be configured to wakeup from deep-sleep mode after a
    specified delay by supplying a non-zero value to the wakeupSeconds
    parameter.  This will configure CT32B0 to toggle pin 0.1 (CT32B0_MAT2)
    after x seconds, waking the device up.  The timer will be configured
    to run off the WDT OSC while in deep-sleep mode, meaning that WDTOSC
    should not be powered off (using the sleepCtrl parameter) when a
    wakeup delay is specified.

    @param[in]  wakeupSeconds
                The number of seconds to wait until the device will
                wakeup.  If you do not wish to wakeup after a specific
                delay, enter a value of 0.

    @code
    // Configure wakeup sources before going into sleep/deep-sleep
    // By default, pin 0.1 is configured as wakeup source
    pmuInit();

    // Enter deep sleep mode (wakeup after 5 seconds)
    // If no wakeup is required, enter 0 for the timeout (lower power)
    pmuDeepSleep(5);
    @endcode
*/
/**************************************************************************/
void pmuDeepSleep(uint32_t wakeupSeconds)
{
  // Setup the board for deep sleep mode, remapping pins for lower power
  pmuSetupHW();

  SCB_PDAWAKECFG = SCB_PDRUNCFG;
  SCB_SCR |= SCB_SCR_SLEEPDEEP;

  /* Configure system to run from WDT and setup TMR32B0 for wakeup        */
  if (wakeupSeconds > 0)
  {
    // Enable the WDTOSC during deep sleep since it's required for wakeup
    // BOD is turned off, but can be turned on here if required
    SCB_PDSLEEPCFG = SCB_PDSLEEPCFG_BOD_OFF_WDOSC_ON;

    // Disable 32-bit timer 0 if currently in use
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;

    // Disable internal pullup on 0.1
    gpioSetPullup(&IOCON_PIO0_1, gpioPullupMode_Inactive);

    /* Enable the clock for CT32B0 (in case it's not enabled) */
    SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B0);

    /* Configure 0.1 as Timer0_32 MAT2 */
    IOCON_PIO0_1 &= ~IOCON_PIO0_1_FUNC_MASK;
    IOCON_PIO0_1 |= IOCON_PIO0_1_FUNC_CT32B0_MAT2;

    /* Set appropriate timer delay */
    TMR_TMR32B0MR0 = PMU_WDTCLOCKSPEED_HZ * wakeupSeconds;

    /* Configure match control register to reset on MR0 */
    TMR_TMR32B0MCR |= (TMR_TMR32B0MCR_MR0_RESET_ENABLED);

    /* Configure external match register to set 0.1 high on match */
    TMR_TMR32B0EMR &= ~(0xFF<<4);                   // Clear EMR config bits
    TMR_TMR32B0EMR |= TMR_TMR32B0EMR_EMC2_HIGH;     // Set MR2 (0.1) high on match

    /* Enable wakeup interrupt (P0.1..11 and P1.0 can be used) */
    NVIC_EnableIRQ(WAKEUP1_IRQn);      // P0.1  (CT32B0_MAT2)
    //NVIC_EnableIRQ(WAKEUP11_IRQn);   // P0.11 (CT32B0_MAT3)

    /* Use RISING EDGE for wakeup detection. */
    SCB_STARTAPRP0 |= SCB_STARTAPRP0_APRPIO0_1;

    /* Clear all wakeup sources */
    SCB_STARTRSRP0CLR = SCB_STARTRSRP0CLR_MASK;

    /* Enable Port 0.1 as wakeup source. */
    SCB_STARTERP0 |= SCB_STARTERP0_ERPIO0_1;

    // Reconfigure clock to run from WDTOSC
    pmuWDTClockInit();
	
    /* Start the timer */
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;
  }
  else
  {
    // No WDTOSC required since there is no timed wakeup
    // BOD is turned off, but can be turned on here if required
    SCB_PDSLEEPCFG = SCB_PDSLEEPCFG_BOD_OFF_WDOSC_OFF;
  }

  // Send Wait For Interrupt command
  __asm volatile ("WFI");
  return;
}

/**************************************************************************/
/*!
    @brief Puts the device in deep power-down mode.

    This function will configure the PMU control register and enter
    deep power-down mode.  Pre-determined values are stored in the four
    general-purpose registers (PMU_GPREG0..3), which can be used to persist
    any essential system settings while the device is in deep power-down
    mode, so long as 3.3V is still available.

    @warning    The only way to wake a device up from deep power-down mode
                is to set a low-level on P1.4.  If 3.3V power is lost, the
                values stored in the four general-purpose registers will
                also be lost.

    @section Example

    @code
    #include "core/cpu/cpu.h"
    #include "core/pmu/pmu.h"

    int main(void)
    {
      cpuInit();
      pmuInit();

      // Enter power-down mode
      pmuPowerDown();

      while(1)
      {
        // Device was woken up by WAKEUP pin
      }
    }
    @endcode
*/
/**************************************************************************/
void pmuPowerDown( void )
{
  uint32_t regVal;

  // Make sure HW and external devices are in low power mode
  pmuSetupHW();

  if ( (PMU_PMUCTRL & ((0x1<<8) | (PMU_PMUCTRL_DPDFLAG))) != 0x0 )
  {
    /* Check sleep and deep power down bits. If sleep and/or
       deep power down mode are entered, clear the PCON bits. */
    regVal = PMU_PMUCTRL;
    regVal |= ((0x1<<8) |
               (PMU_PMUCTRL_DPDEN_SLEEP) |
               (PMU_PMUCTRL_DPDFLAG));
    PMU_PMUCTRL = regVal;

    if ( (PMU_GPREG0 != 0x12345678)||(PMU_GPREG1 != 0x87654321)
       ||(PMU_GPREG2 != 0x56781234)||(PMU_GPREG3 != 0x43218765) )
    {
      while (1);
    }
  }
  else
  {
    /* If in neither sleep nor deep-sleep mode, enter deep power down mode. */
    PMU_GPREG0 = 0x12345678;
    PMU_GPREG1 = 0x87654321;
    PMU_GPREG2 = 0x56781234;
    PMU_GPREG3 = 0x43218765;
    SCB_SCR |= SCB_SCR_SLEEPDEEP;
    PMU_PMUCTRL = PMU_PMUCTRL_DPDEN_DEEPPOWERDOWN;
    __asm volatile ("WFI");
  }
  return;
}

/**************************************************************************/
/*!
    @brief  Configures parts and system peripherals to use lower power
            before entering sleep mode
*/
/**************************************************************************/
void pmuSetupHW(void)
{
  #ifdef CFG_CHIBI
    // Put Chibi/AT86RF212 into sleep mode
    chb_sleep(TRUE);
  #endif

  #ifdef CFG_SDCARD
    // Turn off SD card
    gpioSetValue( CFG_SDCARD_ENPORT, CFG_SDCARD_ENPIN, 0 );
    // Set the card detect pin to output and low (saves power)
    gpioSetDir(CFG_SDCARD_CDPORT, CFG_SDCARD_CDPIN, gpioDirection_Output);
    gpioSetValue(CFG_SDCARD_CDPORT, CFG_SDCARD_CDPIN, 0);

    // Set SSP1 pins to GPIO and output since the SD card can
    // draw current from these pins (saves 500-700uA when card inserted)
    IOCON_PIO2_0 = IOCON_PIO2_0_FUNC_GPIO;
    IOCON_PIO2_1 = IOCON_PIO2_1_FUNC_GPIO;
    IOCON_PIO2_2 = IOCON_PIO2_2_FUNC_GPIO;
    IOCON_PIO2_3 = IOCON_PIO2_3_FUNC_GPIO;
    gpioSetDir(2, 0, gpioDirection_Output);
    gpioSetValue(2, 0, 0);
    gpioSetDir(2, 1, gpioDirection_Output);
    gpioSetValue(2, 1, 0);
    gpioSetDir(2, 2, gpioDirection_Output);
    gpioSetValue(2, 2, 0);
    gpioSetDir(2, 3, gpioDirection_Output);
    gpioSetValue(2, 3, 0);
  #endif

  // Make sure that the battery voltage divider is turned off
  #ifdef CFG_BAT
    gpioSetValue(CFG_BAT_ENPORT, CFG_BAT_ENPIN, 0 );
  #endif

  // Switch to alternate voltage if dual output supply is being used
  #if defined CFG_VREG_ALT_PRESENT && CFG_VREG_ALT_PRESENT == 1
    gpioSetValue(CFG_VREG_ALT_PORT, CFG_VREG_ALT_PIN, 1);
  #endif
}

/**************************************************************************/
/*!
    @brief  Restores parts and system peripherals to an appropriate
            state after waking up from deep-sleep mode
*/
/**************************************************************************/
void pmuRestoreHW(void)
{
  // Set power back to main voltage if dual output supply is being used
  #if defined CFG_VREG_ALT_PRESENT && CFG_VREG_ALT_PRESENT == 1
    gpioSetValue(CFG_VREG_ALT_PORT, CFG_VREG_ALT_PIN, 0);
  #endif

  #ifdef CFG_CHIBI
    // Wakeup Chibi/Transceiver
    chb_sleep(FALSE);
  #endif
}

