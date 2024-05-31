void Config_TIM1(void)
{
    RCC->APB2ENR |= 1 << 0;  // Enable timer 1
    TIM1->CR1 &= ~(1U << 0); // reset CEN and disable counter
    /*
    TIM1->CR1 &= ~(3U<<5); //clear CMS
    TIM1->CR1 &= ~(1U<<4); //enable upcounter
    */
    /**
     * Load counter range,
     * input clock frequency to the counter, and
     * configure enable auto reload register
     **/
    TIM1->ARR = 999;     // counter maximum value 1000 -- expect to get 1KHz
    TIM1->PSC = 179;     // for 1 MHz clock input
    TIM1->CR1 |= 1 << 7; // ARPE --  auto preload and reload eneable
    TIM1->CR1 |= 1 << 5; // enable center alignment
    TIM1->CR2 |= 1 << 3; // Capture/Compare DMA Selection
    /***
     * Set PWM channel
     **/
    /*
    TIM1->CCMR1 &= ~(7U<<4); // CH1 clear
    TIM1->CCMR1 &= ~(7U<<12); //CH2 clear
    TIM1->CCMR2 &= ~(7U<<4); //CH3 clear
    */
    // set the output channel for the PWM mode 1
    TIM1->CCMR1 |= 0x6 << 4;
    TIM1->CCMR1 |= 0x6 << 12;
    TIM1->CCMR2 |= 0x6 << 4;
    // OCxPE for allow value of CCRx is load to the active register
    // when an update event occur
    TIM1->CCMR1 |= 1 << 3;
    TIM1->CCMR1 |= 1 << 11;
    TIM1->CCMR2 |= 1 << 3;

    // Configur channels 3+3 (complementary) for output mode; Enable output
    TIM1->CCER |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10);
    // Set active polarity for the output -- active low
    TIM1->CCER |= (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9) | (1 << 11);
    // OSSR -- Off state selction for run mode low
    TIM1->BDTR |= 1 << 11;
    TIM1->BDTR |= 0x4D << 0; // please take care of dead time for state change
    // between normal complementary output
    TIM1->BDTR |= 1 << 15; // main output enable
    // set timer autoload preload enable
    TIM1->CR1 |= 1 << 7;
    // timer 1 update DMA request enable
    // TIM1->DIER |= 1<<8; //This should be uncomment to the USE of DMA
    // channel 1/2/3 counter initialized (ARR 1000!!)
    TIM1->CCR1 |= 500; // 50% duty cycle
    TIM1->CCR2 |= 300; // 30% duty cycle
    TIM1->CCR3 |= 800; // 80% duty cycle

    // register CCR1 offset 0x34 for DMA
    // Uncommet for DMA
    // uint8_t offset=(&(TIM1->CCR1)-&(TIM1->CR1));
    // offset &= 0x0F;
    // TIM1->DCR |= (uint8_t)(offset<<0);
    // TIM1->DCR |= 13<<0;
    // DMA Burst size
    // uncomment for DMA
    // TIM1->DCR |= 3<<8; //4-DMA transfer
    //  uncomment enable interrupt
    TIM1->DIER |= 7 << 1; // enable matching interrupt
    // Finally enable counting
    TIM1->CR1 |= 1 << 0;
}

void Config_GPIO_TIM(void)
{
    /* GPIO Port for Channel 1, 2 & 3, 1N, 2N, 3N
     * port: GPIOA and GPIOB ->  PA8, PA9, PA10, PA7/PB13, PB0/PB14, PB1/PB15
     */
    RCC->AHB1ENR |= (1 << 0) | (1 << 1); // Enable GPIOA and GPIOB
    // set Moder Register; reset and set -- optional
    GPIOA->MODER &= ~(3U << 16);
    GPIOA->MODER &= ~(3U << 18);
    GPIOA->MODER &= ~(3U << 20);
    // Optional
    GPIOB->MODER &= ~(3U << 26);
    GPIOB->MODER &= ~(3U << 0);
    GPIOB->MODER &= ~(3U << 2);
    // set the values for Alternate function for PWM
    GPIOA->MODER |= (2 << 16) | (2 << 18) | (2 << 20); // pin PA8, PA9,PA10 for output
    GPIOB->MODER |= (2 << 26) | (2 << 0) | (2 << 2);   // pin PB13, PB0, PB1 for
    // complementary output

    // Ouput speed
    GPIOA->OSPEEDR |= (3 << 16) | (3 << 18) | (3 << 20);
    GPIOB->OSPEEDR |= (3 << 26) | (3 << 0) | (3 << 2);
    // Set the alternate function for the PWM output
    // clear the alternate function and set (optional)
    GPIOA->AFR[1] &= ~(15U << 0);
    GPIOA->AFR[1] &= ~(15U << 4);
    GPIOA->AFR[1] &= ~(15U << 8);
    GPIOB->AFR[0] &= ~(15U << 0);
    GPIOB->AFR[0] &= ~(15U << 8);
    GPIOB->AFR[1] &= ~(15U << 20);
    // Set the alternate funtion for 3-phase PWM and complements
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4) | (1 << 8);
    GPIOB->AFR[0] |= (1 << 0) | (1 << 8);
    GPIOB->AFR[0] |= (1 << 20);
}