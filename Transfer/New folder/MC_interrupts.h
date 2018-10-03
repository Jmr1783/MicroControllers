/*
 * File:   MC_interrupts.h
 * Author: Jacob
 *
 * Created on April 10, 2017, 8:22 PM
 */

#ifndef MC_INTERRUPTS_H
#define    MC_INTERRUPTS_H

typedef enum mc_interrupt_calls_E {
//interupt control/triggering

    //flags
    MC_INTCON_RBIF,
    MC_INTCON_INT0IF,
    MC_INTCON_TMR0IF,
    //enable
    MC_INTCON_RBIE,
    MC_INTCON_INT0IE,
    MC_INTCON_TMR0IE,
    //global enables and prioritys see IPEN
    MC_INTCON_PEIE,
    MC_INTCON_GIEL,
    MC_INTCON_GIE,
    MC_INTCON_GIEH,

    //priority
    MC_INTCON2_RBIP,
    MC_INTCON2_INT3IP,
    MC_INTCON2_TMR0IP,
    //Edge select
    MC_INTCON2_INTEDG3,
    MC_INTCON2_INTEDG2,
    MC_INTCON2_INTEDG1,
    MC_INTCON2_INTEDG0,
    //enable 
    MC_INTCON2_NOT_RBPU,

    //flag
    MC_INTCON3_INT1IF,
    MC_INTCON3_INT2IF,
    MC_INTCON3_INT3IF,
    //enable
    MC_INTCON3_INT1IE,
    MC_INTCON3_INT2IE,
    MC_INTCON3_INT3IE,
    //priority
    MC_INTCON3_INT1IP,
    MC_INTCON3_INT2IP,

    
//Flags
    MC_PIR1_TMR1IF,
    MC_PIR1_TMR2IF,
    MC_PIR1_SSPIF,
    MC_PIR1_TX1IF,
    MC_PIR1_RC1IF,
    MC_PIR1_ADIF,

    MC_PIR2_TMR3GIF,
    MC_PIR2_TMR3IF,
    MC_PIR2_HLVDIF,
    MC_PIR2_BCLIF,
    MC_PIR2_OSCFIF,

    MC_PIR3_CCP1IF,
    MC_PIR3_CCP2IF,
    MC_PIR3_CTMUIF,
    MC_PIR3_TX2IF,
    MC_PIR3_RC2IF,

    MC_PIR4_CCP3IF,
    MC_PIR4_CCP4IF,
    MC_PIR4_CCP5IF, 	
    MC_PIR4_CMP1IF,
    MC_PIR4_CMP2IF,
    MC_PIR4_EEIF,
    MC_PIR4_TMR4IF,

    MC_PIR5_RXB0IF,
    MC_PIR5_RXB1IF,
    MC_PIR5_TXB0IF,
    MC_PIR5_TXB1IF,
    MC_PIR5_TXB2IF,
    MC_PIR5_ERRIF,
    MC_PIR5_WAKIF,
    MC_PIR5_IRXIF,
    
// Enables

    MC_PIE1_TMR1IE,
    MC_PIE1_TMR2IE,
    MC_PIE1_TMR1GIE,
    MC_PIE1_SSPIE,
    MC_PIE1_TX1IE,
    MC_PIE1_RC1IE,
    MC_PIE1_ADIE,
    MC_PIE1_PSPIE,

    MC_PIE2_TMR3GIE,
    MC_PIE2_TMR3IE,
    MC_PIE2_HLVDIE,
    MC_PIE2_BCLIE,
    MC_PIE2_OSCFIE,

    MC_PIE3_CCP1IE,
    MC_PIE3_CCP2IE,
    MC_PIE3_CTMUIE,
    MC_PIE3_TX2IE,
    MC_PIE3_RC2IE,

    MC_PIE4_CCP3IE,
    MC_PIE4_CCP4IE,
    MC_PIE4_CCP5IE,
    MC_PIE4_CMP1IE,
    MC_PIE4_CCP2IE,
    MC_PIE4_EEIE,
    MC_PIE4_TMR4IE,

    MC_PIE5_RXB0IE,
    MC_PIE5_RXB1IE,
    MC_PIE5_TXB0IE,
    MC_PIE5_TXB1IE,
    MC_PIE5_TXB2IE,
    MC_PIE5_ERRIE,
    MC_PIE5_WAKIE,
    MC_PIE5_IRXIE,

//Priority
    
    MC_IPR1_TMR1IP,
    MC_IPR1_TMR2IP,
    MC_IPR1_TMR1GIP,
    MC_IPR1_SSPIP,
    MC_IPR1_TX1IP,
    MC_IPR1_RC1IP,
    MC_IPR1_ADIP,
    MC_IPR1_PSPIP,

    MC_IPR2_TMR3GIP,
    MC_IPR2_TMR3IP,
    MC_IPR2_HLVDIP,
    MC_IPR2_BCLIP,
    MC_IPR2_OSCFIP,

    MC_IPR3_CCP1IP,
    MC_IPR3_CCP2IP,
    MC_IPR3_CTMUIP,
    MC_IPR3_TX2IP,
    MC_IPR3_RC2IP,

    MC_IPR4_CCP3IP,
    MC_IPR4_CCP4IP,
    MC_IPR4_CCP5IP,
    MC_IPR4_,
    MC_IPR4_CMP1IP,
    MC_IPR4_CMP2IP,
    MC_IPR4_EEIP,
    MC_IPR4_TMR4IP,

    MC_IPR5_RXB0IP,
    MC_IPR5_RXB1IP,
    MC_IPR5_TXB0IP,
    MC_IPR5_TXB1IP,
    MC_IPR5_TXB2IP,
    MC_IPR5_ERRIP,
    MC_IPR5_WAKIP,
    MC_IPR5_IRXIP,

    //enable
    MC_RCON_NOT_BOR, 
    MC_RCON_NOT_POR,
    //flags
    MC_RCON_NOT_PD,
    MC_RCON_NOT_TO,
    MC_RCON_NOT_RI,
    MC_RCON_NOT_CM,
    //status????????
    MC_RCON_SBOREN,
    //priority on/off
    MC_RCON_IPEN,

} mc_interrupt_calls_E;

//Enums...

//---------------------------------------------
// Function to enable registers. It will take enums and then use case statements to determine which bits to set.
//-----------------------------------------------
#endif    /* MC_INTERRUPTS_H */



    