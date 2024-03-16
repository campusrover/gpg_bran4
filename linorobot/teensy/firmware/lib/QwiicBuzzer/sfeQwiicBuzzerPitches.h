/******************************************************************************
    sfeQwiicBuzzerPitches.h
    SparkFun Qwiic Buzzer Library header file
    This file contains a list of commonly found notes on a piano.

    by Pete Lewis @SparkFun Electronics
    January 2024

    Based on original source code written by Tom Igeo in Jan 2010:
    https://www.arduino.cc/en/Tutorial/BuiltInExamples/toneMelody
    http://www.arduino.cc/en/Tutorial/Tone

    SPDX-License-Identifier: MIT

    Copyright (c) 2023 SparkFun Electronics

    Distributed as-is; no warranty is given.
******************************************************************************/

/*************************************************
     Public Constants
*************************************************/

#define SFE_QWIIC_BUZZER_NOTE_REST  0
#define SFE_QWIIC_BUZZER_NOTE_B0    31
#define SFE_QWIIC_BUZZER_NOTE_C1    33
#define SFE_QWIIC_BUZZER_NOTE_CS1   35
#define SFE_QWIIC_BUZZER_NOTE_D1    37
#define SFE_QWIIC_BUZZER_NOTE_DS1   39
#define SFE_QWIIC_BUZZER_NOTE_E1    41
#define SFE_QWIIC_BUZZER_NOTE_F1    44
#define SFE_QWIIC_BUZZER_NOTE_FS1   46
#define SFE_QWIIC_BUZZER_NOTE_G1    49
#define SFE_QWIIC_BUZZER_NOTE_GS1   52
#define SFE_QWIIC_BUZZER_NOTE_A1    55
#define SFE_QWIIC_BUZZER_NOTE_AS1   58
#define SFE_QWIIC_BUZZER_NOTE_B1    62
#define SFE_QWIIC_BUZZER_NOTE_C2    65
#define SFE_QWIIC_BUZZER_NOTE_CS2   69
#define SFE_QWIIC_BUZZER_NOTE_D2    73
#define SFE_QWIIC_BUZZER_NOTE_DS2   78
#define SFE_QWIIC_BUZZER_NOTE_E2    82
#define SFE_QWIIC_BUZZER_NOTE_F2    87
#define SFE_QWIIC_BUZZER_NOTE_FS2   93
#define SFE_QWIIC_BUZZER_NOTE_G2    98
#define SFE_QWIIC_BUZZER_NOTE_GS2   104
#define SFE_QWIIC_BUZZER_NOTE_A2    110
#define SFE_QWIIC_BUZZER_NOTE_AS2   117
#define SFE_QWIIC_BUZZER_NOTE_B2    123
#define SFE_QWIIC_BUZZER_NOTE_C3    131
#define SFE_QWIIC_BUZZER_NOTE_CS3   139
#define SFE_QWIIC_BUZZER_NOTE_D3    147
#define SFE_QWIIC_BUZZER_NOTE_DS3   156
#define SFE_QWIIC_BUZZER_NOTE_E3    165
#define SFE_QWIIC_BUZZER_NOTE_F3    175
#define SFE_QWIIC_BUZZER_NOTE_FS3   185
#define SFE_QWIIC_BUZZER_NOTE_G3    196
#define SFE_QWIIC_BUZZER_NOTE_GS3   208
#define SFE_QWIIC_BUZZER_NOTE_A3    220
#define SFE_QWIIC_BUZZER_NOTE_AS3   233
#define SFE_QWIIC_BUZZER_NOTE_B3    247
#define SFE_QWIIC_BUZZER_NOTE_C4    262
#define SFE_QWIIC_BUZZER_NOTE_CS4   277
#define SFE_QWIIC_BUZZER_NOTE_D4    294
#define SFE_QWIIC_BUZZER_NOTE_DS4   311
#define SFE_QWIIC_BUZZER_NOTE_E4    330
#define SFE_QWIIC_BUZZER_NOTE_F4    349
#define SFE_QWIIC_BUZZER_NOTE_FS4   370
#define SFE_QWIIC_BUZZER_NOTE_G4    392
#define SFE_QWIIC_BUZZER_NOTE_GS4   415
#define SFE_QWIIC_BUZZER_NOTE_A4    440
#define SFE_QWIIC_BUZZER_NOTE_AS4   466
#define SFE_QWIIC_BUZZER_NOTE_B4    494
#define SFE_QWIIC_BUZZER_NOTE_C5    523
#define SFE_QWIIC_BUZZER_NOTE_CS5   554
#define SFE_QWIIC_BUZZER_NOTE_D5    587
#define SFE_QWIIC_BUZZER_NOTE_DS5   622
#define SFE_QWIIC_BUZZER_NOTE_E5    659
#define SFE_QWIIC_BUZZER_NOTE_F5    698
#define SFE_QWIIC_BUZZER_NOTE_FS5   740
#define SFE_QWIIC_BUZZER_NOTE_G5    784
#define SFE_QWIIC_BUZZER_NOTE_GS5   831
#define SFE_QWIIC_BUZZER_NOTE_A5    880
#define SFE_QWIIC_BUZZER_NOTE_AS5   932
#define SFE_QWIIC_BUZZER_NOTE_B5    988
#define SFE_QWIIC_BUZZER_NOTE_C6    1047
#define SFE_QWIIC_BUZZER_NOTE_CS6   1109
#define SFE_QWIIC_BUZZER_NOTE_D6    1175
#define SFE_QWIIC_BUZZER_NOTE_DS6   1245
#define SFE_QWIIC_BUZZER_NOTE_E6    1319
#define SFE_QWIIC_BUZZER_NOTE_F6    1397
#define SFE_QWIIC_BUZZER_NOTE_FS6   1480
#define SFE_QWIIC_BUZZER_NOTE_G6    1568
#define SFE_QWIIC_BUZZER_NOTE_GS6   1661
#define SFE_QWIIC_BUZZER_NOTE_A6    1760
#define SFE_QWIIC_BUZZER_NOTE_AS6   1865
#define SFE_QWIIC_BUZZER_NOTE_B6    1976
#define SFE_QWIIC_BUZZER_NOTE_C7    2093
#define SFE_QWIIC_BUZZER_NOTE_CS7   2217
#define SFE_QWIIC_BUZZER_NOTE_D7    2349
#define SFE_QWIIC_BUZZER_NOTE_DS7   2489
#define SFE_QWIIC_BUZZER_NOTE_E7    2637
#define SFE_QWIIC_BUZZER_NOTE_F7    2794
#define SFE_QWIIC_BUZZER_NOTE_FS7   2960
#define SFE_QWIIC_BUZZER_NOTE_G7    3136
#define SFE_QWIIC_BUZZER_NOTE_GS7   3322
#define SFE_QWIIC_BUZZER_NOTE_A7    3520
#define SFE_QWIIC_BUZZER_NOTE_AS7   3729
#define SFE_QWIIC_BUZZER_NOTE_B7    3951
#define SFE_QWIIC_BUZZER_NOTE_C8    4186
#define SFE_QWIIC_BUZZER_NOTE_CS8   4435
#define SFE_QWIIC_BUZZER_NOTE_D8    4699
#define SFE_QWIIC_BUZZER_NOTE_DS8   4978


// Backwards compatibility with original "pitches.h" file written by Tim Igeo, 2010.
#define NOTE_B0  SFE_QWIIC_BUZZER_NOTE_B0
#define NOTE_C1  SFE_QWIIC_BUZZER_NOTE_C1
#define NOTE_CS1 SFE_QWIIC_BUZZER_NOTE_CS1
#define NOTE_D1  SFE_QWIIC_BUZZER_NOTE_D1
#define NOTE_DS1 SFE_QWIIC_BUZZER_NOTE_DS1
#define NOTE_E1  SFE_QWIIC_BUZZER_NOTE_E1
#define NOTE_F1  SFE_QWIIC_BUZZER_NOTE_F1
#define NOTE_FS1 SFE_QWIIC_BUZZER_NOTE_FS1
#define NOTE_G1  SFE_QWIIC_BUZZER_NOTE_G1
#define NOTE_GS1 SFE_QWIIC_BUZZER_NOTE_GS1
#define NOTE_A1  SFE_QWIIC_BUZZER_NOTE_A1
#define NOTE_AS1 SFE_QWIIC_BUZZER_NOTE_AS1
#define NOTE_B1  SFE_QWIIC_BUZZER_NOTE_B1
#define NOTE_C2  SFE_QWIIC_BUZZER_NOTE_C2
#define NOTE_CS2 SFE_QWIIC_BUZZER_NOTE_CS2
#define NOTE_D2  SFE_QWIIC_BUZZER_NOTE_D2
#define NOTE_DS2 SFE_QWIIC_BUZZER_NOTE_DS2
#define NOTE_E2  SFE_QWIIC_BUZZER_NOTE_E2
#define NOTE_F2  SFE_QWIIC_BUZZER_NOTE_F2
#define NOTE_FS2 SFE_QWIIC_BUZZER_NOTE_FS2
#define NOTE_G2  SFE_QWIIC_BUZZER_NOTE_G2
#define NOTE_GS2 SFE_QWIIC_BUZZER_NOTE_GS2
#define NOTE_A2  SFE_QWIIC_BUZZER_NOTE_A2
#define NOTE_AS2 SFE_QWIIC_BUZZER_NOTE_AS2
#define NOTE_B2  SFE_QWIIC_BUZZER_NOTE_B2
#define NOTE_C3  SFE_QWIIC_BUZZER_NOTE_C3
#define NOTE_CS3 SFE_QWIIC_BUZZER_NOTE_CS3
#define NOTE_D3  SFE_QWIIC_BUZZER_NOTE_D3
#define NOTE_DS3 SFE_QWIIC_BUZZER_NOTE_DS3
#define NOTE_E3  SFE_QWIIC_BUZZER_NOTE_E3
#define NOTE_F3  SFE_QWIIC_BUZZER_NOTE_F3
#define NOTE_FS3 SFE_QWIIC_BUZZER_NOTE_FS3
#define NOTE_G3  SFE_QWIIC_BUZZER_NOTE_G3
#define NOTE_GS3 SFE_QWIIC_BUZZER_NOTE_GS3
#define NOTE_A3  SFE_QWIIC_BUZZER_NOTE_A3
#define NOTE_AS3 SFE_QWIIC_BUZZER_NOTE_AS3
#define NOTE_B3  SFE_QWIIC_BUZZER_NOTE_B3
#define NOTE_C4  SFE_QWIIC_BUZZER_NOTE_C4
#define NOTE_CS4 SFE_QWIIC_BUZZER_NOTE_CS4
#define NOTE_D4  SFE_QWIIC_BUZZER_NOTE_D4
#define NOTE_DS4 SFE_QWIIC_BUZZER_NOTE_DS4
#define NOTE_E4  SFE_QWIIC_BUZZER_NOTE_E4
#define NOTE_F4  SFE_QWIIC_BUZZER_NOTE_F4
#define NOTE_FS4 SFE_QWIIC_BUZZER_NOTE_FS4
#define NOTE_G4  SFE_QWIIC_BUZZER_NOTE_G4
#define NOTE_GS4 SFE_QWIIC_BUZZER_NOTE_GS4
#define NOTE_A4  SFE_QWIIC_BUZZER_NOTE_A4
#define NOTE_AS4 SFE_QWIIC_BUZZER_NOTE_AS4
#define NOTE_B4  SFE_QWIIC_BUZZER_NOTE_B4
#define NOTE_C5  SFE_QWIIC_BUZZER_NOTE_C5
#define NOTE_CS5 SFE_QWIIC_BUZZER_NOTE_CS5
#define NOTE_D5  SFE_QWIIC_BUZZER_NOTE_D5
#define NOTE_DS5 SFE_QWIIC_BUZZER_NOTE_DS5
#define NOTE_E5  SFE_QWIIC_BUZZER_NOTE_E5
#define NOTE_F5  SFE_QWIIC_BUZZER_NOTE_F5
#define NOTE_FS5 SFE_QWIIC_BUZZER_NOTE_FS5
#define NOTE_G5  SFE_QWIIC_BUZZER_NOTE_G5
#define NOTE_GS5 SFE_QWIIC_BUZZER_NOTE_GS5
#define NOTE_A5  SFE_QWIIC_BUZZER_NOTE_A5
#define NOTE_AS5 SFE_QWIIC_BUZZER_NOTE_AS5
#define NOTE_B5  SFE_QWIIC_BUZZER_NOTE_B5
#define NOTE_C6  SFE_QWIIC_BUZZER_NOTE_C6
#define NOTE_CS6 SFE_QWIIC_BUZZER_NOTE_CS6
#define NOTE_D6  SFE_QWIIC_BUZZER_NOTE_D6
#define NOTE_DS6 SFE_QWIIC_BUZZER_NOTE_DS6
#define NOTE_E6  SFE_QWIIC_BUZZER_NOTE_E6
#define NOTE_F6  SFE_QWIIC_BUZZER_NOTE_F6
#define NOTE_FS6 SFE_QWIIC_BUZZER_NOTE_FS6
#define NOTE_G6  SFE_QWIIC_BUZZER_NOTE_G6
#define NOTE_GS6 SFE_QWIIC_BUZZER_NOTE_GS6
#define NOTE_A6  SFE_QWIIC_BUZZER_NOTE_A6
#define NOTE_AS6 SFE_QWIIC_BUZZER_NOTE_AS6
#define NOTE_B6  SFE_QWIIC_BUZZER_NOTE_B6
#define NOTE_C7  SFE_QWIIC_BUZZER_NOTE_C7
#define NOTE_CS7 SFE_QWIIC_BUZZER_NOTE_CS7
#define NOTE_D7  SFE_QWIIC_BUZZER_NOTE_D7
#define NOTE_DS7 SFE_QWIIC_BUZZER_NOTE_DS7
#define NOTE_E7  SFE_QWIIC_BUZZER_NOTE_E7
#define NOTE_F7  SFE_QWIIC_BUZZER_NOTE_F7
#define NOTE_FS7 SFE_QWIIC_BUZZER_NOTE_FS7
#define NOTE_G7  SFE_QWIIC_BUZZER_NOTE_G7
#define NOTE_GS7 SFE_QWIIC_BUZZER_NOTE_GS7
#define NOTE_A7  SFE_QWIIC_BUZZER_NOTE_A7
#define NOTE_AS7 SFE_QWIIC_BUZZER_NOTE_AS7
#define NOTE_B7  SFE_QWIIC_BUZZER_NOTE_B7
#define NOTE_C8  SFE_QWIIC_BUZZER_NOTE_C8
#define NOTE_CS8 SFE_QWIIC_BUZZER_NOTE_CS8
#define NOTE_D8  SFE_QWIIC_BUZZER_NOTE_D8
#define NOTE_DS8 SFE_QWIIC_BUZZER_NOTE_DS8