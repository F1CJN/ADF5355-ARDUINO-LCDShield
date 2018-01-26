//
//
// 
//
//   ADF5355 and Arduino
//   By Alain Fort F1CJN and Dave Brink (for 64bits routines)jan 3,2018
//   
//
//   Cette version V0 est une version prototype. Peut être certaines fréqences sont bruitées par l4ADF5355.
//  ****************************************************** FRANCAIS ****************************************************
//   Ce programme utilise un Arduino Uno muni d'un "LCD button shield" de marque ROBOT, avec boutons permettant de commander
//   un ADF5355 qui generer une frequence comprise entre 54 et 13600MHz.
//   Vingt fréquences peuvent être memorisees dans le memoire EEPROM de l'Arduino.
//   Si une ou plusieurs fréquences sont mémorisées, alors la fréquence en memoire zero sera affichee à la mise sous tension
//   
//   Le curseur se deplace avec les touches LEFT and RIGHT. Les digits placés sous le curseur peuvent être modifiées avec
//   les touches UP et DOWN, ceci pour la fréquence, la memoire et la frequence de reference:
//   - pour la fréquence, il suffit de placer le curseur sur le chiffre à modifier,
//   - pour la mémoire , il suffit de placer le curseur sur le numero de memoire,
//   - pour la fréqence de référence, il suffit de placer le curseur sur 10 ou 26,
//   - pour la lecture ou écriture de la frequence en memoire, placer le curseur en bas à gauche (passage de REE(lecture
//    EEPROM) à WEE(Ecriture EEPROM).
//    Le curseur disparait apres quelques secondes et est re active lors de l'appui sur une touche.
//
//   MEMORISATION 
//   - pour la frequence, mettre à WEE, puis selectionner le numero de memoire, puis appuyer sur la touche SELECT pendant 
//    une seconde. Le mot MEMORISATION apparait alors sur l'ecran. Ceci fonctionne quelquesoit le placement du curseur excepte sur 
//    l'emplacement de la fréquence de réference 10 ou 26.
//   - Pour la frequence de reference, placer le curseur sur 10 ou 26, puis appuyer pendant 1s sur la touche SELECT.
//
//  ********************************************* HARDWARE IMPORTANT *******************************************************
//  Avec un Arduino UN0 : utiliser un pont de résistances pour réduire la tension de 5V à 1,8V, MOSI (pin 11) vers
//  ADF DATA, SCK (pin13) vers CLK ADF, Select (PIN 3) vers LE. 
//  Resistances de 560 Ohm avec 1000 Ohm à la masse sur les pins 11, 13 et 3 de l'Arduino UNO pour
//  que les signaux envoyés DATA, CLK et LE vers l'ADF5355 ne depassent pas 3,3 Volt.
//  Pin 2 de l'Arduino (pour la detection de lock) connectee directement à la sortie MUXOUT de la carte ADF5355
//  La carte ADF est alimentée en 5V par la carte Arduino (les pins +5V et GND sont proches de la LED Arduino).
//  ***********************************************************************************************************************
//  Attention : si vous utiliser un afficheur ROBOT Version 1.1 il faut modifier la routine de lecture des boutons
//  en enlevant les commentaires de la version 1.1 et en mettant en commentaires la version 1.0
//
//  *************************************************** ENGLISH ***********************************************************
//  This is a prototype program. May be some ADF frequency are "noisy".
//  This sketch uses and Arduino Uno (5€), a standard "LCD buttons shield" from ROBOT (5€), with buttons and an ADF5355 chineese
//  card found at EBAY (60€). The frequency can be programmed between 54 and 13600 MHz.
//  Twenty frequencies can be memorized into the Arduino EEPROM.
//  If one or more frequencies are memorized, then at power on, the memory zero is always selected.
//
//   The cursor can move with le LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10 or 26 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the reference frequence,move the cursor on 10 or 26 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor disappears after few seconds and is re activated when a button is pressed.
//
//   MEMORIZATION 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word MEMORISATION 
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 26 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second. 

//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UN0 : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 560 Ohm with 1000 Ohm to ground on Arduino pins 11, 13 et 3 to adapt from 5V
//  to 1.8V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin 2 (for lock detection) directly connected to ADF5355 card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs +5V and GND are closed to the Arduino LED).

//************************************************* MANUEL*****************************************************************
//Touche LEFT    curseur à gauche, cursor to the left
//Touche RIGHT   curseur à droite, cursor to the right
//Touche UP      incremente frequence ou memoire, increase frequency
//Touche DOWN    decremente frequence ou memoire, decrease frequency
//Touche SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shied version 1.1, it is necessary to modify the read_lcd_buttons sub routine 
// you need not to comment the 1.1 version and to comment the 1.0 version. See below

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF5355_LE 3
#define R04 0X36008B84    // DB4=0
#define R04_EN 0X36008B94 //DB4=1
#define MODE2  1000  //

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte poscursor = 0; //position curseur courante 0 à 15
byte line = 0,X2=0; // ligne afficheur LCD en cours 0 ou 1
byte memoire,RWtemp; // numero de la memoire EEPROM

//uint32_t registers [13] = {0x300E60, 0xC4EC4E1, 0xC4EBFFF2, 0x3, 0x36008B84, 0x800025, 0x35A08076, 0x12000007, 0x102D0428, 0x1615FCC9, 0xC020BA, 0x61300B, 0x1041C}; // 187,5 MHz with a 26 MHz Reference Clock
  uint32_t registers [13] = {0x200E60, 0xC4EC4E1, 0xC043E82, 0x3, 0x36008B84, 0x800025, 0x35A12076, 0x12000007, 0x102D0428, 0xB0B3CC9, 0xC0107A, 0x61300B, 0x1041C}; // 187,5 MHz with a 26 MHz Reference Clock
//uint32_t registers [13] = {0x20BB80, 0x1, 0x3E82, 0x3, 0x36068B84, 0x800025, 0x35A12076, 0x120000E7, 0x102D0428, 0x2013CC9, 0xC0017A, 0x61300B, 0x1041C}; // 187,5 MHz with a 2 MHz Reference Clock
int address,modif=0,WEE=0,Out_Divider;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // utilisé pour mesurer la durée d'appui sur une touche
unsigned int i = 0;


double RFout, REFin, OutputChannelSpacing;
unsigned long RFint,RFintold,RFcalc,RFOUT,TEMP,TEST;
long TEMPLONG,RF_Divider,HERTZ,HERTZold,DELTAIF; 
 unsigned long FVCO[]={0,600000000};// en dizaine de Hz*
 unsigned long FPFD[]={0,26}; // en MHz
 unsigned long FVCOTEMP[]={0,0};// en dizaine de Hz*
 unsigned long FPFDTEMP[]={0,0};

 unsigned long N10E6[]={0,1000000};
 unsigned long N10E3[]={0,1000};
  
 unsigned long INT[]={0x0,0x0};
 unsigned long N[]={0x0,0x0};
 unsigned long NX[]={0x0,0x0};
 unsigned long FRAC1[]={0x0,0x0};
 unsigned long FRAC11[]={0x0,0x0};
 unsigned long FRAC2[]={0x0,0x0};
 unsigned long HZ[]={0x0,0x0};
 unsigned long FRAC1exact[]={0x0,0x0};
 unsigned long MOD1[]={0,16777216};
 unsigned long MOD2[]={0,1000};

 byte lock=2,AMP=4;
 unsigned int long reg0, reg1;
 unsigned int long FRAC1R,FRAC2R,INTR,RF_Select,I_BLEED_N,RFB;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//**************  64 bits routines written by Dave Brink *************************************
unsigned long zero64[]={0,0}; //just for comparisons sake

void init64(unsigned long  an[], unsigned long bigPart, unsigned long littlePart ){
 an[0]=bigPart;
 an[1]=littlePart;
}

//left shift 64 bit "number"
void shl64(unsigned long  an[]){
an[0] <<= 1;
if(an[1] & 0x80000000)
  an[0]++;
an[1] <<= 1;
}

//right shift 64 bit "number"
void shr64(unsigned long  an[]){
an[1] >>= 1;
if(an[0] & 0x1)
  an[1]+=0x80000000;
an[0] >>= 1;
}

//add ann to an
void add64(unsigned long  an[], unsigned long  ann[]){
 an[0]+=ann[0];
 if(an[1] + ann[1] < ann[1])
   an[0]++;
 an[1]+=ann[1];
}

//subtract ann from an
void sub64(unsigned long  an[], unsigned long  ann[]){
 an[0]-=ann[0];
 if(an[1] < ann[1]){
   an[0]--;
 }
 an[1]-= ann[1];
}

//true if an == ann
boolean eq64(unsigned long  an[], unsigned long  ann[]){
 return (an[0]==ann[0]) && (an[1]==ann[1]);
}

//true if an < ann
boolean lt64(unsigned long  an[], unsigned long  ann[]){
 if(an[0]>ann[0]) return false;
 return (an[0]<ann[0]) || (an[1]<ann[1]);
}

//divide num by den
void div64(unsigned long num[], unsigned long den[]){
 unsigned long quot[2];
 unsigned long qbit[2];
 unsigned long tmp[2];
 init64(quot,0,0);
 init64(qbit,0,1);

 if (eq64(num, zero64)) {  //numerator 0, call it 0
   init64(num,0,0);
   return;            
 }

 if (eq64(den, zero64)) { //numerator not zero, denominator 0, infinity in my book.
   init64(num,0xffffffff,0xffffffff);
   return;            
 }

 init64(tmp,0x80000000,0);
 while(lt64(den,tmp)){
   shl64(den);
   shl64(qbit);
 }
 
 while(!eq64(qbit,zero64)){
   if(lt64(den,num) || eq64(den,num)){
     sub64(num,den);
     add64(quot,qbit);
   }
   shr64(den);
   shr64(qbit);
 }

 //remainder now in num, but using it to return quotient for now  
 init64(num,quot[0],quot[1]);
}

//multiply num by den
void mul64(unsigned long an[], unsigned long ann[]){
 unsigned long p[2] = {0,0};
 unsigned long y[2] = {ann[0], ann[1]};
 while(!eq64(y,zero64)) {
   if(y[1] & 1)
     add64(p,an);
   shl64(an);
   shr64(y);
 }
 init64(an,p[0],p[1]);
}
//******************************End of 64 bits routines****************************************


// ****************************Impression variable f64****************************************
void prt(unsigned long  an[]){
Serial.print(an[0],DEC);
Serial.print(" ");
Serial.println(an[1],DEC);
}

//**************************** SP LECTURE BOUTONS ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in < 790)lcd.blink();
  
  if (adc_key_in < 50)return btnRIGHT;  // pour Afficheur ROBOT V1.0
  if (adc_key_in < 195)return btnUP;
  if (adc_key_in < 380)return btnDOWN;
  if (adc_key_in < 555)return btnLEFT;
  if (adc_key_in < 790)return btnSELECT; // Fin Afficheur ROBOT V1.0

  //if (adc_key_in < 50)return btnRIGHT; // pour Afficheur ROBOT V1.1
  //if (adc_key_in < 250)return btnUP;
  //if (adc_key_in < 450)return btnDOWN;
  //if (adc_key_in < 650)return btnLEFT;
  //if (adc_key_in < 850)return btnSELECT; // fin Afficheur ROBOT V1.1

  return btnNONE;  // touches non appuyees
}

//***************************** SP Affichage Fréquence sur LCD *******************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0,0); // ligne 1
  RFcalc=(RFint/1000000);
  if (RFcalc<10)lcd.print(" ");
  if (RFcalc==0)lcd.print(" ");
  if (RFcalc>0)lcd.print(RFcalc);
  lcd.print("."); 
  RFcalc=RFint-((RFint/1000000)*1000000);
  if (RFcalc < 100000) lcd.print("0");
  if (RFcalc < 10000)  lcd.print("0"); 
  lcd.print(RFcalc/1000);lcd.print(".");//
  RFcalc=RFint-((RFint/1000)*1000);
  if (RFcalc<100)lcd.print("0");
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print(".");
  RFcalc=HERTZ-((HERTZ/1000)*1000);
  if (RFcalc<100)lcd.print("0");
  if (RFcalc<10)lcd.print("0");
  lcd.print(HERTZ);
  lcd.print("Hz");
  
  lcd.setCursor(0,1); // 2e ligne
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memoire<10){lcd.print(" ");}
  lcd.print(memoire,DEC);
  delay(100);
  if  ((digitalRead(2)==1))lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.print(FPFD[1],DEC);
  lcd.setCursor(poscursor,line);
}

void WriteRegister32(const uint32_t value)   //Programme un registre 32bits
{
  digitalWrite(ADF5355_LE, LOW);
  for (int i = 3; i >= 0; i--)          // boucle sur 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // décalage, masquage de l'octet et envoi via SPI
  digitalWrite(ADF5355_LE, HIGH);
  digitalWrite(ADF5355_LE, LOW);
}

void SetADF5355()  // Programme tous les registres de l'ADF5355
{ for (int i = 13; i >= 0; i--)  // programmation ADF5355 en commencant par R12
    WriteRegister32(registers[i]);
}

//************************SP Mise à jour des registres ***************************************
void updateADF5355() { // mise à jour des registres

   AMP=3;I_BLEED_N=9; // N=9
   WriteRegister32((RF_Select<<21)|(I_BLEED_N<<13)|RFB<<7|(0x35000076)); //R6      0X35000076 == R6 avec RF_divider et I_BLEED
   //TEST=((RF_Select<<21)|(I_BLEED_N<<13)|0x35000076);
   //Serial.print("TEST=");Serial.println(TEST,HEX);
    Serial.println("");
   if (RFB==1) {Serial.println ("RFB = OFF");} else {Serial.println("RFB = ON");}         
   Serial.print("RF_Select=");Serial.println(RF_Select,DEC);// 
   Serial.print("MOD2 = ");Serial.println(MOD2[1]);
   Serial.print("FRAC2 = ");Serial.println(FRAC2R);
   WriteRegister32(4|R04_EN);                                 //R4  DB4=1
   WriteRegister32(2|(1000<<4)|(FRAC2R<<18));                 //R2
   Serial.print("FRAC1 = ");Serial.println(FRAC1R);
   WriteRegister32(1|(FRAC1R<<4));                            //R1
   Serial.print("INT= ");Serial.println(INTR);
   WriteRegister32(0|INTR<<4);                                //R0   
   WriteRegister32(R04);                                      //R4  DB4=0
   delay(3); //  pause de 3 ms pour stabilisation VCO
   WriteRegister32(0|0X200000|(INTR<<4));                     //R0               
}
 
// *************** SP ecriture Mot long (32bits) en EEPROM  entre adresse et adresse+3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition du long (32bits) en 4 bytes
      //trois = MSB -> quatre = lsb
      byte quatre = (value & 0xFF);
      byte trois = ((value >> 8) & 0xFF);
      byte deux = ((value >> 16) & 0xFF);
      byte un = ((value >> 24) & 0xFF);

      //Ecrit 4 bytes dans la memoire EEPROM
      EEPROM.write(address, quatre);
      EEPROM.write(address + 1, trois);
      EEPROM.write(address + 2, deux);
      EEPROM.write(address + 3, un);
      }

// *************** SP lecture Mot long (32bits) en EEPROM situe entre adress et adress+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long quatre = EEPROM.read(address);
      long trois = EEPROM.read(address + 1);
      long deux = EEPROM.read(address + 2);
      long un = EEPROM.read(address + 3);

      //Retourne le long(32bits) en utilisant le shift de 0, 8, 16 et 24 bits et des masques
      return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
      }

      
//************************************ Setup ****************************************
void setup() {
 
  Serial.begin(115200); 
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10,255); //Luminosite LCD

  Serial.begin (115200); //  Serial to the PC via Arduino "Serial Monitor"  at 115200
  lcd.print("   synthesizer   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF5355     ");
  poscursor = 11; line = 0; 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("   par F1CJN    ");
  delay(1000);

  pinMode(2, INPUT);  // PIN 2 en entree pour lire le lock signal
  pinMode(ADF5355_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF5355_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 et Clock positive
  SPI.setBitOrder(MSBFIRST);            // poids forts en tête

   if (EEPROM.read(250)==55){
     RFint=EEPROMReadlong(memoire*4);              // si EEPROM écrite , on la lit la première adresse memoire
     init64(FPFD,0,EEPROM.read((memoire*4)+80));
     HERTZ=EEPROMReadlong((memoire*4)+160);// lecture HERTZ en EEPROM
     if ((HERTZ >= 1000)|| (HERTZ<0)) HERTZ=0; 
     }
     else {FPFD[1]=26;RFint=144500;HERTZ=0; }
  //Serial.print("FPFD memoire = ");Serial.println(FPFD[1],DEC);prt(FPFD);

  RFintold=187500;//pour que RFintold soit different de RFout lors de l'init //////Avoir
  RFOUT = 187500 ; // fréquence de sortie en khz
  OutputChannelSpacing = 1; // Pas de fréquence = 1kHz

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);
  SetADF5355(); // Set avec 26 MHz

  RFintold=123456;
  RFB=1; //sortie RFB Off
} // Fin setup

//*************************************Loop***********************************
void loop()
{
  RFOUT=RFint;RFB=1;X2=0; // RFB off
  //RFOUT=187500;
  if ((RFint != RFintold)|| (modif==1)||(HERTZ!=HERTZold)) {
   if((53125<=RFOUT)&&(RFOUT<106250))   {RF_Divider=64;RF_Select=6;}  // RFOUT = Frequence de sortie en KHz
   if((106250<=RFOUT)&&(RFOUT<212500))  {RF_Divider=32;RF_Select=5;}  
   if((212500<=RFOUT)&&(RFOUT<425000))  {RF_Divider=16;RF_Select=4;}
   if((425000<=RFOUT)&&(RFOUT<850000))  {RF_Divider=8;RF_Select=3;} 
   if((850000<=RFOUT)&&(RFOUT<1700000)) {RF_Divider=4;RF_Select=2;}
   if((1700000<=RFOUT)&&(RFOUT<3400000)){RF_Divider=2;RF_Select=1;}
   if((3400000<=RFOUT)&&(RFOUT<6800000)){RF_Divider=1;RF_Select=0;}
   if((6800000<=RFOUT)&&(RFOUT<=13600000)){RF_Divider=1;RF_Select=0;X2=1;RFB=0;} // et mettre la sortie X2 à 1 ////////////////////
   // MOD1=16777216;
   // MOD2=1000;
 FVCO[1]=RFint*RF_Divider;
 FPFDTEMP[0]=FPFD[0]; FPFDTEMP[1]=FPFD[1];
 FVCOTEMP[0]=FVCO[0]; FVCOTEMP[1]=FVCO[1];
 
 init64(N,0,10000);
 mul64(FVCO,N);

 init64(N,0,2);               // Preparation pour sortie X2 si 6.8GHZ < RFOUT < 13.6GHz
 if (X2==1){div64(FVCO,N);}; // on divise par 2 pour  6.8GHZ < RFOUT < 13.6GHz afin que FVCO reste entre 3.4 et 6.8GHz
 
 init64(HZ,0,RF_Divider*10*HERTZ); 
 init64(N,0,2);     // nouvel init à N=2  car div64 modififie N
 if (X2==1){div64(HZ,N);};
 add64(FVCO,HZ);   // ajout des µHz

 init64(N,0,10000);
 mul64(FVCO,N);
 div64(FVCO,FPFD);
 N[0]=FVCO[0];N[1]=FVCO[1];
 INT[0]=N[0];INT[1]=N[1];
 init64(NX,0,10E6);
 div64(INT,NX);
 init64(NX,0,10000);
 div64(INT,NX);
 INTR=(INT[1]);
 init64(NX,0,10E4);
 mul64(INT,NX);
 init64(NX,0,10E5);
 mul64(INT,NX);
 sub64(N,INT);
 mul64(N,MOD1);
 FRAC1exact[0]=N[0];FRAC1exact[1]=N[1];
 init64(NX,0,10E7);
 div64(N,NX);
 init64(NX,0,1000);
 div64(N,NX);
  FRAC1R=N[1];
  FRAC11[0]=N[0];FRAC11[1]=N[1];
  init64(N10E6,0,1000000);//prt(N10E6);
  mul64(FRAC11,N10E6);
  init64(N10E3,0,100000);//prt(N10E3);
  mul64(FRAC11,N10E3);
 sub64(FRAC1exact,FRAC11);
 init64(NX,0,10000);
 div64(FRAC1exact,NX);
 init64(NX,0,1000);
 div64(FRAC1exact,NX);
 init64(NX,0,5);// pour realiser un arrondi correct
 add64(FRAC1exact,NX);
 init64(NX,0,10); 
 div64(FRAC1exact,NX);
  FRAC2R=FRAC1exact[1];
  FPFD[0]=FPFDTEMP[0]; FPFD[1]=FPFDTEMP[1];
  FVCO[0]=FVCOTEMP[0]; FVCO[1]=FVCOTEMP[1];
  I_BLEED_N=7;  ///
 
    RFintold=RFint;modif=0;
    HERTZold=HERTZ;
    updateADF5355(); 
    printAll();  // Affichage LCD
  Serial.println("Update"); 
  }

  lcd_key = read_LCD_buttons();  // read the buttons
  //Serial.print("FPFD memoire button = ");Serial.println(FPFD[1],DEC);
  switch (lcd_key)               // Select action
  {
    case btnRIGHT: //Droit
      poscursor++; // cursor to the right
      if (line == 0) {
        if (poscursor == 10 ) { poscursor = 11;   line = 0; } //si curseur sur le .
        if (poscursor == 6 ) { poscursor = 7;   line = 0; } //si curseur sur le .
        if (poscursor == 2 ) { poscursor = 3;   line = 0; } //si curseur sur le .
        if (poscursor == 14 ) {poscursor = 0; line = 1; }; //si curseur à droite
      }
     if (line == 1) {
        if (poscursor == 1 ) {poscursor = 5; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor == 6 ) {poscursor = 15; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor==16) {poscursor=1; line=0;};     
      }  
      //Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //Gauche
      poscursor--; // décalage curseur
      if (line == 0) {
        if (poscursor == 255) {poscursor = 15; line = 1;  };
        if (poscursor == 10) {   poscursor = 9; line=0;}
        if (poscursor == 6) {   poscursor = 5; line=0;}
        if (poscursor == 2) {   poscursor = 1; line=0;}
        
      }
       if(line==1){
          if (poscursor==255) {poscursor=13; line=0;};
          if (poscursor==4) {poscursor=0; line=1;};
          if (poscursor==14) {poscursor=5; line=1;};
      }
      //Serial.print(poscursor,DEC);  
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP: //Haut
      if (line == 0)
      { // RFoutfrequency     
        if (poscursor == 0) RFint = RFint + 10000000 ;
        if (poscursor == 1) RFint = RFint + 1000000 ;
        if (poscursor == 3) RFint = RFint + 100000 ;
        if (poscursor == 4) RFint = RFint + 10000 ;
        if (poscursor == 5) RFint = RFint + 1000 ;
        if (poscursor == 7) RFint = RFint + 100 ;
        if (poscursor == 8) RFint = RFint + 10 ;
        if (poscursor == 9) RFint = RFint + 1 ;
        if (poscursor == 11) HERTZ = HERTZ + 100 ;
        if (poscursor == 12) HERTZ = HERTZ + 10 ;
        if (poscursor == 13) HERTZ = HERTZ + 1 ;
        if (HERTZ >= 1000) {HERTZ=HERTZ-1000; RFint=RFint+1;} 
        if (RFint > 13600000)RFint = RFintold;
      }
      if (line == 1)
      { 
        if (poscursor == 5){ memoire++; 
        if (memoire==20)memoire=0;
        if (WEE==0){
           RFint=EEPROMReadlong(memoire*4); // lecture EEPROM et Affichage
           if (RFint>13600000) RFint=13600000; 
           HERTZ=EEPROMReadlong((memoire*4)+160);// lecture HERTZ en EEPROM
           if ((HERTZ >= 1000)|| (HERTZ<0)) HERTZ=0; 
           init64(FPFD,0,EEPROM.read((memoire*4)+80));// lecture FPFD en EEPROM
          if (FPFD[1]!=26 && FPFD[1]!=10) {FPFD[1]=26;}  
           } 
        }  
        if (poscursor==15){ 
        if(FPFD[1]==10){FPFD[1]=26;} //reglage FREF
        else if ( FPFD[1]==26){FPFD[1]=10;}
        //else if (FPFD[1]!=26 || FPFD[1]!=10) {FPFD[1]=26;} // au cas ou FPFD different de 10 et 26
       else FPFD[1]=26;// au cas ou FPFD different de 10 et 26
       // Serial.print(FPFD[1],DEC);Serial.print("\r\n");
        modif=1;  }
                    
      if( (poscursor==0) && (WEE==1))WEE=0;
      else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
        printAll();
      break; // fin bouton up

    case btnDOWN: //bas
      if (line == 0) {
        if (poscursor == 0) RFint = RFint - 10000000 ;
        if (poscursor == 1) RFint = RFint - 1000000 ;
        if (poscursor == 3) RFint = RFint - 100000 ;
        if (poscursor == 4) RFint = RFint - 10000 ;
        if (poscursor == 5) RFint = RFint - 1000 ;
        if (poscursor == 7) RFint = RFint - 100 ;
        if (poscursor == 8) RFint = RFint - 10 ;
        if (poscursor == 9) RFint = RFint - 1 ;
        if (poscursor == 11) {HERTZ = HERTZ - 100;}
        if (poscursor == 12) {HERTZ = HERTZ - 10 ;}
        if (poscursor == 13) {HERTZ = HERTZ - 1 ;}
        if (HERTZ < 0) {HERTZ=1000+HERTZ; RFint=RFint-1;} 
          
        if (RFint < 54000) RFint = RFintold;
        if (RFint > 13600000)  RFint = RFintold;
        printAll();
        break;
      }

     if (line == 1)
      { 
        if (poscursor == 5){memoire--; 
        if (memoire==255)memoire=19;
        if (WEE==0){
           RFint=EEPROMReadlong(memoire*4); // lecture RFint en EEPROM et Affichage
           if (RFint>13600000) RFint=13600000;
           init64(FPFD,0,EEPROM.read((memoire*4)+80));         // lecture FPFD en EEPROM
           HERTZ=EEPROMReadlong((memoire*4)+160);            // lecture HERTZ en EEPROM
           if ((HERTZ >= 1000)|| (HERTZ<0)) HERTZ=0;  
           if (FPFD[1]!=26 && FPFD[1]!=10) {FPFD[1]=26;}  
           } 
           
        } // fin poscursor =5 

       if (poscursor==15){ 
       if( FPFD[1]==10){FPFD[1]=26;} //reglage FREF
       else if (FPFD[1]==26){FPFD[1]=10;}
       else if (FPFD[1]!=26 && FPFD[1]!=10) {FPFD[1]=26;} // au cas ou PFDRF different de 10 et 26
       
       modif=1;
       }
                   
       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      
      printAll();
      break; // fin bouton bas
      }

    case btnSELECT:
      do { adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer inc toutes les 1 millisecondes
         if (timer2 > 600) {      //attente 600 millisecondes
         
         if (line==1 && poscursor==0) {                      // position 0 sur ligne 1   
         EEPROMWritelong(((memoire*4)+80),FPFD[1]);          // ecriture FPFD réference en EEPROM (memoire*4)+80
         EEPROMWritelong(memoire*4,RFint);                   // ecriture RFint en EEPROM à adresse (memoire*4)
         EEPROMWritelong(((memoire*4)+160),HERTZ);           // ecriture HERTZ en EEPROM à adresse (memoire*4)+160
         EEPROM.write(250,55);                               // ecriture RF en EEPROM à adresse 250
         lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");
         lcd.setCursor(poscursor,line);}
         delay(500);timer2=0;
         printAll();
        }; // 

        } //  fin do
        
      while (adc_key_in < 900); // attente relachement
      break;  // Fin bouton Select

     case btnNONE: {
        break;
      };
      break;
  }// Fin LCD keys

   if (lcd_key != btnNONE)  // If a key is pressed                  ///////////////correction bug us /////////////
   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900); // attente relachement touche
   delay (10);timer++; // inc timer
   //Serial.print(timer,DEC);
   if (timer>1000){lcd.noBlink();timer=0;} // curseur off

}   // fin loop


