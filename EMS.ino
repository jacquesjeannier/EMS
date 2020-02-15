#include "SPI.h"

#define NB_MES_ANA 8 // Nb de mesures analogiques


typedef struct
{
  float   A;        float     B;
  float Filt;       float   Val;
  int   Last;       int     Cpt;
  int   DeadB;
}
StrMes ;      // Structure de stockage d'une valeur de mesure

static StrMes  AnaMes[NB_MES_ANA];       // Table des données analogiques
static StrMes EGTMes;                    // Mesure EGT 
static StrMes RpmMes;                    // Mesure RPM
static long Cpt ;                        // Compteur de vie
static byte Tim4ms ;                     // Comptage des timer 4 ms pour top à 500 mS
static long NbTicRpm ;                   // Compteur de tic front montant entrée vitesse


#include "SPI.h"
int PCS = 10;                           // Numero de la sortie chip select pour module SPI MAX6675


// Programme d'initialisation ****************************************************
void setup() {

  Serial.begin(4800);

  // Initialisation des mesure analogiques
  int    i;

  i = 0;


  // A0 = Courant batterie 1.56/2.94 V => -10/10 Amperes => 320 /602 points => A=0.0708 B=-33(* 10) => A=0.708 B=-330
  AnaMes[i].A = 0.708;  AnaMes[i].B = -328;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 0.5 ; AnaMes[i].DeadB = 2; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A1 = Temperature Air 3.74/0.9 V => 0/110 °C => 765 /184 points => A=-0.1891 B=145 (* 1)
  AnaMes[i].A = -0.1891;  AnaMes[i].B = 145;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 1 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A2 = Pression Air 0.25/4.75V => 25/369 KPa=> 51 /973 points => A=0.3733 B=6 (* 1)
  AnaMes[i].A = 0.3733;  AnaMes[i].B = 14;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 0.5 ; AnaMes[i].DeadB = 5 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A3 = Pression Huile 0.5/4.5V => 0/689 kPa (100 PSI) => 102 /921 points => A=0.8411 B=-86 (*1)
  AnaMes[i].A = 0.8411;  AnaMes[i].B = -86;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 0.5 ; AnaMes[i].DeadB = 10 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A4 = Temperature Huile  0.75/1.75V => -25/125 °C => 154 /358 points => A=0.4883 B=-50 (*1)
  AnaMes[i].A = 0.4883;  AnaMes[i].B = -50;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 1 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A5 = Temperature Eau  0.75/1.75V => -25/125 °C => 154 /358 points => A=0.4883 B=-50 (*1)
  AnaMes[i].A = 0.4883;  AnaMes[i].B = -50;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 1 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A6 = Tension Batterie 0/5V => 0/28 V => 102 /921 points => A=0.0273 B=0 (*10) A=0.273
  AnaMes[i].A = 0.294;  AnaMes[i].B = 0;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 0.3 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A7 = Temperature compartiment moteur 0.75/1.75V => -25/125 °C => 154 /358 points => A=0.4883 B=-50 (*1)
  AnaMes[i].A = 0.4883;  AnaMes[i].B = -50;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 1 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; i++;

  // A8 = EGT acquisition SPI (*1)
  EGTMes.A = 1;  EGTMes.B = 0;   EGTMes.Val = 0;  EGTMes.Filt = 1 ; EGTMes.DeadB = 1 ; EGTMes.Last = 0 ; EGTMes.Cpt = 0 ;

  // A9 Vitesse moteur Interruption D2  et Timer 2 (*1) (500 ms)
  RpmMes.A = 1;  RpmMes.B = 0;   RpmMes.Val = 0;  RpmMes.Filt = 0 ; RpmMes.DeadB = 50 ; RpmMes.Last = 0 ; RpmMes.Cpt = 0 ;


  noInterrupts();

  // configuration Timer 2 a 4 mS
  bitClear (TCCR2A, WGM20); // WGM20 = 0 Mode Normal
  bitClear (TCCR2A, WGM21); // WGM21 = 0

  TCCR2B = 0b00000110;      // Clock / 256 soit 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001;      // Interruption locale autorisée par TOIE2
  TCNT2 = 6;                // PreChargement du timer à 6 => Overflow à 250 * 16uS = 4 mS
  Tim4ms = 0;               // RAZ compteur 4mS

  // Configuration Interruption D2 front montant
  attachInterrupt(0, TicInc, RISING); //attache l'interruption externe n°0 à la fonction "TicInc"

  // Initialisation variables 
  NbTicRpm = 0;             // RAZ des variables statiques à l'init
  Cpt = 0;
  Tim4ms = 0;

  interrupts();           // Réactivation des interruptions

  // Initialiser le bus SPI
  pinMode(PCS, OUTPUT); 
  digitalWrite(PCS, HIGH);
  SPI.begin ();
}

// routine d'interruption externe n°0 pour comptage Rpm moteur
void TicInc ()
{
  NbTicRpm++ ;
}

// routine d'interruption du timer 2 : 125 * 4 ms =  500 ms pour mesurre Rpm moteur
#define TIC_WEIGHT 24   // 5 tic par tour => Ratio Rpm/Nb tics par sec = 12  soit 24 pour 500 mS

ISR (TIMER2_OVF_vect)
{
  TCNT2 = 6;
  if (Tim4ms++ == 125)
  {

    // SI 500 ms écoulé stocker le nb de tics avec filtrage dans la mesure rpm (arondi a 10) puis RAZ du compteur de tic
    
    RpmMes.Val = ( NbTicRpm * TIC_WEIGHT);
    RpmMes.Val = RpmMes.Val/10;
    RpmMes.Val = RpmMes.Val*10;
    NbTicRpm = 0;
    
    // RAZ compteur de temps
    Tim4ms = 0;
    
  }
}


// Envoi d'une donnée avec anti bagottement
void SendMes ( StrMes * AnaMes)
  {
    int ValInt;
    int ValGap;
    char FmtVal[6];
    // Anti bagottement: On envoie une nouvelle valeur que si ecart supérieur à 1 ou à 10 cycles (5 secondes)
    ValInt = AnaMes->Val ;
    ValGap = abs (ValInt -  AnaMes->Last) ;
    if (ValGap == 0) AnaMes->Cpt=0;
    else
      if (ValGap <= AnaMes->DeadB )
        { 
        if (AnaMes->Cpt++ > 10)
          {
          AnaMes->Last = ValInt ;
          AnaMes->Cpt = 0;
          }
        }
       else
        {
        AnaMes->Last = ValInt ;
        }
    sprintf(FmtVal, "%5d", AnaMes->Last);
    FmtVal[5] = 0;
    Serial.print (" ");
    Serial.print (FmtVal);
  }


// Programme principal *****************************************************************

void loop()
{

  int   i;
  float Val;
  int   ValInt;
  int   ValGap;
  char  FmtVal[6] ;
  short Reg;
  char  str[20];

  delay (500);
  
  // Compteur de vie 0 à 99
  if (Cpt++ > 99)     Cpt = 0;
  sprintf (FmtVal, "%5d", Cpt);
  FmtVal[5] = 0;
  Serial.print (FmtVal);

  // lecture des entrées analogiques,  mise a l'echelle ,  filtrage et envoi du message 
  for (i = 0; i < NB_MES_ANA; i++)
  {
    Val = ((float) analogRead(i)) *  AnaMes[i].A + AnaMes[i].B;
    AnaMes[i].Val = ( AnaMes[i].Filt * AnaMes[i].Val + Val) / (1 + AnaMes[i].Filt);
    SendMes ( &AnaMes[i]);
  }

  // Lecture temperature EGT
  //Val = TempEGT.read_temp();
  
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE1)); 
  digitalWrite(PCS, LOW);
  Reg = SPI.transfer16 (0x0);
  digitalWrite(PCS, HIGH);
  // Verifier si le thermocouple est ouvert (Bit 2)
  if ((Reg & 2) == 0)
    {      
    Val = Reg/16;    //1024 Degré 12 bit et 2 bit poids faible à ignorer => /16
    EGTMes.Val = ( EGTMes.Filt * EGTMes.Val + Val) / (1 + EGTMes.Filt);
    }
  else
    EGTMes.Val = 0;
  SendMes ( &EGTMes);

  // Envoi Mesure vitesse
  SendMes ( &RpmMes);

  Serial.println ("");

}
