#include "SPI.h"

#define NB_MES_ANA 8      // Nb de mesures analogiques
#define NB_MES     10     // Nb total de mesures
#define FMT        "%5d"  // Format données dans message
#define DTLG       6      // Nb d'octets par variable dans le messsage
#define PERIODE    10     // Lecture des mesures toutes les 10 ms
#define NB_CYCLE   50     // Envoi message toutes les 50 mesures (500 mS)
#define NB_FORCAGE 10     // Forcage envoi mesure dans bande morte tous les 10 messages (5 Sec)
#define RPM_DELAY  100000 // Periode de mesure des RPM (en micro sec)
#define RPM_NB_TIC 1      // Nombre d'impulsion par tour pour la mesure des RPM


typedef struct
{
  float   A;        float     B;  // coefficients de mise à l'echelle y=Ax+B
  float Filt;       float   Val;  // Constante de temps de filtrage, Valeur mesure 
  int   Last;       int     Cpt;  // Memo dernière valeur envoyée, compteur pour forcage envoi
  int   DeadB;      int     Pos;  // Bande morte pour envoi, Position dans le message
}
StrMes ;      // Structure de stockage d'une valeur de mesure

static StrMes  AnaMes[NB_MES_ANA];         // Table des données analogique
static StrMes EGTMes;                      // Mesure EGT 
static StrMes RpmMes;                      // Mesure RPM
static long Cpt ;                          // Compteur de vie
static long NbTicRpm ;                     // Compteur de tic front montant entrée vitesse
static unsigned long MicroSecMem ;         // Memorisation micro secondes comptage
static long NbCycle ;                      // Compteur de cycle pour envoi message
static char SendBuffer[DTLG*(NB_MES+1)+1]; // Buffer d'emission


int PCS = 10;                              // Numero de la sortie chip select pour module SPI MAX6675

static float ATCT [16];                    // Table de correction de temperature sonde Air
static float WTCT [16];                    // Table de correction de temperature sondes Eau huile

// Programme d'initialisation ****************************************************
void setup() {

  Serial.begin(4800);

  analogReference(EXTERNAL);
  
  // Initialisation des mesures analogiques
  int    i;
  i = 0;

  // Pour les valeurs de filtrage : Filt = T/PERIODE avec T = Constante en secondes (si PERIODE = 10mS Filt=100*T) 
  // Entrée arduino Nano 0V = 0 5V=1023 204,6 pt/Volt

  // A0 = Courant batterie 1.56/2.94 V => -10/10 Amperes => 320 /602 points => A=0.0814 B=-41.7(* 10) => A=0.814 B=-417 (offset sur le 0 en pratique -371)
  //AnaMes[i].A = 0.814;  AnaMes[i].B = -371;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 50 ; AnaMes[i].DeadB = 2; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1;  i++;
  // A0 = Pression Gazoil  0.5/4.5V => 0/1034 kPa (150 PSI) => A=1.2616 B= -86 (*1) 
  AnaMes[i].A = 1.2634;  AnaMes[i].B = -129;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 50 ; AnaMes[i].DeadB = 10 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A1 = Temperature Air 4.57/1.111 V => 0/110 °C  => A=-0.1553 B=145 (* 1) 
  AnaMes[i].A = -0.1553;  AnaMes[i].B = 145;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 100 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A2 = Pression Air 0.25/4.75V => 25/369 KPa => A=0.3733 B=6 (* 1) en pratique offset 8 => B=14
  AnaMes[i].A = 0.3733;  AnaMes[i].B = 14;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 50 ; AnaMes[i].DeadB = 5 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A3 = Pression Huile 0.5/4.5V => 0/689 kPa (100 PSI) => A=0.8411 B= -86 (*1) 
  AnaMes[i].A = 0.8411;  AnaMes[i].B = -86;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 50 ; AnaMes[i].DeadB = 10 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A4 = Temperature Huile  0.75/1.75V => -25/125 °C => A=0.4883 B=-50 (*1)
  AnaMes[i].A = -0.1355;  AnaMes[i].B = 128.44;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 100 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A5 = Temperature Eau  0.75/1.75V => -25/125 °C =>  A=0.4883 B=-50 (*1)
  AnaMes[i].A = -0.1355;  AnaMes[i].B = 128.44;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 100 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A6 = Tension Batterie 0/20,55V => 0/5 V =>  A=0.02 B=0 (*10) A=0.2
  AnaMes[i].A = 0.2;  AnaMes[i].B = 0;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 30 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A7 = Temperature compartiment moteur 0.75/1.75V => -25/125 °C => A=0.4883 B=-50 (*1)
  AnaMes[i].A = 0.4883;  AnaMes[i].B = -50;   AnaMes[i].Val = 0;  AnaMes[i].Filt = 100 ; AnaMes[i].DeadB = 1 ; AnaMes[i].Last = 0 ; AnaMes[i].Cpt = 0 ; AnaMes[i].Pos = i+1; i++;

  // A8 = EGT acquisition SPI (*1)
  EGTMes.A = 1;  EGTMes.B = 0;   EGTMes.Val = 0;  EGTMes.Filt = 1 ; EGTMes.DeadB = 100 ; EGTMes.Last = 0 ; EGTMes.Cpt = 0 ; EGTMes.Pos = i+1; i++;

  // A9 Vitesse moteur Interruption D2  
  RpmMes.A = 1;  RpmMes.B = 0;   RpmMes.Val = 0;  RpmMes.Filt = 0 ; RpmMes.DeadB = 10 ; RpmMes.Last = 0 ; RpmMes.Cpt = 0 ; RpmMes.Pos = i+1;

  // Les mesures de temperature sont effectuées par des ponts diviseur. La tension du pont est très approximativement proportionnelle à la temperature
  // Les tables de correction permettent de convertir la temperature approximative en temperature vraie
  // Les tables de correction sont prévues avec index 0 = correction à -10°c, index 1 = correction à 0 °c, ... index 15 = correction à 140°c

  // Initialisation correction sonde de temperature d'air (Sonde NTC 10K à 25°C B=3440 alimentée par  R=2.7 K)
  // Note : Pont resistif calé pour etre juste à  0°c, 50° et 110 °C
  ATCT [0] = -22;   ATCT [1] = 0;     ATCT [2] = 14;    ATCT [3] = 24;   
  ATCT [4] = 33;    ATCT [5] = 42;    ATCT [6] = 50;    ATCT [7] = 58;
  ATCT [8] = 66;    ATCT [9] = 75;    ATCT [10] = 85;   ATCT [11] = 96;
  ATCT [12] = 110;  ATCT [13] = 128;  ATCT [14] = 157;  ATCT [15] = 215;

 // Initialisation correction sonde de temperature d'eau huile (Sonde NTC 10K à 25°C B=3950 alimentée par  R=2.7 K))
 // Note : Pont resistif calé pour etre juste à  0°c , 50° et 100 °C
  WTCT [0] = -25;   WTCT [1] = 0;    WTCT [2] = 15;      WTCT [3] = 25;   
  WTCT [4] = 34;    WTCT [5] = 42;   WTCT [6] = 50;      WTCT [7] = 58;
  WTCT [8] = 66;    WTCT [9] = 75;   WTCT [10] = 86;     WTCT [11] = 100;
  WTCT [12] = 119;  WTCT [13] = 151; WTCT [14] = 250;    WTCT [15] = 265;

  
  // Initialisation variables globales
  NbTicRpm = 0;             // RAZ variables statiques à l'init
  MicroSecMem = 0;
  Cpt = 0;
  NbCycle = 0;

  // Initialiser le bus SPI
  pinMode(PCS, OUTPUT); 
  digitalWrite(PCS, HIGH);
  SPI.begin ();
    
  // Configuration Interruption D2 front montant
  noInterrupts();
  attachInterrupt(0, TicInc, RISING); //attache l'interruption externe n°0 à la fonction "TicInc"
  interrupts();
}

// routine d'interruption externe n°0
// Comptage du nb de tic et si plus de x ms depuis le dernier calcul, calcul de la vitesse de rotation en mesurant les micro secondes écoulées
// si le compteur de micro secondes interne arduino est repassé à zéro on ignore ce cycle de cmptage
void TicInc ()
{
  unsigned long MicroSec;

  MicroSec = micros();
  if (MicroSec < MicroSecMem) // compteur de micro seconde interne arduino repassé à zéro => RAZ sequence de comptage
  {
    NbTicRpm = 0;  
    MicroSecMem = 0;
  }
  else
  {
    NbTicRpm++ ;
    if ((MicroSec-MicroSecMem) > RPM_DELAY)
    {    
      RpmMes.Val =  60000000*(float)NbTicRpm/((float)(MicroSec-MicroSecMem)*RPM_NB_TIC);
      NbTicRpm = 0;
      MicroSecMem = MicroSec;
    }
  }

}


// Correction de temperature sonde (Recoit la température mesurée, la table de correction et renvoie la valeur corrigée)
float TCorr ( float Val, float TCT [])
  {
    int Index;
    float T0;
    float TC;
    
    // Calcul de l'index 
    Index =  int (Val/10) +1;
    if (Index > 14) TC = TCT[15];
    else
        {
        if (Index <0) TC = TCT[0];
        else
          {
             // Interpolation linéaire 
            T0 = 10*(Index-1);
            TC = (TCT[Index + 1] - TCT[Index])*(Val - T0) / 10 +  TCT[Index] ;
          }
        }
    return(TC);
  }

// Stockage d'une donnée dans le buffer démission avec anti bagottement
void SendMes ( StrMes * AnaMes )
  {
    int ValInt;
    int ValGap;
    char FmtVal[6];
    
    // Anti bagottement: On envoie une nouvelle valeur que si ecart supérieur à deadBand ou à 10 cycles (5 secondes)
    ValInt = AnaMes->Val ;
    ValGap = abs (ValInt -  AnaMes->Last) ;
    if (ValGap == 0) AnaMes->Cpt=0;
    else
      if (ValGap <= AnaMes->DeadB )
        { 
        if (AnaMes->Cpt++ > NB_FORCAGE )
          {
          AnaMes->Last = ValInt ;
          AnaMes->Cpt = 0;
          }
        }
       else
        {
        AnaMes->Last = ValInt ;
        }
        
    //Formater la donnée suivie d'un espace dans le buffer d'émission
    sprintf(&SendBuffer[AnaMes->Pos*DTLG], FMT, AnaMes->Last);
    SendBuffer[AnaMes->Pos*DTLG + DTLG - 1] = 32;
  }


// Programme principal *****************************************************************

void loop()
{

  int   i;
  int   k;
  float Val;
  int   ValInt;
  int   ValGap;
  char  FmtVal[6] ;
  short Reg;
  char  str[20];

  // La boucle est cadencée à 10ms
  delay (PERIODE);

  // lecture des entrées analogiques  mise a l'echelle et filtrage  
  for (i = 0; i < NB_MES_ANA; i++)
  {
    Val = ((float) analogRead(i)) *  AnaMes[i].A + AnaMes[i].B;

    switch(i)
   {
    // Pour les NTC, appliquer la correction de temperature
    case 1: Val = TCorr (Val,ATCT);break;
    case 4: 
    case 5: Val = TCorr (Val,WTCT);break;
   }
    AnaMes[i].Val = ( AnaMes[i].Filt * AnaMes[i].Val + Val) / (1 + AnaMes[i].Filt);
  }

  // Lecture temperature EGT
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

  // Envoi des valeurs tous les 500ms
  if ( NbCycle++ > NB_CYCLE )
    {
    NbCycle = 0;
    
    // incrémenter le compteur de vie 0 à 99
    if (Cpt++ > 99)     Cpt = 0;

    // Ecriture  du cpt de vie suivi d'un espace dans le buffer d'émission
    sprintf (&SendBuffer[0], FMT, Cpt);
    SendBuffer[DTLG-1] = 32;
    SendBuffer[DTLG*(NB_MES+1)] = 0;

    // Ecriturede des valeurs de mesure dans le buffer d'émission
     for (i = 0; i < NB_MES_ANA ; i++)     SendMes (&AnaMes[i]);
    SendMes (&EGTMes);
    SendMes (&RpmMes);

    // Ecriture Fin de chaine
    SendBuffer[DTLG *(NB_MES+1)-1] = 0;

    // Envoi du message sur la ligne série
    Serial.println (SendBuffer);
    
    }

}
