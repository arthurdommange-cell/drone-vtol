/* Projet tilt-rotor : ROULIS SUR BALANCIER UNIQUEMENT.
 * Nano 33 BLE Rev2, coeur Arduino Mbed OS Nano Boards.
 * Bibliotheque : Arduino_BMI270_BMM150.
 * Moteurs inhibes par defaut. Lire le guide avant activation.
 * Pas de maintien d'altitude, de tangage, de lacet ni de transition.
 * Sortie du PID en microsecondes ESC, PAS en N.m.
 */
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <mbed.h>
#include <math.h>

// A confirmer sans helices. Aucun calibrage ESC automatique.
constexpr bool CONFIG_VERIFIEE = false;
constexpr int PIN_ESC_G = 9, PIN_ESC_D = 10;
constexpr int PIN_MAINTIEN = 4, PIN_ARMER = 5, PIN_GAZ = A0;
constexpr int ESC_STOP_US = 1000;
constexpr int ESC_PLAFOND_US = 1300; // limite initiale d'essai, PAS une valeur universelle
constexpr int ESC_PERIODE_US = 20000; // 50 Hz, seulement si accepte par vos ESC
constexpr float RAMPE_US_S = 60.0f;
constexpr float CORRECTION_MAX_US = 60.0f;
// Repere IMU : x longitudinal, y transversal, z vertical positif au repos.
// Adapter les signes ET les axes si le montage ne respecte pas ce repere.
constexpr float SIGNE_GYRO = 1.0f, SIGNE_ANGLE = 1.0f;
constexpr float SIGNE_MIXAGE = 1.0f; // verifier que la correction ramene l'angle vers zero
constexpr bool BOUCLE_ANGLE = false; // false : amortissement de vitesse ; true : retour a 0 deg
constexpr float KP_ANGLE = 0.0f; // (deg/s)/deg ; regler APRES la boucle de vitesse
constexpr float KP_VITESSE = 0.0f; // us/(deg/s) ; a identifier
constexpr float KI_VITESSE = 0.0f; // us/deg
constexpr float KD_VITESSE = 0.0f; // us/(deg/s^2)
constexpr float VITESSE_MAX = 25.0f; // deg/s
constexpr float ANGLE_COUPURE = 20.0f; // banc avec butees au-dela de ce seuil
constexpr float TAU_ANGLE = 0.5f, TAU_DERIVEE = 0.04f;
constexpr uint32_t CAPTEUR_TIMEOUT_US = 60000;
constexpr int N_CAL = 300;

mbed::PwmOut *escG = nullptr, *escD = nullptr;
float angleDeg=0, vitesse=0, biais=0, somme=0, somme2=0;
float iTerme=0, derivee=0, precedenteVitesse=0;
float baseUs=ESC_STOP_US, correction=0;
float ax=0, ay=0, az=1;
int nCal=0, outG=ESC_STOP_US, outD=ESC_STOP_US;
bool pret=false, defaut=false, arme=false, armAvant=false, angleInit=false;
uint32_t debutMs=0, dernierGyro=0, dernierAccel=0, dernierLog=0;

float borne(float x,float a,float b){ return fmaxf(a,fminf(b,x)); }
void sortie(int g,int d){
  outG=g; outD=d;
  if(escG) escG->pulsewidth_us(g);
  if(escD) escD->pulsewidth_us(d);
}
void desarmer(){
  arme=false; baseUs=ESC_STOP_US; iTerme=0; derivee=0; correction=0;
  sortie(ESC_STOP_US,ESC_STOP_US);
}
void panne(){ defaut=true; desarmer(); } // reinitialisation requise

void setup(){
  pinMode(PIN_MAINTIEN,INPUT_PULLUP); // contact ferme vers GND = autorisation
  pinMode(PIN_ARMER,INPUT_PULLUP);    // bouton poussoir vers GND
  pinMode(LED_BUILTIN,OUTPUT);
  analogReadResolution(12);
  Serial.begin(115200); // pas de while(!Serial) bloquant
  escG=new mbed::PwmOut(digitalPinToPinName(PIN_ESC_G));
  escD=new mbed::PwmOut(digitalPinToPinName(PIN_ESC_D));
  escG->period_us(ESC_PERIODE_US); escD->period_us(ESC_PERIODE_US);
  desarmer();
  debutMs=millis(); dernierGyro=dernierAccel=micros();
  if(!IMU.begin()) panne();
  dernierGyro=dernierAccel=micros(); debutMs=millis();
}

void loop(){
  uint32_t now=micros();
  bool maintien=digitalRead(PIN_MAINTIEN)==LOW;
  bool bouton=digitalRead(PIN_ARMER)==LOW;
  bool frontArm=bouton && !armAvant;
  armAvant=bouton;
  int pot=analogRead(PIN_GAZ); // potentiometre 10 kohm entre 3V3 et GND
  if(!maintien || !CONFIG_VERIFIEE) desarmer();

  if(!defaut && IMU.accelerationAvailable()){
    if(!IMU.readAcceleration(ax,ay,az) || !isfinite(ax) || !isfinite(ay) || !isfinite(az)) panne();
    else dernierAccel=now;
  }
  if(!defaut && IMU.gyroscopeAvailable()){
    float gx,gy,gz;
    if(!IMU.readGyroscope(gx,gy,gz) || !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {panne(); return;}
    uint32_t ecoule=now-dernierGyro;
    dernierGyro=now;
    float dt=ecoule*1e-6f;
    float norme=sqrtf(ax*ax+ay*ay+az*az);
    float angleAcc=SIGNE_ANGLE*atan2f(ay,az)*180.0f/PI;
    if(!pret){
      // Carte horizontale et immobile : sinon recommencer l'accumulation.
      bool immobile=fabsf(gx)<3 && fabsf(gy)<3 && fabsf(gz)<3 &&
                    norme>0.9f && norme<1.1f && fabsf(angleAcc)<5 && az>0 &&
                    now-dernierAccel<CAPTEUR_TIMEOUT_US;
      if(!immobile){nCal=0;somme=0;somme2=0;}
      else {
        somme+=gx; somme2+=gx*gx; nCal++;
        if(nCal>=N_CAL){
          biais=somme/N_CAL;
          float variance=fmaxf(0,somme2/N_CAL-biais*biais);
          if(variance>0.25f) panne();
          else {pret=true;angleDeg=angleAcc;angleInit=true;precedenteVitesse=0;}
        }
      }
    } else {
      if(dt<=0 || dt>0.05f){panne();return;}
      vitesse=SIGNE_GYRO*(gx-biais);
      angleDeg+=vitesse*dt;
      // L'accelerometre n'est utilise que si sa norme reste proche de g.
      if(norme>0.85f && norme<1.15f){
        float a=TAU_ANGLE/(TAU_ANGLE+dt);
        angleDeg=a*angleDeg+(1-a)*angleAcc;
      }
      float ad=TAU_DERIVEE/(TAU_DERIVEE+dt);
      derivee=ad*derivee+(1-ad)*(vitesse-precedenteVitesse)/dt;
      precedenteVitesse=vitesse;
      if(!isfinite(angleDeg) || fabsf(angleDeg)>ANGLE_COUPURE) panne();
      if(arme && !defaut){
        float cible=ESC_STOP_US+(ESC_PLAFOND_US-ESC_STOP_US)*(pot/4095.0f);
        // Augmentation progressive ; diminution immediate de la base.
        baseUs=(cible<baseUs)?cible:fminf(cible,baseUs+RAMPE_US_S*dt);
        float consV=BOUCLE_ANGLE?borne(-KP_ANGLE*angleDeg,-VITESSE_MAX,VITESSE_MAX):0;
        float err=consV-vitesse;
        float marge=fminf(CORRECTION_MAX_US,fminf(baseUs-ESC_STOP_US,ESC_PLAFOND_US-baseUs));
        marge=fmaxf(0,marge);
        float pd=KP_VITESSE*err-KD_VITESSE*derivee;
        float candidat=borne(iTerme+KI_VITESSE*err*dt,-CORRECTION_MAX_US,CORRECTION_MAX_US);
        float brut=pd+candidat;
        // Integration conditionnelle : pas d'accumulation vers une saturation.
        if((brut<=marge && brut>=-marge) || (brut>marge && err<0) || (brut<-marge && err>0)) iTerme=candidat;
        if(pot<40) iTerme=0;
        correction=borne(pd+iTerme,-marge,marge);
        float diff=SIGNE_MIXAGE*correction;
        sortie((int)lroundf(baseUs+diff),(int)lroundf(baseUs-diff));
      }
    }
  }
  now=micros();
  if(!defaut && (now-dernierGyro>CAPTEUR_TIMEOUT_US || now-dernierAccel>CAPTEUR_TIMEOUT_US)) panne();
  if(!pret && millis()-debutMs>15000) panne();
  // Appuyer APRES pret, gaz au minimum ; un bouton reste enfonce ne rearme pas.
  if(frontArm && CONFIG_VERIFIEE && pret && angleInit && !defaut && maintien &&
     pot<40 && fabsf(angleDeg)<5 && millis()-debutMs>3000){
    desarmer(); arme=true;
  }
  if(!arme) sortie(ESC_STOP_US,ESC_STOP_US);
  digitalWrite(LED_BUILTIN,arme?HIGH:LOW);
  // CSV a 10 Hz, abandon de ligne si le port USB manque de place.
  // ms,pret,defaut,arme,angle_deg,vitesse_deg_s,gauche_us,droite_us
  if(now-dernierLog>=100000 && Serial){
    dernierLog=now;
    char ligne[120];
    int n=snprintf(ligne,sizeof(ligne),"%lu,%d,%d,%d,%.3f,%.3f,%d,%d\n",
      (unsigned long)millis(),pret,defaut,arme,(double)angleDeg,(double)vitesse,outG,outD);
    if(n>0 && n<(int)sizeof(ligne) && Serial.availableForWrite()>=n) Serial.write((const uint8_t*)ligne,n);
  }
}
