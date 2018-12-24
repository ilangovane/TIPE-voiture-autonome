/*########################################################################
#####################VARIABLES GLOBALES #################################
#######################################################################
###########################################################*/
 #include <QTRSensors.h> 
//capteur infrarouge 
int ir1 = 30 ;//extremité droite 
int ir2 = 31 ; 
int ir3 = 32 ;
int ir4 = 33 ; 
int ir5 = 34 ; 
int ir6 = 35 ;
int ir7 = 36 ; 
int ir8 = 37 ; //extremité gauche  

bool ir[8];//état des capteurs 0 ou 1 
String priorite = "gauche";//"X":aucune prioritée ,"droite":prioritée à droite,"gauche":prioritée à gauche

String voie ; 
unsigned int  IR[8];//liste qui contiendra les temps de chute de tension de chaque capteur IR (8 au total)
bool sens[8];

QTRSensorsRC qtr(( unsigned char[]) {30,31,32,33,34,35,36,37},8, 3000);//création de l'objet qtr
 
//sens moteurs 
int dir_a=12 ;
int dir_b=13 ;

//frein moteurs 
int brake_a=9 ; 
int brake_b=8 ;

//vitesse moteurs 
int pwm_a=3;
int pwm_b=11;

//capteur ultrason
int trig = 52;//cette broche émet des ondes ultrasonores 
int echo = 53;//cette broche réceptionne des ondes 

//led RGB numéro 1 
int rouge1=6;
int vert1=10;
int bleu1=7;

//led RGB numéro 2 
int rouge2=2;
int vert2=5;
int bleu2=4;

//phototransistor (feu tricolore:la lampe vient éclairer le phototransistor)
int infra = 22;

void setup() {
  //hypothèse la voiture se trouve sur la voie 1
  voie = "1";
  //communication avec l'ordinateur (appuyer loupe à droite en haut )
  Serial.begin(9600);

  //configutation moteur A en sortie :(avancer / reculer) / vitesse / frein 
  pinMode(dir_a,OUTPUT);
  pinMode(pwm_a,OUTPUT);
  pinMode(brake_a,OUTPUT);
  //configutation moteur B en sortie :tourner à (droite / gauche) / vitesse / frein 
  pinMode(dir_b,OUTPUT);
  pinMode(pwm_b,OUTPUT);
  pinMode(brake_b,OUTPUT);
  digitalWrite(brake_b,HIGH);//frein roues arrieres (voiture immobile)
  //configuration capteur ultrason en sortie 
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
  digitalWrite(trig, LOW);//émetteur ultrason éteint au début du programme 
  //configuration leds RGB en sortie
  pinMode(rouge1,OUTPUT);
  pinMode(vert1,OUTPUT);
  pinMode(bleu1,OUTPUT);
  pinMode(rouge2,OUTPUT);
  pinMode(vert2,OUTPUT);
  pinMode(bleu2,OUTPUT);
  pinMode(infra,INPUT);
 /*int i ;
    for (i = 0; i < 250; i++)  // cablibration des capteurs IRS 
  {
    qtr.calibrate();
    
  }*/
   qtr.read(IR);
  for(int c=7; c>=0 ;c--){
    Serial.println("capteur (setup) : " );
    Serial.println(7-c );
    sens[7-c] = capt_ir(IR[c]);
    Serial.println(sens[7-c]);
  }

}

void loop() {
lecture(140 , 0);
Panneaux(5,140,0.20);
sens[8] = analyse (sens) ;

}

/*#############################################################
##################CAPTEURs IR ##################################
#############################################################*/
bool capt_ir(int temps){//revoie VRAI si surface NOIRE et FAUX si suface blanche ,en argument le temps de chute de tension
  if (temps < 500){//temps inferieur à 50 microsecondes <==> blanc
    return false;//surface blanche
  }else{
    return true; //surface noire 
  }
}

int compter(){//renvoie le nombre de capteurs qui lit (lisent) une surface noire 
    int c=0;//compteur initialisé à 0 
    qtr.read(IR);//met à jour la liste IR en mettant le temps de chute de tension des broches digitales
    for(int i = 0 ; i<= 7 ; i++ ){
    
      if(IR[i]>500){//temps > 50 microsecondes => surface noire
      c = c+1 ; // incrementation du compteur si surface noire
      }
    }
    return c;//retourne le nombre total de capteur qui lisent une surface noire
  
}

void changer_voie(){
          /*Serial.println("Obstacle");
                //HYPOTHESE : voie "1" =>extremité gauche de la voiture  et voie "2" => =>extremité droite de la voiture
                //si la distance entre la voiture et obstacle est faible ,il faut changer de voie ,en fera en sorte qu'on ait qu'un seul obstacle sur la route 
                if(voie == "1"){
                  direction_droite();
                  avancer(pwm);

                          while (compter() !=0 ){
                                              
                            //on attend que la ligne noire ne soit plus détecter (à cause du if le programme fera voie 1,2,1,2,1 d'où les oscillations des roues avants
                          }
                          while (compter() ==0 ){
                                   direction_tout_droit();           
                            //surface blanche (on attend la prochaine ligne noire ) 
                          }
                         
                              
                            

                  voie = "2" ; //la voiture est sur la voie 2 ,nouvelle hypothese 
              obstacle = 0;
                }else if(voie == "2"){
                  direction_gauche();
                  avancer(pwm);
                          while (compter() !=0 ){
                                    
                            //on attend que la ligne noire ne soit plus détecter (à cause du if le programme fera voie 1,2,1,2,1 d'où les oscillations des roues avants
                          }
                          while (compter() ==0 ){
                                 direction_tout_droit();         
                            //surface blanche (on attend la prochaine ligne noire ) 
                          }

                  voie = "1" ; //la voiture est sur la voie 1 ,nouvelle hypothese
                   obstacle = 0.10; 
                }*/
}
//IR[] contient le temps de chute de tension des 8 capteurs IR ,liste de temps entier
void lecture(int pwm , float distance_virage){//parametres  pwm entre 0 et 255 inclu, la  distance de virage doit être comprise entre 0.30 et 0.50 m 

  
   qtr.read(IR);//met à jour la liste IR en mettant le temps  de chute de tension des broches digitales des capteurs IR
      
       
      filtre(IR);
      //les voies 1 et 2 sont en trait d'épaisseur 1 bande ,au maximum deux capteurs IRS peuvent détecter une surface noire
      if(  (ir[3] || ir[4])&& distance_obstacle_voiture()>= distance_virage  ){
          //il faut  rester sur la ligne noire avec le capteur ir4-5 qui détecte la ligne noire 
          direction_tout_droit();
          avancer(pwm);
          Serial.println("DEVANT");

      }
      else if((    (  ir[0] || ir[1]  || ir[2] )   )&& distance_obstacle_voiture()>= distance_virage 
      ){  //l'obstacle est loin et une ligne noire est détecté (il faut calibrer la ligne noire aux capteurs IR4-5) 
            avancer(pwm);
            direction_droite();  
             Serial.println("DROITE");
      }else if(
         ( ( ir[5] || ir[6] || ir[7])  ) && distance_obstacle_voiture()>= distance_virage){
          //l'obstacle est loin et une ligne noire est détecté (il faut calibrer la ligne noire aux capteurs IR4 ou IR5)
            avancer(pwm);
            direction_gauche(); 
            Serial.println("GAUCHE");
      }else if (distance_obstacle_voiture()<= distance_virage ){
        changer_voie();
      }else{
        Serial.println("STOP voiture");
        stopper_voiture();//si aucun des cas n'est vérifié, la voiture s'arrête
      }
  while(compter() == 0 || compter() == 8){
        Serial.println("STOP voiture");
        stopper_voiture();//si aucun des cas n'est vérifié, la voiture s'arrête
    
  }
}


 
//#################################################################
//########MOTEUR B:vitesse roues arrieres #########################
//#################################################################

void reculer(int pwm){
 digitalWrite(dir_b,LOW);//on suppose HIGH = reculer et LOW = avancer 
 analogWrite(pwm_b,pwm);//alimentation du moteur A par impulsions PWM
 digitalWrite(brake_b,LOW);//frein B non utilisé  
}

void avancer(int pwm){
 digitalWrite(dir_b,HIGH);//on suppose HIGH = reculer et LOW = avancer
 analogWrite(pwm_b,pwm);//alimentation du moteur B par impulsions PWM 
 digitalWrite(brake_b,LOW);//frein B non utilisé  
}

void stopper_voiture(){
 digitalWrite(brake_b,HIGH);//frein B utilisé
}

//#################################################################
//##########MOTEUR A:direction roues avants #######################
//#################################################################

void direction_gauche(){//tourner les roues de devants vers la droite 
 digitalWrite(dir_a,HIGH);//on suppose HIGH = gauche et LOW = droite 
 analogWrite(pwm_a,255);//alimentation du moteur A par impulsions PWM (5V<=>255 en pwm)
 digitalWrite(brake_a,LOW);//frein A non utilisé

}

void direction_droite(){//tourner les roues de devants vers la gauche 
 digitalWrite(dir_a,LOW);//on suppose HIGH = gauche et LOW = droite
 analogWrite(pwm_a,255);////alimentation du moteur B par impulsions PWM (5V<=>255 en pwm)
 digitalWrite(brake_a,LOW);//frein A non utilisé

}

void direction_tout_droit(){//laisser les roues à leur position initiale
 digitalWrite(brake_a,HIGH);//frein utilisé
}


//#################################################################
//#########################CAPTEUR ULTRASON########################
//#################################################################
float distance_obstacle_voiture(){
digitalWrite(trig, HIGH);//envoyer les ondes sonores
delayMicroseconds(10);//pause en microsecondes 
 digitalWrite(trig, LOW);//éteindre l'émetteur 
 unsigned long duree = pulseIn(echo, HIGH);//lire la durée de temps haut du signal reçu (largeur proportionnel à la distance de l'obstacle) 

//receptionner les ondes sonores et en déduire une duree 
  if(duree > 30000)//l'obstacle est hors de portée,on renvoie une distance infinie
   {
    return 1000000.0;//distance "infinie"
   }
   else//les ondes sont revenues 
   {
      //l'onde se reflechit sur l'obstacle et parcourt deux fois la distance voiture - obstacle 
      duree = duree/2;

     //vitesse constante de l'onde 
     
      float T = duree/1000000.0; //on met T en secondes
      float D = T * 340 ; //on multiplie par la vitesse, v=d/t
      Serial.println("distance (en m):");
      Serial.print(D); //affiche la distance mesurée (en mètres)
      Serial.println("");//retour à la ligne 
      return D;//la fonction renvoie la distance en mètres
   }
  
}



/*###############################################################################################
##################################################LEDs RGB#######################################
###############################################################################################"*/

void allumer_leds_rouge(){
  digitalWrite(rouge1,HIGH);
  digitalWrite(rouge2,HIGH);
}
void allumer_leds_verte(){
  digitalWrite(vert1,HIGH);
  digitalWrite(vert2,HIGH);

}
void allumer_leds_bleue(){
  digitalWrite(bleu1,HIGH);
  digitalWrite(bleu2,HIGH);
}

void allumer_leds_blanche(){
allumer_leds_rouge();
allumer_leds_verte();
allumer_leds_bleue();
}


void eteindre_leds_rouge(){
  digitalWrite(rouge1,LOW);
 digitalWrite(rouge2,LOW);

}


void eteindre_leds_verte(){
  digitalWrite(vert1,LOW);
  digitalWrite(vert2,LOW);

}
void eteindre_leds_bleue(){

  digitalWrite(bleu1,LOW);
  digitalWrite(bleu2,LOW);
}

void eteindre_leds_blanche(){
  eteindre_leds_rouge();
  eteindre_leds_verte();
  eteindre_leds_bleue();
}
/*###########################################################################################################
#########################################################PANNEAUX#########################################
###########################################################################################################*/
int  impulsion(){//retourne le nombre d'impulsions ,chaque dent blanc du créneau donne un impulsion
  String dent = "NSP"; //initialisation (dent peut prendre trois valeurs "N" si dent noire ,"B" si absence de dent et "NSP" si on ne sait pas (initialisation)
  int n = 0;//nbre d'impuslsion 
  while (true){
        if(compter()>0 && (dent=="B" ||dent=="NSP")   ){//il ya une dent noire et on a recensé un dent blanche ,incrementation compteur n + mise à jour des dents
          dent ="N";//la dernière information des capteurs IR est une dent noire
        }
        if(compter() == 0 && (dent=="N" || dent =="NSP") ){
          dent="B";//la dernière information des capteurs IR est l'absence de dent noire
          n=n+1;//incrementation du compteur
        }
        Serial.println(n);//affiche à l'écran le nombre d'impulsion 
        if(compter() == 8){//ligne de "FIN" qui indique la fin du panneau 
          return n;//retourne le nombre d'impuslion + seul moyen de sortir du while infini
        }
  }
}
void Panneaux(int duree,int pwm,float distanceM){
  BICOLORE(150);//le panneau tricolore ne requiert pas de code barre 
if(compter() == 8 ){//le système lit la ligne de "DEBUT" qui indique présence d'un panneau de signalisation
  while(compter() == 8){//on attend de ne plus avoir de ligne de "DEBUT" pour compter les impulsions 
    
  }
  int i = impulsion(); //nbre d'impulsion entre les 2 lignes noires

  if (i == 1 ){//1 impulsion
    STOP(duree,pwm);
    priorite  = "X";
  }else if(i ==2){//2 impulsions 
    PIETONS(distanceM , pwm); 
    priorite  = "X";     
  }else if(i == 3 ){//3 impulsions 
    TUNNEL();      
    priorite  = "X";
  }else if( i == 4){//4 impulsions
      while(compter() == 8){//on attend de ne plus avoir de ligne 
    
       }
       priorite = "droite";//Quand il ya  2 chemins possibles , la priorité va à celle qui va à droite
  }else if(i == 5){
       while(compter() == 8){//on attend de ne plus avoir de ligne 
    
       }
       priorite  = "gauche";//Quand il ya  2 chemins possibles , la priorité va à celle qui va à gauche
  }

    
}
  
}

void STOP(int duree,int pwm){//durée : en secondes , pwm : liée à l'alimentation du moteur
  Serial.println("STOP");
  allumer_leds_rouge();//information visuelle 
  stopper_voiture();
  delay(duree * 1000 );//temporation en seconde
  avancer(pwm);
  while(compter() == 8){//on ne veut  plus être sur la ligne noire (8 capteurs détectent la ligne => le système croira qu'il a encore affaire à une trame codée )
  //patience  
  }
  eteindre_leds_rouge();//information visuelle
  
}

void PIETONS(float distanceM , int pwm){// la voiture n'avance pas tant qu'un obstacle humain est devant lui (piétons)
  allumer_leds_bleue();//information visuelle 
  while(distance_obstacle_voiture() <= distanceM ){
  stopper_voiture();  
  }
  avancer(pwm);
  eteindre_leds_bleue();//information visuelle
  while(compter() == 8){//on ne veut  plus être sur la ligne noire (8 capteurs détectent la ligne)
  //patience  
  }
}

void TUNNEL(){
  if (digitalRead(rouge1) && digitalRead(bleu1) && digitalRead(vert1)){//led RGB1 allumée <=>led RGB2 allumée
    eteindre_leds_blanche();
  }else{
      allumer_leds_blanche();
  }
    while(compter() == 8){//on ne veut  plus être sur la ligne noire (8 capteurs détectent la ligne)
  //patience  
  }

}

void BICOLORE(int pwm){
        if (phototransistor()){//feu vert
          avancer(pwm);
          allumer_leds_verte();

        }else{//feu rouge
           stopper_voiture();
           while ( !phototransistor() ){
              allumer_leds_rouge();
            }
    
        }
    
    eteindre_leds_blanche();  
}

/*#######################################################################################################
###################################PHTOTOTRANSISTOR####################################################
######################################################################################################*/
bool phototransistor(){

        if(digitalRead(infra) == HIGH ){//feu vert
                Serial.println("detecte");
               return true; //réception de lumière
        }else{//feu rouge
          Serial.println("pas detecte");
               return false; //absense de lumière
         }
}
/*#########################################################################################################
#############################################SENS UNIQUE##################################################
########################################################################################################"*/


bool analyse(bool trame[8]){
  qtr.read(IR);
  bool a[8];
  for(int c=7; c>=0 ;c--){
    Serial.println("capteur : " );
    Serial.println(7-c+1 );
    a[7-c] = capt_ir(IR[c]);
    Serial.println(a[7-c]);
  }
 
  for(int i = 0; i<=7 ; i++){//parcourt le capteurs IR de la 8 à 1
      if(a[i]){
                if(trame[i] || trame[i+1] || (trame[i-1] && i>0 ) ){
                  return a;//décalage du capteur IR par rapport à la ligne noire vers la droite ou gauche 
                }else if( compter() != 0 ){// contradiction a passe de 0 à 1 
                    allumer_leds_rouge();//information visuelle
                    stopper_voiture();
                      while(true){
                        //la voiture s'arrête définitivement 
                        //Serial.println(compter() );
                      }
                }
                
      }
  }
  return a;
}

void structure(){

sens[8] = analyse (sens) ;

}

void filtre(unsigned int temps[8]){//filrer les motifs indésirables (sens unique) et réaliser la fonction lecture()
bool detecte = false;
if(priorite == "X" || priorite == "droite"){
        for(int i =0 ; i<=7 ;i++){//on parcourt les capteurs de la droite vers la gauche 
        if (!detecte && capt_ir(temps[i]) ){//temps de chute de tension > 50 microsecondes => surface noire
          detecte = true; 
          ir[i] = true;
        }else{
         ir[i] = false;//surface blanche  
        }
      }
}else{
        for(int i =7 ; i>=0 ;i--){//on parcourt les capteurs de la gauche vers la droite
        if (!detecte && capt_ir(temps[i]) ){//temps de chute de tension > 50 microsecondes => surface noire
          detecte = true; 
          ir[i] = true;
        }else{
         ir[i] = false;//surface blanche  
        }
      }
  
}


}



/*#######################################################################################################################
###############################################ASSERVISSEMENT######################################################
#########################################################################################################################*/





