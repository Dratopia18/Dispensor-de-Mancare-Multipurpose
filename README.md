# Dispensor-de-Mancare-Multipurpose

Pentru a face proiectul, aveti nevoie de:
- 1x Placa compatibila cu Arduino Uno R3
- 1x Senzor infra-rosu de detectare a obstacolelor (Distanta: 2-180 cm)
- 1x Servo-Motor SG90 pentru miscarea trapei
- 1x Display LCD I2C pentru a prezenta situatia automatului
- 3x Butoane pentru a seta automatul
- 3x Breadboarduri
- Cutie de prim ajutor (ce contine toate componentele de mai sus, cu exceptia servo-motorului)
- Recipient de plastic cu gaura (ce contine servo-motorul)
- Trapa de plastic (de tip solnita)
- Raft de plastic (de care sunt prinse cutia si recipientul)
- Un suport de plastic pentru a prinde servo-motorul de recipient
- Suruburi si piulite pentru prinderea componentelor (recipientul, cutia, componentele electronice, servo motorul)

## Pasul 1 - Facem rost de componente
- Pentru componentele electronice, recomand sa folositi Optimus Digital pentru cumpararea componentelor 
- Cutia de prim ajutor se poate gasi la orice hypermarket din tara
- Recipientul din plastic poate fi orice (de la un bidon, pana la o cutie)
- Raftul de plastic l-am luat de la Dedeman

## Pasul 2 - Plasam componentele in cutie si recipient
- Pentru placa, puneti-l lipit de un perete, ca sa avem access cat mai usor la portul USB al placii, ce alimenteaza dispozitivul
- Pentru breadboarduri, lipiti-le oriunde vi se pare util (recomandat ar fi sa atasati conectiunile prin fire tata-tata sau mama-tata mai intai)
- Pentru senzor si butoane, recomand sa faceti marcaje cu o carioca/creion pe cutie, unde vi se pare ca ar trebui puse componentele (puneti componentele fata in fata cu pozitia unde le veti pune mai tarziu pentru un parcurs mai usor)
- Pentru display, desenati un dreptunghi de 16 cm lungime si 2 cm latime in locul unde va fi pozitionat displayul (recomandat ar fi sa fie fix langa butoane si senzor) si doua marcaje pe unde veti prinde displayul de cutie cu suruburi (recomandarea ar fi ca aceste marcaje sa fie diagonal opuse una de cealalta)
- Pentru servo-motor, trebuie facute:
  - Un marcaj in cutie pentru firele servo-motorului
  - Un marcaj in recipient pentru firele servo-motorului
  - Doua marcaje in recipient pentru a prinde mai tarziu suportul servo-motorului de recipient
  - Un marcaj in suport pentru a prinde servo-motorul de suport
  - De asemenea, trebuie sa lipiti trapa de plastic de servo-hornul ce foloseste 2 elici si sa fie prins cu doua suruburi

## Pasul 3 - Facem gauri (cu o bormasina)
- O gaura in cutie pentru portul USB al placii Arduino
- O gaura in cutie pentru surubul de care vom atasa placa de cutie
- O gaura in cutie pentru firele servo-motorului
- O gaura mai mare in cutie pentru displayul LCD
- Doua gauri in cutie, apropiate de gaura mare, pentru a prinde displayul de cutie
- Trei gauri in cutie pentru a prinde butoanele
- Doua gauri in cutie pentru senzor (unul pentru detectorul IR si unul pentru receptorul IR)
- O gaura in cutie pentru a prinde senzorul de cutie
- Doua gauri in cutie pentru prinderea cutiei de raft
- O gaura in recipient pentru firele servo-motorului
- Doua gauri in recipient pentru a prinde suportul servo-motorului de recipient
- O gaura in suport pentru a prinde servo-motorul cu suportul

Pentru gaura portului USB si a displayului, este recomandat sa folositi o pila patratica pentru a slefui gaurile, astfel incat sa fim siguri ca intra componentele in gaurile respective

## Pasul 4 - Atasam componentele in cutie (in afara de display)
- Prindem placa de cutie cu un surub si o piulita
- Prindem senzorul de cutie cu un surub si o piulita
- Prindem butoanele (in cazul in care NU sunt lipite de breadboard) cu piulite (in cazul in care ai butoanele lipite de breadboard, trebuie sa prinzi 2 suruburi de breadboard si sa strecori firele prin inchizatura. Daca nu vrei asta, uita-te la pasul 5)
- Prindem cutia de raft cu suruburi si piulite
- Prindem servo-motorul de suport cu un surub si o piulita
- Prindem suportul de recipient cu un surub si o piulita

## Pasul 5 -  Conectam componentele la placa
- Pentru display:
  - Vom folosi pinurile de GND1, VCC1(5V), SCL si SDA si le vom conecta cu fire tata-tata la primul breadboard
  - Folosim fire mama-tata pentru a conecta porturile firelor la pinii necesari
  - Abia apoi atasezi displayul in gaura mare creata si il prinzi cu suruburi si piulite
- Pentru senzor:
  - Vom folosi pinul digital 4 (PD4) pentru output si il vom conecta cu firul tata-tata la al doilea breadboard
  - Folosim fire tata-tata pentru a conecta GND1 si VCC1(5V) la al doilea breadboard
  - Folosim fire mama-tata pentru a conecta porturile firelor la pinii necesari (pentru PD4, il conectam la OUT, in cazul in care senzorul are asta)
- Pentru butoane:
  - Vom folosi pinurile digitale 5, 6 si 7 (PD5, PD6 si PD7) si le vom conecta cu fire tata-tata la al treilea breadboard
  - Folosim firul tata-tata pentru a conecta GND1 la al treilea breadboard (recomandarea ar fi ca pinii digitali si GND1 sa fie pe circuite DIFERITE)
  - Pentru conectivitate ai 3 optiuni:
    - Atasezi butoanele de breadboard, folosesti inca doua fire tata-tata pentru a transfera GND1 pe celelalte 2 butoane si lipesti breadboardul fix unde ai gaurile pentru butoane (trebuie 2 gauri pentru a prinde breadboardul de cutie)
    - Folosesti fire mama-tata sa conectezi porturile firelor cu pinii necesari
    - Daca ai rude ce au butoane fancy de acum 40 de ani, folosesti fire tata-tata, un pistol de lipit si un voltmetru (pentru a va da seama care e GND si care e pentru pinul digital), si lipiti firele de pinii necesari (DISCLAIMER: Faceti acest lucru cu un adult sau cu o persoana ce are experienta in asta. Daca va raniti sau faceti conexiunile prost, va trebuie butoane noi)
- Pentru servo-motor
  - Strecuram firele necesare pentru conectarea servomotorului prin gaurile facute  
  - Vom folosi pinii de GND2, VCC2(5V) si pinul digital 3 si ii vom conecta cu firele tata-tata la porturile necesare pentru servo-motor
## Pasul 6 - Alimentam dispozitivul
Il conectam la laptop pentru a incarca codul scris de mine in dispozitiv

## Pasul 7 - Incarcam codul in dispozitiv
Prin VSCode si PlatformIO (ce poate fi descarcat prin VSCode Extensions), incarcam codul scris cu Ctrl + Alt + U

Si gata! Ai un dispensor de mancare multipurpose la tine acasa. Costurile in a produce acest dispozitiv se invarte pe la 200 de lei, ce e cam de doua ori mai ieftin decat alte dispensoare ce fac acelasi lucru. 
Pentru a intelege cum functioneaza codul, citeste README-ul din src
