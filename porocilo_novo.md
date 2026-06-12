# Končno poročilo: RInS

**Ekipa:** Zeta
**Člani:** Lara Mehle,

------------------------------------------------------------------------

## 1. Uvod

V sklopu predmeta Razvoj inteligentnih sistemov smo razvili avtonomni
robotski sistem na platformi TurtleBot4 , ki se sooča z izzivi sodobne
industrije. Naš cilj je bil razviti sistem, ki se zna popolnoma
samostojno premikati po prostoru, prepoznavati prisotne delavce in
izvajati napredne preglede znotraj industrijskega okolja.

Glavni cilj prvega dela naloge (Task 1) je bil zgraditi mapo ograjenega
prostora ter znotraj tega območja locirati tri naključno postavljene
natisnjene obraze in dva barvna obroča. Robot je moral prostor
sistematično preiskati. Ob vsaki zaznavi obrazov se je moral osebi
približati in jo pozdraviti. Ob zaznavi obročev pa je moral prepoznati
njihovo barvo in jo naglas sporočiti.

Drugi del projekta predstavlja neposredno nadgradnjo in celovito
integracijo vseh predhodno razvitih komponent v scenarij industrijskega
okolja, ki simulira koncepte Industrije 5.0. To okolje sestavljajo jasno
označene transportne poti, omejene delovne celice, tekoči trakovi ter
različne ovire.

Celotno okolje je razdeljeno na dve ločeni sobi z različnimi pravili
gibanja in specifičnimi nalogami:

Misija se začne v prvi sobi, kjer mora robot najprej izvesti
sistematično raziskovanje celotnega prostora. Med navigacijo mora
zaznati, lokalizirati in si v svoj globalni semantični zemljevid
zapomniti položaje vseh prisotnih objektov in oseb. Poleg detekcije
obrazov in obročev iz prve naloge je nabor objektov tukaj razširjen še
na industrijske cilindre (sode). Pri cilindrih mora robot zanesljivo
določiti njihovo barvo ter prostorsko orientacijo. Če je cilinder
postavljen vertikalno, si robot samo zapomni njegov položaj. V primeru,
da sistem zazna ležeč cilinder, pa se sproži namenski algoritem za
analizo teksture oziroma segmentacijo podlage, saj mora robot preveriti,
ali gre za razlitje tekočine, si to anomalijo zapomniti ter jo
zabeležiti za končno poročilo.

Pomemben del prve sobe je interakcija z delavci na njihovih delovnih
mestih. Robot mora pristopiti do vsake zaznane osebe, jo identificirati
ter pridobiti navodila za specifično nalogo. Čeprav uradna navodila
predvidevajo možnost govornega dialoga s prepoznavanjem govora (ASR),
kjer moramo pri ženskih delavkah zaradi spreminjanja mnenja ukaze
večkrat potrditi, smo v naši implementaciji izbrali tehnično robustnejši
(in lažji) pristop. Robot navodila za nalogo pridobi z optičnim branjem
in dekodiranjem namenskih QR kod, ki se nahajajo ob delavcih. Prejeta
navodila lahko obsegajo različne tipe nalog, kot so natančno štetje in
določene barve obročev v prostoru, preverjanje stanja sodov ali
detekcija površinskih anomalij na ploščicah, ki so postavljene na
določenem tekočem traku. Ko robot dobi navodilo za preverbo anomalij na
določenem tekočem traku, mora navigirati do pravega in opraviti napredno
detekcijo drobnih razpok in strukturnih poškodb na dveh težavnostnih
nivojih blagih in hudih anomalij.

Po uspešno opravljeni poizvedbi in analizi prvega prostora se robot
usmeri proti drugi sobi, kjer se pravila navigacije spremenijo. Tukaj
mora robot z vizualnim sistemom zaznati in natančno slediti označeni
modri črti na tleh, ki ga varno vodi skozi celoten prostor. Celotna
misija se zaključi, ko robot uspešno navigira do lokacije tehničnega
direktorja, na podlagi vseh zbranih podatkov avtonomno generira uradno
inšpekcijsko poročilo v obliki PDF datoteke ter ga predloži vodstvu
obrata.

### 1.1 Zahtevane zmožnosti in funkcionalnosti robota

Da bi sistem uspešno izpolnil vse pogoje, smo morali razviti in
integrirati širok spekter robotskih funkcionalnosti.

Med **nujne funkcionalnosti** sodijo:

\- **Avtonomna navigacija na podlagi ciljev** --- krmiljenje mobilne
platforme do vnaprej določenih lokacij,

**- Detekcija obrazov**

**- Detekcija obročev**

**- Detekcija cilindrov**

\- **Zanesljivo prepoznavanje barv** --- določanje barv obročev in
cilindrov (rdeča, zelena, modra, rumena, vijolična, oranžna, rjava),

\- **Interakcija z osebami** --- približanje zaznanim obrazom in osnovna
komunikacija,

Med **napredne zmožnosti**, ki prinašajo dodatno vrednost, spadajo:

\- **Napredna prepoznava oseb** --- prepoznavanje identitete vnaprej
znanih obrazov

\- Prepoznava spola oseb,

\- **Inteligentno avtonomno raziskovanje** --- namesto fiksnih točk
dinamično pokrivanje prostora,

\- **Varno in usmerjeno gibanje** --- v prvem prostoru robot ne sme
prečkati rumenih linij, v drugem pa z vizualnim vodenjem sledi modri
črti do lokacije tehničnega direktorja,

\- **Natančna inšpekcija delovnih celic** --- detekcija delovne postaje,
zaznavanje ploščic ter segmentacija in prepoznavanje površinskih poškodb
na dveh težavnostnih nivojih (hude in blage degradacije),

\- **Napreden govorni dialog** --- avtomatsko prepoznavanje govora; pri
moških delavcih robot navodila sprejme neposredno, pri ženskih delavkah
mora izbiro potrditi vsaj dvakrat,

\- **Generiranje inšpekcijskega poročila** --- robot ob koncu misije
avtonomno ustvari uradno PDF poročilo.

Poleg algoritmov končna ocena upošteva tudi robustnost sistema, hitrost
izvedbe naloge, vizualizacijo v RViz ter splošni vtis delovanja.

------------------------------------------------------------------------

## 2. Metode

Za doseganje optimalnega delovanja in izpolnjevanje vseh zahtev smo
razvili celovit nabor algoritmov, ki vključuje navigacijo, zaznavanje
objektov, prepoznavanje, inšpekcijo in govorni dialog. V nadaljevanju so
podrobno predstavljene posamezne tehnične rešitve.

### 2.1 Navigacija

Za avtonomno navigacijo smo uporabili navigacijski sklad **Nav2**, ki je
standardni okvir za ciljno navigacijo v ROS 2. Robot se lokalizira na
vnaprej zgrajeni karti v koordinatnem sistemu `map` z uporabo AMCL,
odometrije in laserskega skenerja. V konfiguraciji Nav2 globalno pot
načrtuje `NavfnPlanner`, lokalno sledenje poti pa izvaja
`RegulatedPurePursuitController`. Navigacijski cilji se pošiljajo prek
akcije `NavigateToPose`, kjer je cilj podan kot poza v globalnem
koordinatnem sistemu karte.

Potek navigacije nadzira `behavior_manager`, samo zaporedje patrolnih
točk pa izvaja `waypoint_navigator`. Navigacijo smo razdelili na
naslednje faze.

1. **Inicializacija in lokalizacija.** Pred začetkom avtonomnega gibanja
   se zaženejo Nav2, AMCL in vozlišča paketa `task1`. Patruljni cilji se
   pošiljajo šele, ko je pripravljen akcijski strežnik `navigate_to_pose`.
   Za začasne cilje do zaznanih objektov `behavior_manager` dodatno čaka
   na veljavno pozo iz `/amcl_pose`, saj so cilji in zaznani objekti
   obravnavani v koordinatnem sistemu `map`. Patrulja se sproži prek ukaza
   `/patrol_command`, `behavior_manager` pa nato omogoči dejansko izvajanje
   prek teme `/patrol_enabled`.

2. **Patrulja po prvi sobi.** Osnovno raziskovanje prve sobe poteka po
   vnaprej določenih točkah iz `config/waypoints.yaml`. Vsaka točka
   vsebuje položaj `x`, `y`, orientacijo `yaw` in po potrebi premor
   `pause`. Nekatere točke imajo enak položaj in različne orientacije, zato
   robot na pomembnih mestih izvede pregled okolice v več smereh. Tako se
   prostor pokrije ponovljivo, zaznavni algoritmi pa imajo dovolj časa za
   zaznavanje oseb, obročev in cilindrov. Ko robot konča vse orientacije
   na eni fizični lokaciji, `waypoint_navigator` objavi `/patrol_group_end`.
   Ko so opravljene vse patrolne točke, objavi še `/patrol_finished`, kar
   v `behavior_manager` sproži zaporedje po končani patrulji.

3. **Navigacija do obrazov.** Ko je potrjena lokacija obraza, se patrulja
   ne prekine nujno takoj, temveč se cilj najprej doda v vrsto čakajočih
   ciljev. Pristop se praviloma začne po zaključku trenutne skupine
   orientacij, da robot ne prekine pregleda iste waypoint lokacije.
   `behavior_manager` nato iz orientacije obraza izračuna pristopno točko
   pred osebo in pošlje začasni cilj prek `NavigateToPose`. Po prihodu
   robot izvede interakcijo, cilj označi kot obdelan in nato nadaljuje
   patruljo z naslednjo točko. Če je obraz zaznan med sledenjem modri
   črti, se sledenje začasno izključi, robot opravi pristop k osebi, nato
   pa se modra črta ponovno omogoči.

4. **Obroči med patruljo.** Obroči se zaznavajo in lokalizirajo med
   patruljiranjem. Ker za obroče ni potrebna posebna navigacijska poza,
   robot praviloma ne prekine poti, temveč lokacijo in barvo obroča
   zabeleži med nadaljevanjem patrulje.

5. **Cilindri med patruljo.** Pokončni cilindri se ob zaznavi samo
   zabeležijo v globalni karti. Če je zaznan ležeči cilinder, lahko
   `behavior_manager` po zaključku trenutne waypoint skupine začasno
   ustavi patruljo in pošlje robota na varno pristopno točko ob cilindru.
   Pristopna točka ima odmik in stranski zamik, da robot ne zapelje
   neposredno v objekt. Po obdelavi cilindra se navigacija vrne v običajno
   patrolno zaporedje.

6. **Varovanje pred rumeno črto v prvi sobi.** Med navigacijo po prvi sobi
   se uporablja ločeno varovalo za rumeno črto. To ne zamenja Nav2
   planiranja poti, ampak deluje kot varnostni sloj: kamera spremlja
   rumeno oznako v bližini robota, ob nevarnem približanju pa robot ustavi
   oziroma se umakne nazaj. Na ta način se ohrani osnovna waypoint
   navigacija, hkrati pa robot ne nadaljuje čez prepovedano območje.

7. **Prehod do delovne postaje.** Ko robot iz QR navodila dobi nalogo za
   pregled določene delovne postaje, si `behavior_manager` zapomni barvo
   postaje. Prehod se izvede šele po končani patrulji in po obdelavi vseh
   čakajočih ciljev. Če je pristopna poza rdeče ali zelene postaje znana
   iz `/workstation_markers`, se pošlje Nav2 cilj do te poze. Če položaj
   ni znan, sistem ne izvede tega obiska in nadaljuje na zaključni del
   navigacije. Ta faza opisuje samo prihod do delovne postaje; natančna
   poravnava in pregled delovne celice sta obravnavana v ločenem poglavju.

8. **Zaključna točka in prehod v drugo sobo.** Po koncu patrulje oziroma
   po zaključku obiska delovne postaje robot navigira do zaključne točke
   pred drugo sobo. Ta cilj je v `behavior_manager` nastavljen kot
   `final_wp_x`, `final_wp_y` in `final_wp_yaw`. Med tem se rumeno
   varovalo izključi, stanje robota pa preide v `FINISHING_ROUNDS`.

9. **Sledenje modri črti.** Ko je zaključna točka dosežena,
   `behavior_manager` omogoči `blue_line_explorer` prek
   `/blue_line_enabled` in stanje robota spremeni v `FOLLOW_BLUE_LINE`.
   Takrat se ciljno premikanje po karti zamenja z vizualnim vodenjem po
   modri črti. Nepotrebna vozlišča za prvo sobo se ustavijo, robot pa v
   drugi sobi išče modro črto, ji sledi in pri razcepih izbira ustrezno
   smer. Če črto izgubi ali naleti na oviro, preide v obnovitveno fazo
   iskanja oziroma obračanja, dokler ponovno ne najde poti.

Trenutna faza robota se objavlja tudi kot stanje na temi `/robot_state`.
To stanje uporabljajo posamezna vozlišča, da vedo, ali smejo poseči v
gibanje robota. V stanju `IDLE` robot miruje in čaka na začetek naloge.
V stanju `PATROL` je aktivna navigacija po waypointih. Ob pristopu k
zaznanemu obrazu preide v `APPROACH_FACE`, med interakcijo pa v
`INTERACT_FACE`. Pri cilindrih se za ležeče cilindre uporablja analogni
prehod `APPROACH_BARREL` oziroma `INTERACT_BARREL`, medtem ko se pokončni
cilindri samo zabeležijo. Pri prehodu proti delovni postaji se uporabi
stanje `APPROACH_WORKSTATION`, po uspešnem prihodu pa `WORKSTATION`, kjer
robot čaka na signal, da se lahko vrne v navigacijski tok. Podrobnosti
delovne celice so opisane v ločenem poglavju. Po koncu prvega dela se
robot premakne v zaključno navigacijsko fazo `FINISHING_ROUNDS`, nato pa
v `FOLLOW_BLUE_LINE`, kjer nadzor prevzame sledenje modri črti. Varovalo
rumene črte ima pri tem ločeno interno stanje `CLEAR` oziroma `BACKING`,
saj deluje kot varnostni sloj nad običajnimi hitrostnimi ukazi.

S tem je glavna navigacija razdeljena na globalno premikanje po karti v
prvi sobi, lokalne prekinitve zaradi pomembnih objektov in vizualno
vodenje po modri črti v drugi sobi. Podrobnosti posameznih zaznavnih
algoritmov in delovne postaje so opisane v ločenih razdelkih poročila.

### 2.2 Zaznavanje in prepoznavanje obrazov

Za zaznavanje obrazov smo uporabili knjižnico **face_recognition**, ki
temelji na dlib implementaciji detektorja z metodo HOG (Histogram of
Oriented Gradients) in 128-dimenzionalnih obraznih enkoderjev, naučenih
z metričnim učenjem (arhitektura ResNet-34, naučena na 3 milijonih
slikah). HOG detektor v sliki poišče regije, ki ustrezajo vzorcem
obraza, nato pa za vsak zaznani obraz izračuna kompakten
128-dimenzionalni opisnik (encoding), ki enolično opiše posameznika.

Prepoznavanje poteka z izračunom evklidske razdalje med encodingom
zaznanega obraza in referenčnimi enkoderi znanih oseb, ki so bili
predhodno izračunani iz referenčnih fotografij. Oseba je prepoznana, če
je razdalja manjša od praga zaupanja 0,6.

Za zaznavanje spola smo uporabili prednastavljeni **Caffe model**
(gender_net), ki iz izrezane slike obraza vrne binarno klasifikacijo
(moški/ženski). Spol se zazna pred primerjavo z bazo in služi kot
dodatna validacija: najprej se poišče najboljši kandidat po evklidski
razdalji enkoderjev, nato pa se zaznani spol primerja s spolom tega
kandidata v referenčni bazi. Če se spola ne ujemata, se prag zaupanja
zaostrí z 0,6 na 0,45, kar prepreči napačne prepoznave med osebami
različnih spolov s sicer podobnimi obraznimi enkoderi.

Zaznave posameznih okvirjev smo clustrali v globalnem koordinatnem
sistemu karte (cluster radius 0,6 m) z glasovanjem po identiteti. Obraz
je bil potrjen po vsaj 5 zaznavah, kar zmanjša število lažnih pozitivnih
rezultatov. Potrjena lokacija je bila posredovana modulu
behavior_manager, ki je sprožil navigacijo k osebi in glasovnim
pozdravom s sintezo govora.

Po potrditvi identitete se v RViz2 na lokaciji osebe prikaže marker z
njenim imenom. Vzporedno se objavi tudi debug slika (/face_debug_image),
na kateri so obrazi obkroženi z okvirji, zraven pa so izpisani zaznano
ime, spol in vloga --- kar olajša razhroščevanje in vizualno preverjanje
delovanja sistema med izvajanjem.

### 2.3 Zaznavanje obročev

Za zaznavanje obročev sta bili narejeni dve implementaciji:

**Prva implementacija** je temeljila na pristopu *color-first*: najprej
se je v barvni sliki izvedla HSV segmentacija za vsak barvni razred
posebej (rdeča, zelena, modra, rumena, oranžna, črna), nato pa so se v
vsaki barvni maski iskale konture. Za vsako konturo se je preverilo, ali
jo je mogoče aproksimirati z elipso (ker obroč pri pogledu pod kotom
tvori elipsasto obliko), ter ali ustrezata razmerje stranic (min. 0,28)
in cirkularnost. Globinska verifikacija je preverila, ali je center
obroča prazen (neskončna globina), rob pa poln. Potrditev zaznave je
zahtevala prisotnost v vsaj 8 zaporednih kadrih.

Ker je bila ta rešitev občutljiva na svetlobne pogoje in ozadje v
simulatorju --- HSV pragovi so se pogosto sprožili na objektih podobne
barve ---, smo jo nadomestili z robustnejšo drugo implementacijo.

V drugi in končno uporabljeni implementaciji smo uporabili Houghevo
transformacijo krogov (Hough Circle Transform) na sliki disparitete.
Pristop temelji na naslednjih korakih:

1.  Globinska slika se pretvori v disparitetno, kar ojača bližnje
    strukture,
2.  `cv2.HoughCircles` zazna krožne robove,
3.  **Verifikacija z globino**: center obroča mora biti prazen
    (neskončna globina oz. ozadje), rob mora biti poln (≥35 % vzorcev v
    radiju mora imeti veljavno globino),
4.  **Verifikacija z več kadri** (cross-frame confirmation): obroč mora
    biti zaznan v vsaj 8 zaporednih kadrih brez prekinitve daljše od 4
    kadrov,
5.  Barva obroča se določi z **HSV segmentacijo** na barvni sliki v
    definiranih barvnih razponih.

### 2.4 Zaznavanje sodov

Sodi (cilindri) so bili zaznavani s kombinacijo **HSV barvne
segmentacije** in analize oblaka točk iz OAK-D kamere.

Oblak točk se segmentira po HSV barvni maski --- vsaka točka se
ovrednoti glede na njen odtenek, nasičenost in svetlost. Za vsak zaznani
barvni segment se izračuna centroid v koordinatnem sistemu kamere in s
pomočjo TF2 transformacij pretvori v globalni koordinatni sistem karte
(map frame).

Ker robot isto mesto večkrat opazi iz različnih zornih kotov, se zaznave
**clustrajo** z inkrementalnim algoritmom: nova zaznava se doda
obstoječemu grozdu, če je znotraj 0,30 m (pokončni sodi) oziroma 0,65 m
(ležeči sodi, ki imajo večjo variabilnost centroida). Centroid grozda se
sproti posodablja z inkrementalnim povprečjem. **Barva** soda se določi
z glasovanjem večine prek vseh zaznav v grozdu po osmih barvnih
razredih: rdeča, zelena, modra, rumena, oranžna, vijolična, rjava, črna.
**Orientacija** (pokončen/ležeč) se prav tako določi z glasovanjem na
podlagi oblike in porazdelitve točk oblaka.

Za zaznavanje razlitja smo implementirali ločen servis
(/spill_check), ki ob klicu analizira zadnji zajet oblak točk. Točke se
najprej filtrirajo na razdaljo do 1 m od kamere in transformirajo v map
frame. Nato se izreže horizontalna **Z-rezina med 0,5 cm in 15 cm nad
tlemi** --- to je višinski pas, v katerem bi bila vidna razlita
tekočina. Razlitje je zaznano, če ta rezina vsebuje vsaj 4000 točk.
Izsek rezine se za vizualno preverjanje objavi tudi v RViz2.

### 2.5 Sledenje modri črti

Sledenje modri črti je namenjeno navigaciji robota v drugi sobi, kjer se
robot ne premika več po vnaprej določenih Nav2 waypointih, temveč sledi
talni oznaki. Funkcionalnost izvaja vozlišče `blue_line_explorer`, ki se
vključi po zaključku prvega dela naloge oziroma po prejemu sporočila na
temi `/blue_line_enabled`. Ko je omogočeno, vozlišče prevzame izdajanje
hitrostnih ukazov na `/cmd_vel_unstamped` in objavlja stanje robota
`FOLLOW_BLUE_LINE`.

Glavni deli algoritma so:

- **Zajem slike:** sistem uporablja zgornjo kamero
  (`/top_camera/rgb/preview/image_raw`), ki gleda proti tlom in zato
  omogoča zaznavanje modre črte neposredno pred robotom.

- **Barvna segmentacija:** slika se pretvori v barvni prostor HSV, nato
  se izdela maska za modro oziroma cian barvo. Poleg HSV praga se preverja
  tudi dominantnost modrega in zelenega kanala nad rdečim, kar izboljša
  zaznavo pri različnih svetlobnih pogojih.

- **Čiščenje maske:** nad masko se izvedeta morfološko odpiranje in
  zapiranje, s čimer se odstranijo manjši šumi. Če je v maski premalo
  modrih pikslov, sistem črte ne obravnava kot zanesljivo zaznane.

- **Krmiljenje po centroidu:** iz momentov maske se izračuna centroid
  črte. Horizontalni odmik centroida od sredine slike predstavlja napako,
  iz katere se izračuna kotna hitrost robota. Večji kot je odmik črte od
  sredine, močneje robot popravi smer.

- **Glajenje gibanja:** kotni ukaz je omejen in dodatno zglajen z
  eksponentnim filtrom, da robot ne reagira sunkovito na kratke motnje v
  sliki.

Za obravnavo razcepov je slika razdeljena na tri regije zanimanja:

- **leva regija** zazna možnost zavoja v levo,
- **sredinska regija** predstavlja nadaljevanje naravnost,
- **desna regija** zazna možnost zavoja v desno.

Če je aktivna samo sredinska regija, robot vozi naravnost z višjo
hitrostjo. Ko je zaznana leva ali desna veja, vozlišče preide v način
`split_active`, zmanjša hitrost in začne previdneje obravnavati razcep.
Pri razcepih robot preferira levo smer, kar je izvedeno z dodatnim kotnim
odmikom, vendar samo takrat, ko je leva veja dejansko vidna. Ko se robot
po zavoju ponovno poravna na sredinsko črto, se po kratkem časovnem
zadržanju vrne v običajen način sledenja.

Delovanje vozlišča je organizirano kot stroj stanj:

- `IDLE` - vozlišče je neaktivno in ne posega v gibanje robota.
- `SEARCH` - robot počasi obrača in išče modro črto.
- `FOLLOW` - robot sledi zaznani črti na podlagi centroida maske.
- `UTURN` - obnovitveno stanje, ki se sproži ob izgubi črte, trku ali
  preblizu zaznani oviri.

Če robot izgubi črto za več kot približno 2 sekundi, se sproži
obnovitveni obrat. Po končanem obratu se robot vrne v stanje `SEARCH` in
ponovno poišče modro črto. Podobno se obnovitev sproži tudi ob zaznanem
trku ali kadar LiDAR zazna oviro neposredno pred robotom. Pri manjši
razdalji do ovire robot najprej zmanjša linearno hitrost, pri kritični
razdalji pa izvede obrat.

Za razhroščevanje in spremljanje delovanja vozlišče objavlja:

- `/blue_line/debug_image` - označena slika z masko modre črte,
  centroidom, regijami za smeri in trenutnim stanjem,
- `/blue_line/status` - tekstovni opis trenutnega načina, števila modrih
  pikslov, napake centroida in aktivnih smernih regij.

Ti izhodi so bili uporabljeni pri nastavljanju pragov, preverjanju izbire
smeri na razcepih in ugotavljanju, ali je robot v fazi iskanja, sledenja
ali obnovitvenega obrata.

### 2.6 Izogibanje rumeni črti

Izogibanje rumeni črti je namenjeno varnemu gibanju robota v prvi sobi,
kjer robot ne sme zapeljati čez rumeno označena prepovedana območja.
Funkcionalnost je implementirana kot ločeno vozlišče `yellow_line_avoider`.
To vozlišče ne nadomešča Nav2 planiranja poti, temveč deluje kot reaktivni
varnostni sloj nad običajno navigacijo. Ko rumena črta ni nevarno blizu,
je vozlišče pasivno in ne objavlja hitrostnih ukazov. Ko zazna, da se je
robot preveč približal rumeni črti, pa začasno prevzame nadzor nad
gibanjem in robota umakne nazaj.

Glavni deli delovanja so:

- **Zajem slike:** rumena črta se zaznava iz slike zgornje kamere
  (`/top_camera/rgb/preview/image_raw`), ki je usmerjena proti tlom.
  Kamera omogoča neposreden pogled na območje pred robotom, kjer bi lahko
  robot prečkal prepovedano oznako.

- **Barvna segmentacija:** slika se pretvori v HSV barvni prostor, kjer
  se izdela maska za rumeno barvo. Uporabljen je prag približno
  `H = 18-35`, `S >= 100`, `V >= 80`, kar zajame rumeno talno oznako in
  zmanjša vpliv manj nasičenih površin.

- **Čiščenje maske:** maska se obdela z morfološkim zapiranjem in
  odpiranjem, s čimer se odstranijo šum in manjši napačni segmenti.

- **Nevarno območje:** sistem ne reagira na vsako rumeno zaznavo v sliki,
  temveč preverja samo spodnji sredinski del slike. To območje predstavlja
  del tal neposredno pred robotom. Če je v njem več kot nastavljeno število
  rumenih pikslov, se črta obravnava kot nevarno blizu.

Delovanje je organizirano v dve glavni stanji:

- `CLEAR` - rumena črta ni v nevarnem območju. Vozlišče ne objavlja
  hitrostnih ukazov, zato lahko Nav2, patrola ali druga vozlišča normalno
  vodijo robota.

- `BACKING` - rumena črta je zaznana v nevarnem območju. Robot se najprej
  ustavi, nato pa se z majhno hitrostjo premakne nazaj. Privzeta hitrost
  umika je približno `0,12 m/s`, umik pa traja približno `1,8 s`.

Pri umiku vozlišče objavlja ukaze na dve temi: `/cmd_vel_unstamped` in
`/cmd_vel`. S tem lahko začasno preglasi tako ukaze vozlišč paketa
`task1` kot tudi izhod Nav2 sistema. Ukazi se med umikanjem objavljajo z
visoko frekvenco, zato ima varnostni odziv prednost pred običajno
navigacijo. Ko se umik zaključi, vozlišče pošlje ničelno hitrost in se
vrne v stanje `CLEAR`.

Vozlišče se lahko omogoči ali onemogoči prek teme `/yellow_line_enabled`.
Med stanji, kjer rumena črta ni relevantna za navigacijo, na primer med
prehodom na delovno postajo ali med sledenjem modri črti, se varovalo ne
uporablja. Na ta način ne moti drugih delov sistema, ki imajo svoje
lokalno krmiljenje.

Za spremljanje in razhroščevanje se uporabljata:

- `/yellow_line/debug_image` - označena slika z rumeno masko, nevarnim
  območjem in trenutnim stanjem,
- `/yellow_line_status` - tekstovni opis stanja, predvsem `CLEAR` ali
  `BACKING`, ter število rumenih pikslov v nevarnem območju.

Ob sprožitvi varovala robot dodatno izpiše oziroma izgovori opozorilo
`Prohibited`, kar olajša razumevanje, zakaj se je navigacija nenadoma
ustavila ali umaknila. Celoten pristop je preprost, vendar učinkovit:
Nav2 še vedno skrbi za globalno navigacijo, `yellow_line_avoider` pa
prepreči zadnji korak čez prepovedano rumeno črto.

### 2.7 Zaznavanje poškodb ploščic

Za zaznavanje poškodb na ploščicah delovnih postaj smo trenirali
**U-Net** segmentacijski model \[4\]. U-Net je konvolucijska nevronska
mreža z arhitekturo enkoder-dekoder in preskočnimi povezavami (skip
connections), ki ji omogočajo natančno segmentacijo na ravni pikslov.

Model je bil treniran za tri razrede poškodb. Vhod je normalizirana
slika 512×512 px s **CLAHE** predprocesiranjem (Contrast Limited
Adaptive Histogram Equalization, clip_limit=4). Izhod je binarna
segmentacijska maska poškodbe. Morfološka poobdelava (odpiranje 3×3,
zapiranje 5×5) je bila uporabljena za čiščenje napovedi in eliminacijo
šuma. Prag zaznave je bil določen z iskanjem po mreži vrednosti \[0,20
... 0,70\] na validacijski množici in optimiziran po metriki IoU
(Intersection over Union).

### 2.8 Lokalizacija objektov v map frame

Vsi zaznani objekti so bili lokalizirani v globalnem koordinatnem
sistemu karte (map frame) s pomočjo: - **TF2** transformacij med
okvirjem kamere in map frame, - **grozdevanja** (clustering)
ponavljajočih se zaznav z mediano koordinat za odpornost na osamelce, -
**pragov potrditve**: objekt je potrjen šele po minimalnem številu
zaznav (obrazi: ≥5, obroči: ≥6, sodi: ≥10).

### 2.9 Inšpektor ploščic

Inšpekcija ploščic je večstopenjski proces, ki se sproži na zahtevo po
prebrani QR kodi in zajema navigacijo do delovne postaje, fino
pozicioniranje ob tekočem traku, sistematično skeniranje ploščic ter
ekstrakcijo čistih slik za klasifikacijo poškodb.

**Zaznavanje in pomnjenje delovnih postaj.** Med patruliranjem sistem že
vnaprej zaznava lokacije delovnih postaj. Vsaka postaja je označena z
rdečo ali zeleno barvno črto na tleh. Zaznavanje poteka s HSV barvno
segmentacijo slike iz OAK-D kamere, pri čemer se zaznani barvni segmenti
dodatno filtrirajo po dolžini (min. 1,5 m), saj kratki segmenti
pripadajo drugim objektom (npr. sodom). Za vsak segment se iz oblaka
točk s PCA analizo izračuna centroid in smerni vektor, z ravnostnim
filtrom pa se preveri, da gre res za talno oznako. Lokacija se potrdi
šele po večkratni zaznavi (najmanj 10), s čimer se izloči šum; končna
pozicija se določi kot mediana vseh kandidatov. Iz potrjene lokacije se
izračuna pristopna točka, ki je odmaknjena približno 1 m od linije v
smeri proti robotu --- tako dobi sistem varno razdaljo za začetek finega
pozicioniranja.

**Orkestracija.** Ko robot prebere QR kodo z navodilom za inšpekcijo
določene delovne postaje (rdeče ali zelene), se zahteva shrani v vrsto.
Izvedba se ne začne takoj: sistem najprej počaka, da je patrola končana
in da robot vstopi v namensko stanje za delovne postaje. Šele nato se
sproži inspekcijska sekvenca. Če je v vrsti več zahtev, se izvajajo
zaporedno.

**Navigacija in fino pozicioniranje.** Robot se najprej z navigacijo
Nav2 pripelje do vnaprej shranjene pristopne točke. Sledi petstopenjsko
fino pozicioniranje:

1.  *Približevanje tekočemu traku:* robot vozi naravnost naprej, dokler
    LiDAR ne zazna ovire na razdalji približno 0,30 m -- takrat ve, da
    je dosegel trak.
2.  *Poravnava orientacije:* robot se zavrti na mestu do vnaprej
    določenega ciljnega kota, ki ustreza smeri tekočega traku za izbrano
    barvo postaje. Orientacija se sproti preverja prek TF transformacij.
3.  *Fina poravnava s kamero:* s pomočjo Houghove transformacije na
    sliki zgornje kamere robot zazna linije ploščic na tekočem traku
    in se zavrti tako, da je nagib minimalen (pod 0,5°). S tem doseže
    natančno vzporednost s trakom.
4.  *Vzvratna vožnja:* robot vozi nazaj, dokler z OAK-D kamero ne zazna
    rumene črte na tleh, ki označuje začetek območja skeniranja. Če
    rumene črte ne zazna, se ustavi ob zaznavi ovire v zadnjem LiDAR
    stožcu.
5.  *Skeniranje:* robot vozi počasi naprej vzdolž traku in hkrati s
    spodnjim vidnim poljem OAK-D kamere preverja, ali je še v območju
    delovne postaje (zaznava rdeče oziroma zelene barve). Ko barvni
    marker izgine, robot ve, da je dosegel konec postaje, in se po kratki
    dodatni vožnji (za zajem morebitnih zadnjih ploščic) ustavi. Med
    skeniranjem se ob vsaki zaznani ploščici za kratek čas ustavi
    (2--4 s), da kamera zajame stabilno sliko brez gibalne zamegljenosti.

**Ekstrakcija slik ploščic.** Detekcija posamezne ploščice poteka s
pragovno segmentacijo (Otsu) na sliki zgornje kamere, ki gleda
pravokotno na ploščice. Svetle ploščice se ločijo od temne podlage
tekočega traku. Za robustnost se pred detekcijo uporabi svetlostni
sprožilec -- ploščica se išče le, če je v kontrolnem območju slike
dovolj visoka svetlost, kar prepreči lažne zaznave. Ko je ploščica
zaznana, se njene natančne robne točke določijo s konveksno ogrinjačo
in Douglas-Peucker aproksimacijo, kar da štiri oglišča tudi, če rotiran
okvir (minAreaRect) ne zajame natančno robov. Nato se z izračunom
homografije izvede perspektivna korekcija, ki ploščico projicira v
fronto-paralelni pogled -- s tem se odpravi morebitna perspektivna
deformacija in ploščica je pripravljena za klasifikator v
standardizirani obliki. Dodatno se robovi zožijo za približno 6 %, da
se izreže morebitno ozadje ob robovih. Zajem slike se izvede z
zakasnitvijo 0,5 s po ustavitvi robota, s čimer se počaka, da se
tresljaji umirijo in je slika res ostra.

**Zaključek.** Po končanem skeniranju se robot obrne za približno 130°
stran od traku in se odpelje na varno razdaljo, s čimer zapusti
delovno območje. Inspekcija se zaključi, rezultati klasifikacije pa se
zabeležijo v končno poročilo.

------------------------------------------------------------------------

## 3. Implementacija in integracija

### 3.0 Pregled arhitekture

Sistem je implementiran kot sklop ROS 2 vozlišč, večinoma v paketu `task1`,
z dodatnimi vozlišči za zaznavanje oseb in segmentacijo cilindrov. Posamezna
vozlišča imajo jasno ločene naloge: navigacija, zaznavanje, lokalizacija
objektov, varnostni odzivi, pregled delovne postaje in priprava končnega
poročila. Spodnja tabela povzema glavne komponente, ki so pomembne za
integracijo celotnega sistema.

| Vozlišče | Vloga v sistemu |
|---|---|
| `waypoint_navigator` | Izvaja obhod po vnaprej določenih točkah prek Nav2 akcije `NavigateToPose`. |
| `behavior_manager` | Centralno vodi stanja robota in preklaplja med patruljo, začasnimi cilji, delovno postajo, zaključno točko ter sledenjem modri črti. |
| `detect_people` | Zazna osebe v sliki in objavi njihove 3D lokacije. |
| `face_recognizer` | Prepozna identiteto in spol oseb iz slike kamere. |
| `face_localizator` | Združuje zaznave obrazov in potrjene lokacije objavi v koordinatnem sistemu `map`. |
| `detect_rings_v2` | Zaznava obroče z računalniškim vidom in globinsko verifikacijo. |
| `ring_localizator` | Združuje in potrjuje zaznave obročev. |
| `cylinder_segmentation` | Iz oblaka točk segmentira cilindrične objekte in objavi začetne markerje sodov. |
| `cylinder_localizator` | Združuje zaznave sodov, določi barvo, orientacijo in pripravi podatke za poročilo. |
| `barrel_inspector` | Izvede preverjanje razlitja pri ležečih sodih. |
| `line_localizator` | Zaznava barvne talne linije in jih objavlja kot markerje. |
| `workstation_recorder` | Iz zaznanih linij določi pristopne poze delovnih postaj. |
| `yellow_line_avoider` | Deluje kot reaktivni varnostni sloj za preprečevanje prečkanja rumene črte. |
| `blue_line_explorer` | Po zaključku prve sobe prevzame vizualno sledenje modri črti. |
| `station_inspector` | Vodi celoten tok pregleda delovne postaje in fino pozicioniranje robota. |
| `tile_detect` | Iz slike zgornje kamere izreže posamezne ploščice na tekočem traku. |
| `tile_classifier` | S klasifikacijskim modelom zazna poškodbe ploščic. |
| `qr_reader` | Bere QR kode z navodili in zaključno kodo za generiranje poročila. |
| `report_manager` | Zbira rezultate in generira končno poročilo. |
| `robot_state_overlay` | V RViz prikazuje trenutno stanje robota. |
| `orchestrator` | Skrbi za višjenivojsko koordinacijo procesov, ki se zaženejo po potrebi. |

Komunikacija med vozlišči poteka predvsem prek:

- **tem** za slike, markerje, stanja, zaznave in hitrostne ukaze,
- **akcije** `NavigateToPose` za ciljno navigacijo prek Nav2,
- **parametrov** za nastavljanje pragov, hitrosti in kamer,
- **pomožnih skript** za testiranje posameznih modulov in pripravo karte.

### 3.1 Navigacija

Implementacija navigacije je razdeljena med več ROS 2 vozlišč, pri čemer
Nav2 skrbi za samo planiranje in izvajanje poti, vozlišče
`behavior_manager` pa skrbi za odločanje, kdaj se posamezen navigacijski
način vključi ali ustavi.

#### 3.1.1 Waypoint patrulja

Patruljiranje po prvi sobi izvaja vozlišče `waypoint_navigator.py`.
Vozlišče ob zagonu naloži seznam točk iz `config/waypoints.yaml`. Vsaka
točka vsebuje:

- `x`, `y` - položaj cilja v koordinatnem sistemu `map`,
- `yaw` - orientacijo robota na cilju,
- `pause` - čas mirovanja po prihodu na cilj.

Za vsako točko se sestavi `PoseStamped` cilj in pošlje na Nav2 akcijo
`navigate_to_pose` tipa `NavigateToPose`. Po uspešno doseženem cilju
vozlišče poveča indeks trenutne točke in po potrebi počaka nastavljeni
premor. Če ima več zaporednih ciljev enake koordinate in različne
orientacije, robot na isti fizični lokaciji izvede pregled v več smereh.

Patrulja se ne izvaja ves čas, temveč jo omogoča ali ustavlja
`behavior_manager` prek teme `/patrol_enabled`. Ko so opravljene vse
točke, `waypoint_navigator` objavi `/patrol_finished`, kar sproži prehod
na naslednji del naloge.

#### 3.1.2 Centralni nadzor stanj

Vozlišče `behavior_manager.py` je centralni nadzornik navigacijskega
toka. Objavlja trenutno stanje robota na `/robot_state`, hkrati pa
upravlja, ali je aktivna patrulja, začasni pristop do zaznanega objekta,
prehod na delovno postajo ali sledenje modri črti.

Najpomembnejša stanja za navigacijo so:

- `IDLE` - robot miruje in čaka na začetek naloge,
- `PATROL` - aktivna je waypoint patrulja,
- `APPROACH_FACE` / `INTERACT_FACE` - robot se približa osebi in izvede
  interakcijo,
- `APPROACH_BARREL` / `INTERACT_BARREL` - robot se približa ležečemu
  cilindru oziroma sodu,
- `APPROACH_WORKSTATION` - robot se pelje do pristopne točke delovne
  postaje,
- `FINISHING_ROUNDS` - robot gre proti zaključni točki pred drugo sobo,
- `FOLLOW_BLUE_LINE` - nadzor prevzame sledenje modri črti.

`behavior_manager` spremlja tudi `/amcl_pose`, saj brez veljavne poze
robota ne more izračunati varnih pristopnih točk do zaznanih objektov.
Če navigacijski strežnik Nav2 še ni pripravljen ali AMCL poza še ni
znana, se začasni cilji ne pošiljajo.

#### 3.1.3 Začasni cilji med patruljo

Med patruljo `behavior_manager` prejema potrjene zaznave objektov iz
lokalizacijskih vozlišč. Za navigacijo so pomembne predvsem teme:

- `/detected_face_locations` - potrjene lokacije obrazov,
- `/detected_cylinder_locations` - potrjene lokacije cilindrov oziroma
  sodov,
- `/patrol_group_end` - signal, da je robot zaključil pregled ene
  waypoint lokacije,
- `/target_done` in `/resume_patrol` - zaključek interakcije in vrnitev v
  patruljo.

Pri obrazu se patrulja začasno ustavi, iz orientacije obraza in trenutne
poze robota pa se izračuna pristopna točka pred osebo. Ta se pošlje kot
začasni `NavigateToPose` cilj. Po prihodu na cilj robot preide v
interakcijo, nato pa se obraz doda na seznam že obdelanih ciljev, da ga
sistem ne obišče ponovno.

Pri obročih navigacija praviloma ne pošlje dodatnega cilja, saj se
obroči samo zaznajo, lokalizirajo in zabeležijo med patruljo. Zato
zaznava obroča ne prekine waypoint navigacije.

Pri cilindrih se pokončni cilindri samo zabeležijo. Če je cilinder
zaznan kot ležeč, se ustvari začasni cilj ob cilindru z varnim odmikom,
da se robot objektu približa, vendar ne zapelje neposredno vanj. Po
zaključku obdelave se robot vrne v stanje patrulje.

#### 3.1.4 Prehod do delovne postaje

Ko QR bralnik objavi navodilo na `/qr`, si `behavior_manager` zapomni,
ali mora robot obiskati rdečo ali zeleno delovno postajo. Prehod se ne
izvede takoj, temveč šele po zaključku patrulje. Takrat se iz
`/workstation_markers` oziroma že znanih pristopnih poz izbere ustrezna
postaja in pošlje Nav2 cilj tipa `approach_workstation`.

Ta del implementacije pokriva samo navigacijski prihod do delovne
postaje. Vse nadaljnje fino pozicioniranje, pregled tekočega traku in
obdelava ploščic so opisani v poglavju 3.9.

#### 3.1.5 Rumena in modra črta v navigacijskem toku

Rumena črta je implementirana kot reaktivni varnostni sloj v vozlišču
`yellow_line_avoider.py`. Vozlišče spremlja sliko zgornje kamere in v
stanju `CLEAR` ne posega v navigacijo. Če rumeno črto zazna v nevarnem
območju slike, preide v stanje `BACKING`, objavi hitrostne ukaze na
`/cmd_vel_unstamped` in `/cmd_vel` ter robota za kratek čas umakne nazaj.
S tem lahko začasno preglasi običajne ukaze patrulje ali Nav2.

Po zaključku navigacije po prvi sobi `behavior_manager` robota pošlje na
zaključno točko, nato pa omogoči vozlišče `blue_line_explorer.py` prek
teme `/blue_line_enabled`. Takrat se stanje robota spremeni v
`FOLLOW_BLUE_LINE`, nepotrebna zaznavna vozlišča se ustavijo, nadzor nad
gibanjem pa prevzame vizualno sledenje modri črti. `blue_line_explorer`
objavlja ukaze na `/cmd_vel_unstamped` in stanje/debug informacije na
`/blue_line/status` ter `/blue_line/debug_image`.

S tem je navigacija v implementaciji razdeljena na tri povezane dele:
Nav2 za globalne premike po karti, `behavior_manager` za preklapljanje
med fazami in namenski reaktivni moduli za posebna pravila gibanja
(rumena in modra črta).

### 3.2 Zaznavanje in prepoznavanje obrazov

Sistem za zaznavanje in prepoznavanje obrazov je sestavljen iz treh vzporedno delujočih vozlišč, ki skupaj tvorijo pipeline: `detect_people` zazna osebo in jo lokalizira v 3D prostoru, `face_recognizer` določi identiteto in spol, `face_localizator` pa zaznave združi v potrjene lokacije v globalnem koordinatnem sistemu.

---

#### 3.2.1 Detekcija oseb in 3D lokalizacija (DetectPeople)

**DetectPeople** (`detect_people.py`) je prvo vozlišče v verigi. Naroči se na RGB sliko (`/oakd/rgb/preview/image_raw`) in oblak točk (`/oakd/rgb/preview/depth/points`).

**Postopek detekcije:**

1. Vsak RGB okvir se posreduje modelu YOLOv8n z vhodom `imgsz=(256, 320)`. Iščejo se samo detekcije razreda 0 (person).
2. Za vsak zaznani bounding box se izračuna center `(cx, cy)`.
3. Iz oblaka točk se prebere 3D točka na koordinatah `(cx, cy)` — to je prostorska lega osebe v okvirju kamere.
4. Lokacija se objavi kot `Marker` na temo `/people_marker` z `frame_id = base_link`.

Vozlišče sprejema parameter `device`, ki se posreduje YOLO modelu za izbiro inference platforme.

---

#### 3.2.2 Prepoznavanje identitete in spola (FaceRecognizer)

**FaceRecognizer** (`face_recognizer.py`) se vzporedno naroči na RGB sliko (`/oakd/rgb/preview/image_raw`) in markerje iz `/people_marker`. Prepoznavanje poteka pri **2 Hz** (throttle = 0,5 s, ker je HOG detekcija računsko zahtevna).

**Referenčna baza oseb:**

Ob zagonu vozlišče prebere vse datoteke `.png` iz mape `config/personnel/`. Ime datoteke kodira identiteto po shemi `ime_zaimek1_zaimek2_vloga.png` (npr. `jana_she_her_manager.png`). Za vsako sliko se z `face_recognition.face_encodings()` izračuna **128-dimenzionalni enkoder**, ki se shrani skupaj z imenom, vlogo, zaimki in izpeljanim spolom.

**Postopek prepoznavanja za vsak okvir:**

1. `face_recognition.face_locations()` z HOG detektorjem poišče vse obraze v sliki.
2. `face_recognition.face_encodings()` izračuna 128-dimenzionalni enkoder za vsak zaznani obraz.
3. **Zaznavanje spola** iz izrezanega dela slike: Caffe model `gender_net` (vhod 227×227 px, normalizacija z vrednostmi `[78.4, 87.8, 114.9]`) vrne binarno klasifikacijo moški/ženski.
4. **Iskanje identitete**: evklidska razdalja med enkodiranjem zaznanega obraza in vsemi referenčnimi enkodirji. Prag zaupanja je privzeto `tolerance = 0,6`. Če se zaznani spol razlikuje od spola najboljšega kandidata v bazi, se prag zaostrí na `0,45`, s čimer se preprečijo napačne prepoznave med osebami različnih spolov s sicer podobnimi enkodirji.
5. Uspešna prepoznava se objavi na `/recognized_person` kot JSON s polji `name`, `pronouns`, `role`, `gender`, `confidence`, `map_x`, `map_y`. Objava je omejena na enkrat na 2 sekundi na posamezno osebo, da se prepreči poplava sporočil.

Za razhroščevanje se na `/face_debug_image` objavlja anotiran okvir z barvnimi pravokotniki (zeleni = prepoznana oseba, modri = neznan obraz), imenom, spolom in vlogo.

---

#### 3.2.3 Grozdevanje in potrditev lokacij (FaceLocalizator)

**FaceLocalizator** (`face_localizator.py`) je zadnji člen verige. Naroči se na `/people_marker` (lokacije iz DetectPeople) in `/recognized_person` (identitete iz FaceRecognizer).

**Grozdevanje zaznav:**

1. Vsak marker iz `/people_marker` se s TF2 transformacijo pretvori v koordinatni sistem `map`.
2. Zaznava se zavrže, če je **razdalja do robota večja od 2,5 m** (`max_detection_range`), saj so oddaljene zaznave nezanesljive.
3. Nova zaznava se doda obstoječemu grozdu, če je znotraj **radija 0,6 m** (`cluster_radius`). Sicer se ustvari nov grozd.
4. **Centroid grozda** se izračuna kot **mediana** vseh točk po x, y in z — mediana je odporna na osamelce.

**Potrditev osebe:**

Ko grozd doseže **≥5 zaznav** (`threshold_detections`), se preveri, ali na razdalji 0,8 m (`duplicate_radius`) v `marked_locations` že obstaja potrjena lokacija. Če ne:

1. Iz hrambe zadnjih 10 prepoznav (`recent_recognitions`) se poišče najboljša prepoznava v bližini (po zaupanju).
2. Iz pozicij robota in centroida grozda se izračuna **smer pogleda osebe** (`face_yaw = atan2(robot_y - y, robot_x - x)`) — kot, pri katerem je robot videl obraz, ki ga behavior_manager uporabi za izračun frontalne pristopne poze.
3. Potrjena lokacija se shrani v `marked_locations` in objavi na `/detected_face_locations` kot dva markerja: zelena krogla (SPHERE, premer 0,3 m) in lebdeče besedilo (TEXT_VIEW_FACING) z imenom osebe.

**Posodobitev identitete po potrditvi:** Vsaka nova prepoznava iz `/recognized_person` se primerja z že potrjenimi lokacijami. Če prihaja z razdalje ≤0,8 m in ima višje zaupanje od dosedaj shranjenega, se marker v RViz2 posodobi z novim imenom — obraz, ki je bil sprva označen kot "Unknown", se tako naknadno identificira, ko se robot dovolj približa.

### 3.3 Zaznavanje obročev

Sistem za zaznavanje obročev je sestavljen iz dveh vozlišč: `detect_rings_v2` zazna kandidate v posameznem okviru s Houghevo transformacijo, `ring_localizator` pa zaznave združuje v stabilne, potrjene lokacije v globalnem koordinatnem sistemu.

---

#### 3.3.1 Zaznavanje obročev v sliki (RingDetectorV2)

**RingDetectorV2** (`detect_rings_v2.py`) se naroči na RGB sliko (`/oakd/rgb/preview/image_raw`), globinsko sliko (`/oakd/rgb/preview/depth`, 16UC1 v mm) in oblak točk (`/oakd/rgb/preview/depth/points`). Aktivno deluje le v stanjih `IDLE` in `PATROL`.

**Priprava globinske slike:**

1. Globinska slika se pretvori iz 16UC1 (mm) v float32 (m).
2. Slika se omeji na **zgornjo polovico** kadra — obroči visijo na stojalih, spodnja polovica vsebuje le tla in šum.
3. Globina se pretvori v **disparitetno sliko** (1/globina_m), kar ojača bližnje strukture in naredi krožne oblike bolj izrazite.
4. Disparitetna slika se normalizira na 8-bit in zglajena z Gaussovim jedrom 5×5.

**Zaznavanje krogov s Houghevo transformacijo:**

`cv2.HoughCircles` z nastavitvami `dp=1`, `minDist=40`, `param1=48`, `param2=25`, `minRadius=1`, `maxRadius=70` poišče krožne robove v disparitetni sliki.

**Validacija in barvna maska za vsak zaznani krog (`_evaluate_circle`):**

1. Krog se zavrže, če center leži preblizu roba slike (manj kot en radij od meje).
2. Ustvari se maska okroglega območja in se **razširi za 20 %** z morfološko dilatacijo — s tem se zajamejo robni piksli obroča.
3. Razširjena maska se kombinira z masko veljavnih globinskih točk (disparity > 0), s čimer se izločijo piksli brez globine.
4. **Barvni filter**: na HSV sliki se izdela maska za znane barve obročev (rdeča, zelena, modra, črna). Kombinirana maska se AND-a z barvno masko — ohranjeni so le piksli z veljavno globino in znano barvo.
5. Iz zamaskiranih pikslov se iz oblaka točk izračuna **povprečna 3D pozicija** obroča v okvirju kamere.

**Klasifikacija barve** poteka na osnovi maskiranih pikslov: izračuna se povprečna BGR vrednost, pretvori v HSV in razvrsti po preprosti HSV tabeli (rdeča, zelena, modra, črna; nizka nasičenost → črna).

Vsaka veljavna detekcija se objavi na `/ring_marker` z `frame_id = base_link` in **življenjsko dobo 200 ms** — markerji so namenjeni samo za takoj. Trajno hrambo prevzame `ring_localizator`.

---

#### 3.3.2 Grozdevanje in potrditev lokacij (RingLocalizator)

**RingLocalizator** (`ring_localizator.py`) se naroči na `/ring_marker` in akumulira zaznave v grozdih v koordinatnem sistemu `map`.

**Grozdevanje:**

Vsak prejeti marker se s TF2 transformira v map frame. Nova zaznava se doda obstoječemu grozdu, če je znotraj **radija 1,0 m** (`CLUSTER_RADIUS`). Sicer se ustvari nov grozd. Centroid se posodablja z inkrementalnim povprečjem.

**Trostopenjski sistem potrditve:**

| Stanje | Prag zaznav | Radij markerja | Prosojnost |
|---|---|---|---|
| `ring_candidate` | ≥2 | 0,110 m | 45 % |
| `ring_actionable` | ≥4 | 0,150 m | 85 % |
| `ring_confirmed` | ≥6 | 0,175 m | 100 % |

Za prehod v `ring_actionable` in `ring_confirmed` mora grozd prestati **test kompaktnosti**: vsaj 60 % točk mora biti znotraj radija 0,30 m od mediane grozda. S tem se izločijo lažni grozdi, ki so nastali iz zaznav istega obroča z zelo različnih zornih kotov.

**Preprečevanje podvajanja:** preden se grozd potrdi, se preveri, ali na razdalji manj kot 1,2 m (`MIN_MARK_DIST`) že obstaja drug potrjen obroč. Če je, se nov grozd označi kot potlačen (`suppressed`).

**Robustni centroid** potrjenega obroča se izračuna kot **mediana** kompaktnih točk (inlierjev) po x, y in z — mediana je odporna na osamelce iz slabih globinskih meritev.

V RViz2 se vsak obroč vizualizira kot `LINE_STRIP` v obliki kroga (36 točk) v ustrezni barvi obroča.

---

### 3.4 Zaznavanje sodov

Sistem za zaznavanje sodov je sestavljen iz štirih vozlišč: `cylinder_segmentation` zazna cilindrične oblike v oblaku točk, `cylinder_localizator` jih clustra in potrdi v map frame, `barrel_inspector` sproži inšpekcijo ležečih sodov, `pointcloud_viewer` pa izvede dejansko zaznavanje razlitja.

---

#### 3.4.1 Segmentacija cilindrov iz oblaka točk (CylinderSegmentation)

**CylinderSegmentation** (`cylinder_segmentation.cpp`) je C++ vozlišče, ki obdeluje oblak točk iz OAK-D kamere (`/oakd/rgb/preview/depth/points`). Med obračanjem (kotna hitrost > 0,2 rad/s) se obdelava preskoči, saj gibanje kamere povzroča artefakte.

**Predprocesiranje oblaka točk:**

1. **Passthrough filter**: ohranijo se točke v razponu x = [0, 2,5 m] in z = [-0,25, 0,5 m] — izreže se ozadje in tla.
2. **VoxelGrid downsampling** z ločljivostjo 1,5 cm × 1,5 cm × 1,5 cm zmanjša oblak ~4× in pohitri RANSAC.
3. **Odstranitev ravnin**: do 3 dominantne ravnine (tla, stene) se iterativno odstranijo z RANSAC algoritmom za ravnine (prag 0,02 m), da cilindrični segmentator ne pritrdi na robove sten.

**RANSAC segmentacija cilindrov:**

Normalni vektorji se ocenijo z iskanjem k=20 sosedov. `SACSegmentationFromNormals` iščemodel valja z nastavitvami: ciljna polmer 0,14 m ± 0,04 m (Gazebo sodi), `normal_distance_weight=0.1`, `distance_threshold=0.01`, max 150 iteracij. Brez omejitve osi — zaznani so tako pokončni kot ležeči sodi.

**Filtriranje lažnih zaznav:**

- **Z-razpon** inlierjev mora biti med 0,07 m in 0,70 m — izloči talne oznake in ravne objekte.
- **Barvni filter (HSV)**: iz povprečne RGB vrednosti inlierjev se izračuna nasičenost in svetlost. Objekt se zavrže, če nasičenost < 0,22 IN svetlost > 0,25 — s tem se izključijo sivi in bež zaboji, medtem ko črni sodi (nizka svetlost) prestanejo filter.
- **Orientacija**: iz komponent osi valja se izračuna `|az / |osi||`; vrednost > 0,5 pomeni pokončen sod, sicer ležeč.

Vsak veljavni cilindrični segment se objavi na `cylinder_markers` z `marker.text = "vertical"/"horizontal"` in življenjsko dobo 2 s.

---

#### 3.4.2 Grozdevanje in potrditev (CylinderLocalizator)

**CylinderLocalizator** (`cylinder_localizator.py`) sprejema markerje iz `cylinder_markers`, jih s TF2 transformira v map frame in accumula v grozdih.

**Grozdevanje:**

Nova zaznava se doda obstoječemu grozdu, ki ima enako orientacijo in združljivo barvo, če je znotraj radija **0,33 m** (pokončni sodi) oziroma **0,70 m** (ležeči, ker se centroid bolj razlikuje glede na zorni kot). Barva in orientacija grozda se sproti glasujeta: vsaka zaznava doda glas za svojo klasifikacijo, ob potrditvi zmaga večina.

**Potrditev in preprečevanje podvajanja:**

- Grozd se vizualizira kot kandidat po ≥4 zaznavah in potrdi po **≥10 zaznavah** (CONFIRM_THRESH).
- Za potrditev mora grozd prestati test kompaktnosti: vsaj 60 % točk mora biti znotraj 0,32 m od mediane.
- Pred potrditvijo se preveri, da drug potrjeni sod ni bližje kot 0,18 m (fizično prekrivanje) oziroma 0,48 m za isti tip/barvo (pokončni) ali 1,5 m (ležeči — centrodi bolj razpršeni).
- **Robustni centroid** se izračuna kot mediana kompaktnih inlierjev po x, y in z.

Potrjeni sod se objavi na `/detected_cylinder_locations` s poljem `marker.text` (orientacija) in barvno kodiranim RGB. Vzporedno se shrani vstop v JSON poročilo in slika iz OAK-D kamere.

---

#### 3.4.3 Inšpekcija sodov in zaznavanje razlitja

**BarrelInspector** (`barrel_inspector.py`) posluša `/detected_cylinder_locations`. Ob potrditvi soda:

- **Pokončni sod**: razlitje ni mogoče, rezultat se takoj objavi na `/barrel_inspection_result` z `leak_detected = false`.
- **Ležeči sod**: objavi se vmesni rezultat `null` (zaznava razlitja še ni znanana). Ko `behavior_manager` pripelje robota na ustrezno pristopno točko, pošlje sprožilec `/spill_check_trigger` z ID-jem soda.

**PointCloudSpillCheck** (`pointcloud_viewer.py`) streže servis `/spill_check` (tip `Trigger`). Ob klicu:

1. Vzame zadnji zajet oblak točk (`/oakd/rgb/preview/depth/points`).
2. **Filter razdalje**: ohranijo se le točke do 1,0 m od kamere.
3. **Transformacija v map frame** s TF2.
4. **Z-rezina**: izreže se horizontalni pas med 0,5 cm in 15 cm nad tlemi (`SLICE_Z_MIN=0.005`, `SLICE_Z_MAX=0.15`).
5. Razlitje je zaznano, če Z-rezina vsebuje **≥4000 točk** (`SPILL_POINT_THRESH`).

Rezina se objavi na `/slice_points` za vizualno preverjanje v RViz2. Rezultat se vrne `BarrelInspectorju`, ki sproži glasovni izhod (espeak) in shrani fotografijo ob ležečem sodu z razlitjem.

### 3.5 Zaznavanje poškodb ploščic

Sistem za zaznavanje poškodb ploščic temelji na U-Net segmentacijskem modelu, ki je bil posebej treniran za klasifikacijo površinskih razpok in strukturnih poškodb na dveh težavnostnih nivojih. Sistem obsega treniranje modela, evalvacijo in izbiro praga ter inferenčno vozlišče v ROS 2.

---

#### 3.5.1 Treniranje modela (unet_train.py)

Model je arhitekture **U-Net** z enkodirjem ResNet-34, predhodno naučenim na ImageNet (`segmentation_models_pytorch`). Vhod je RGB slika 512×512 px, izhod je binarna segmentacijska maska poškodbe.

**Podatkovni nabor:**

Slike so organizirane v tri razrede poškodb (`damaged_0`, `damaged_1`, `damaged_3`) ter razred nepoškodovanih ploščic (`okay_*`). Nabor se stratificirano razdeli v učno (80 %) in validacijsko (20 %) množico, da se ohrani razmerje med razredi v obeh delih.

**Predprocesiranje in augmentacije:**

Vsaka slika se predhodno obdela s **CLAHE** (clip_limit=4, tile_grid=8×8) za izboljšanje lokalnega kontrasta, nato normalizira z ImageNet parametri (μ=[0.485, 0.456, 0.406], σ=[0.229, 0.224, 0.225]). Med treningom se izvajajo naslednje augmentacije: horizontalni in vertikalni zrcalni odsev, rotacija za 90°, elastična transformacija, distorzija mreže, naključna sprememba svetlosti in kontrasta ter Gaussov šum.

**Funkcija izgube:**

Skupna izguba je kombinacija treh členov: `0.35 × BCE + 0.35 × Tversky + 0.3 × ClDice`. Tversky izguba (α=0.2, β=0.8) kaznuje napačno klasificirane poškodovane piksle bolj kot lažne pozitivne. **ClDice** (Centerline Dice) je posebna izguba, ki kaznuje missing skelete in konice razpok ter izboljša zveznost segmentiranih linij. BCE utež za pozitivne piksle (`pos_weight`) se izračuna iz razmerja negativnih in pozitivnih pikslov v učni množici (omejena na 10), kar kompenzira razredno neravnovesje.

**Optimizacija:**

Optimizer: Adam (`lr=1e-4`). Urnik: CosineAnnealingLR (`T_max=50`). Zgodnje ustavljanje po 10 epohah brez izboljšanja validacijskega Dice koeficienta. Najboljši model se shrani v `results/unet/best_model.pth`.

---

#### 3.5.2 Evalvacija in izbira praga (unet_evaluate.py)

Po treniranju skripta `unet_evaluate.py` izvede **pregled pragov** v razponu [0,20 ... 0,70] na testni množici. Za vsak prag se izračuna povprečni IoU čez vse razrede poškodb. Izbere se prag z najboljšim skupnim povprečnim IoU.

Za vsak tip poškodb se poroča: n slik, mean/median/min/max IoU. Po morfološki poobdelavi (odpiranje 3×3, zapiranje 5×5) se shranijo vizualizacijska primerjalna polja: originalna slika | napoved (modra) | referenčna maska (zelena).

---

#### 3.5.3 Inferenca v realnem času (TileClassifier)

**TileClassifier** (`tile_classifier.py`) je ROS 2 vozlišče, ki se naroči na `/tile_warped` — perspektivno korigirane slike ploščic, ki jih pošilja `tile_detect` med inšpekcijo (opisano v 3.9.4).

**Postopek klasifikacije za vsako ploščico:**

1. BGR slika se pretvori v RGB in obdela s transformacijami: CLAHE → resize 512×512 → normalizacija z ImageNet parametri.
2. U-Net inferenca na GPU/CPU z onemogočenim gradientom.
3. Sigmoidna aktivacija vrne verjetnostno masko.
4. Maska se binarizira s pragom **0,20** (`THRESHOLD`) in morfološko poobdela (odpiranje 3×3, zapiranje 5×5).
5. Izračuna se razmerje poškodovanih pikslov. Ploščica je klasificirana kot **`DEFECT`**, če je razmerje ≥ 0,2 % (`MIN_DEFECT_RATIO = 0.002`), sicer **`OK`**.
6. Rezultat se objavi na `/tile_classification` v obliki `"DEFECT:tile_id"` oziroma `"OK:tile_id"`, kjer je `tile_id` zaporedna številka ploščice.
7. Na `/tile_heatmap` se objavi vizualizacija: verjetnostna mapa JET barvne palete, preložena čez originalno sliko ploščice (50/50).


### 3.6 Implementacija sledenja modri črti

Sledenje modri črti je implementirano v vozlišču
`blue_line_explorer.py`. To vozlišče se ne uporablja kot del Nav2
planiranja, ampak kot ločen vizualni krmilnik, ki po zaključku navigacije
po karti prevzame neposredno objavljanje hitrostnih ukazov. Namenjeno je
drugemu prostoru, kjer robot ne sledi več vnaprej znanim waypointom,
temveč talni modri črti.

#### 3.6.1 Vključitev v navigacijski tok

Vozlišče `blue_line_explorer` se zažene že ob zagonu sistema v
`task1.launch.py`, vendar privzeto ni aktivno (`enabled_on_start=False`).
V normalnem poteku ga vključi `behavior_manager`, ko robot doseže
zaključno točko po prvi sobi. Takrat se izvede metoda
`_start_blue_line_runtime()`:

1.  globalno stanje robota se nastavi na `FOLLOW_BLUE_LINE`,
2.  ustavijo se vozlišča, ki v drugi sobi niso več potrebna
    (`waypoint_navigator`, zaznavanje obročev, cilindrov, rumeno varovalo,
    lokalizacija delovnih postaj itd.),
3.  aktivnim ostanejo vozlišča, ki so še potrebna: `blue_line_explorer`,
    `behavior_manager`, `robot_state_overlay`, `qr_reader`,
    `report_manager` ter zaznavanje oseb,
4.  na temo `/blue_line_enabled` se objavi `True`, s čimer
    `blue_line_explorer` začne izvajati vizualno vodenje.

Ob aktivaciji `blue_line_explorer` objavi tudi:

- `/manual_control_active = True`, da sistem ve, da trenutno hitrostne
  ukaze prevzema lokalni krmilnik in ne Nav2,
- `/patrol_command = False`, da se patrulja ne more ponovno zagnati,
- `/robot_state = FOLLOW_BLUE_LINE`, kar uporablja tudi vizualni prikaz
  stanja v RViz.

Vozlišče ima sicer naročnino tudi na `/patrol_finished`, vendar se ta
samodejni zagon uporabi samo, če je parameter `start_on_patrol_finished`
nastavljen na `True`. V glavni integraciji se uporablja eksplicitni vklop
prek `behavior_manager`.

#### 3.6.2 Vhodi, izhodi in parametri

Glavni vhod za sledenje je slika zgornje kamere na robotski roki:
`/top_camera/rgb/preview/image_raw`. Slika se prejema z `BEST_EFFORT`
QoS profilom, ker je za krmiljenje pomembnejša sveža slika kot pa
zanesljiva dostava vsakega okvirja. Poleg slike vozlišče uporablja še
LiDAR temo `/scan`, stanje robota `/robot_state`, aktivacijsko temo
`/blue_line_enabled` in po možnosti temo `/hazard_detection` za zaznavo
trka.

Izhodi vozlišča so:

- `/cmd_vel_unstamped` - neposredni hitrostni ukazi robota,
- `/robot_state` - objava globalnega stanja med sledenjem črti,
- `/manual_control_active` - signal, da je aktiven lokalni krmilnik,
- `/patrol_command` - izklop patrulje med sledenjem,
- `/blue_line_finished` - signal ob izklopu vozlišča,
- `/blue_line/debug_image` - označena diagnostična slika,
- `/blue_line/status` - tekstovni opis trenutnega internega načina,
  števila modrih pikslov, napake centroida in aktivnih smernih regij.

Najpomembnejši parametri so:

- barvni prag `blue_lo=[82, 120, 60]` in `blue_hi=[102, 255, 255]`,
- dodatni pogoj dominantnosti modro-cian barve
  (`use_blue_dominance=True`, `blue_dominance_margin=40`,
  `blue_min_channel=80`),
- minimalno število modrih pikslov `min_blue_pixels=150`,
- hitrosti `linear_speed=0.15` in `fast_linear_speed=0.40`,
- proporcionalni faktor krmiljenja `kp_steer=1.2`,
- omejitev kotne hitrosti `max_angular_speed=0.7`,
- dodatni levi odklon na razcepih `left_bias=0.25`,
- glajenje kotnega ukaza `angular_smoothing=0.35`,
- čas za izhod iz razcepa `split_exit_hold=0.5 s`,
- čas izgubljene črte pred obnovitvijo `line_lost_timeout=2.0 s`,
- trajanje obrata `uturn_duration=6.5 s`.

#### 3.6.3 Zaznavanje modre črte v sliki

Zaznavanje se izvaja v metodi `_detect()`. Vozlišče vedno obdeluje
najnovejšo prejeto sliko. Po potrebi se zgornji del slike odreže s
parametrom `roi_top_fraction`, nato se nad preostalim območjem izvede
barvna segmentacija:

1.  slika se pretvori iz BGR v HSV,
2.  izdela se maska znotraj praga `blue_lo`--`blue_hi`,
3.  če je omogočen `use_blue_dominance`, se izdela še dodatna maska, kjer
    morata biti modri in zeleni kanal dovolj visoka ter oba za nastavljeni
    rob večja od rdečega kanala,
4.  HSV maska in maska dominantnosti se združita z logičnim OR,
5.  nad masko se izvedeta morfološko odpiranje in zapiranje z eliptičnim
    jedrom velikosti 5x5, kar odstrani manjši šum in zapolni drobne luknje.

Po čiščenju maske se prešteje število modrih pikslov. Če jih je manj kot
`min_blue_pixels`, se črta obravnava kot nezanesljivo zaznana in metoda
vrne `None`. Če je črta dovolj velika, se iz momentov maske izračuna njen
centroid. Horizontalni odmik centroida se normalizira v interval približno
od -1 do 1:

```text
cx_norm = (cx - širina_slike / 2) / (širina_slike / 2)
```

Vrednost `cx_norm` predstavlja napako krmiljenja. Če je centroid levo od
sredine slike, je napaka negativna; če je desno, je pozitivna. Ta napaka
se kasneje uporabi za izračun kotne hitrosti robota.

#### 3.6.4 Zaznavanje smeri in razcepov

Poleg centroida vozlišče preverja še tri regije zanimanja, ki predstavljajo
možne smeri nadaljevanja:

- leva regija: vodoravno območje od 0 % do 33 % širine slike,
- sredinska regija: območje od 34 % do 66 % širine slike,
- desna regija: območje od 67 % do 100 % širine slike.

Leva in desna regija uporabljata navpični pas med
`branch_roi_top_fraction=0.42` in `branch_roi_bottom_fraction=0.90`,
sredinska regija pa pas med `straight_roi_top_fraction=0.20` in
`straight_roi_bottom_fraction=0.65`. Regija je aktivna, če vsebuje vsaj
`min_branch_pixels=50` modrih pikslov in hkrati pokritost vsaj
`min_branch_ratio=0.008`.

Če je aktivna leva ali desna regija, vozlišče preide v interni način
`split_active`. Takrat robot zmanjša hitrost na `linear_speed`, saj se
nahaja na razcepu ali se po razcepu še poravnava. Izhod iz razcepa se
zgodi šele, ko je aktivna sredinska regija, centroid pa je dovolj blizu
sredine (`abs(cx_norm) < 0.30`) za čas `split_exit_hold`.

Izbira smeri je izvedena konzervativno:

- ko je vidna leva veja, se osnovnemu krmiljenju doda `left_bias`, zato
  robot preferira levo smer,
- ko leva veja ni več vidna, se levi odklon odstrani in robot se ponovno
  poravnava samo po centroidu,
- če je vidna samo desna preostala veja brez sredinske smeri, se desni
  vpliv omeji, da robot po že izvedenem levem zavoju ne začne drseti nazaj
  proti desni.

S tem razcep ni obravnavan kot en sam trenutek, temveč kot kratka faza:
vstop v razcep, izbira smeri, poravnava in šele nato vrnitev v običajno
sledenje.

#### 3.6.5 Krmiljenje robota

V glavnem ciklu `_update()` se logika izvaja vsakih 0,1 s. Če vozlišče ni
aktivno, ne objavlja hitrosti. Ko je aktivno, vsakič ponovno objavi
`/manual_control_active=True` in `/patrol_command=False`, da ostane
patrulja izključena.

Krmiljenje uporablja proporcionalni zakon:

```text
angular_raw = -kp_steer * cx_norm
```

Na razcepu se temu po potrebi doda `left_bias`. Rezultat se omeji z
`max_angular_speed`, nato pa se zgladi z eksponentnim filtrom:

```text
angular = alpha * angular_raw + (1 - alpha) * previous_angular
```

V kodi je `alpha` nastavljen s parametrom `angular_smoothing`. Namen
filtra je, da kratke spremembe v maski ne povzročijo sunkovitega
obračanja robota. Linearna hitrost je `fast_linear_speed`, kadar robot
sledi jasni sredinski črti, in `linear_speed`, kadar je aktiven razcep ali
je pred robotom ovira v območju upočasnjevanja.

#### 3.6.6 Obnovitev poti in varnost

Interni stroj stanj v `blue_line_explorer` uporablja štiri načine:

- `IDLE` - vozlišče je izključeno,
- `SEARCH` - robot se počasi obrača in išče modro črto,
- `FOLLOW` - robot sledi zaznani črti,
- `UTURN` - robot izvaja obnovitveni obrat.

V stanju `SEARCH` robot počasi rotira v levo s polovico hitrosti
`uturn_angular_speed`, dokler maska ponovno ne zazna zanesljive modre
črte. V stanju `FOLLOW` se izvaja normalno krmiljenje po centroidu. Če
črta izgine, robot najprej ustavi hitrostne ukaze. Če je črta izgubljena
dlje kot `line_lost_timeout`, se sproži `UTURN`.

Obnovitveni obrat se sproži tudi v dveh varnostnih primerih:

- če LiDAR v ozkem sprednjem stožcu zazna oviro bližje od
  `lidar_stop_distance=0.37 m`,
- če je na temi `/hazard_detection` zaznan trk tipa `BUMP`.

Za LiDAR se upošteva kot `lidar_forward_angle_deg=-90°`, ker je RPLIDAR
na robotu zasukan, in stožec širine `lidar_cone_half_angle_deg=15°`.
Najmanjša veljavna razdalja v tem stožcu se shrani kot `front_dist`. Če
je razdalja manjša od `lidar_slow_distance=0.55 m`, robot še ne obrne,
ampak zmanjša linearno hitrost. Če pade pod zaustavitveni prag, preide v
`UTURN`, objavi stanje `BLUE_LINE_DEAD_END` in se vrti približno
`uturn_duration=6.5 s`. Po končanem obratu se vrne v `SEARCH`.

Pomembno je ločiti interna stanja vozlišča od globalnega stanja robota.
Med iskanjem in sledenjem vozlišče na `/robot_state` objavlja
`FOLLOW_BLUE_LINE`, ob obnovitvenem obratu pa `BLUE_LINE_DEAD_END`. RViz
prekrivni prikaz (`robot_state_overlay`) ta stanja prikaže z modro
obarvanim ozadjem, da je faza druge sobe jasno vidna.

#### 3.6.7 Prekinitve zaradi obrazov in QR zaključek

Tudi med sledenjem modri črti ostaneta aktivna zaznavanje oseb in
`qr_reader`. Če `behavior_manager` med stanji `FOLLOW_BLUE_LINE`,
`LINE_FOLLOWING` ali `BLUE_LINE_*` prejme potrjeno lokacijo obraza,
začasno izklopi `/blue_line_enabled`, ustavi neposredne hitrostne ukaze in
doda obraz kot začasni cilj. Pristop do osebe se izvede prek Nav2, po
zaključeni interakciji pa `behavior_manager` ponovno objavi
`/blue_line_enabled=True` in stanje vrne v `FOLLOW_BLUE_LINE`.

`qr_reader.py` spremlja `/robot_state`. Ko zazna stanje modre črte, preide
v način `blue_line`. V tem načinu QR kode išče na sliki OAK-D kamere
`/oakd/rgb/preview/image_raw`, pri čemer zaradi obremenitve obdela samo
vsak četrti okvir. Za branje uporablja najprej `WeChatQRCode`, nato pa
več poskusov z OpenCV `QRCodeDetector`: osnovno sivo sliko, izboljšano
sliko s CLAHE in binarizirano sliko z Otsujevim pragom.

QR kode v modri fazi se deduplicirajo, da ista koda ne sproži večkratnega
ukaza. Če prebrano besedilo vsebuje `report`, `qr_reader` objavi ukaz
`make` na `/report_commands`. S tem se sproži generiranje poročila. Po
približno treh sekundah nato ugasne vozlišča, ki niso več potrebna, in
pusti aktiven predvsem `report_manager`.

Za razhroščevanje se pri modri črti uporabljata dve vrsti izhodov:

- `/blue_line/debug_image` prikazuje modro masko, centroid, sredinsko
  referenco, smerne regije in LiDAR razdaljo,
- `/blue_line/status` v tekstovni obliki izpisuje interni način,
  `blue_pixels`, `cx_norm` in aktivne smeri.

Za samostojno testiranje obstaja še skripta `blue_line_test.sh`. Ta
nastavi položaj roke oziroma kamere, po potrebi zažene
`robot_state_overlay`, omogoči `blue_line_explorer` z
`enabled_on_start=True` in omogoči spremljanje `/blue_line/status`. Skripta
je uporabna za nastavljanje pragov in preverjanje obnašanja na razcepih
brez celotnega poteka naloge.

### 3.7 Implementacija izogibanja rumeni črti

Izogibanje rumeni črti je implementirano kot reaktivni varnostni sloj v
vozlišču `yellow_line_avoider.py`. Namen tega modula ni načrtovanje poti
po karti, temveč preprečevanje zadnjega premika čez rumeno prepovedano
oznako. Zato vozlišče v normalnem stanju ne posega v gibanje robota; ukaze
objavlja samo takrat, ko kamera zazna, da je rumena črta že v neposredni
bližini robota.

#### 3.7.1 Vloga v navigacijskem sistemu

`yellow_line_avoider` deluje vzporedno z Nav2 in waypoint patruljo.
Globalno pot še vedno načrtuje Nav2, rumeno varovalo pa spremlja spodnji
del slike zgornje kamere. Če v nevarnem območju zazna dovolj rumenih
pikslov, takoj objavi ničelno hitrost, nato pa za kratek čas objavlja
vzvratno hitrost. S tem robot ne nadaljuje čez rumeno črto, tudi če bi ga
Nav2 cilj ali ročno vodenje peljalo naprej.

Vozlišče je registrirano kot ROS 2 konzolni program
`yellow_line_avoider` v `setup.py`. V namenskem testnem toku se zažene s
skripto `yellow_line_avoider.sh`, ki najprej nastavi roko oziroma kamero v
položaj `look_for_spill`, odpre debug sliko in nato zažene vozlišče z
nastavljenimi parametri. `behavior_manager` ima pripravljeno temo
`/yellow_line_enabled`, prek katere lahko varovalo izklopi ob prehodu v
faze, kjer rumena črta ni več del navigacijskega problema.

V kodi se rumena barva pojavi še v drugih modulih, vendar z drugačnim
namenom. `line_localizator.py` zna zaznavati rumene linije kot splošne
barvne talne segmente, `station_inspector.py` pa rumeno črto uporablja kot
lokalni pogoj pri finem pozicioniranju ob delovni postaji. To ni isti
mehanizem kot varovalo `yellow_line_avoider`, zato je podrobna obravnava
delovne celice ločena v poglavju 3.9.

#### 3.7.2 Vhodi, izhodi in parametri

Glavni vhod je slika zgornje kamere:
`/top_camera/rgb/preview/image_raw`. Slika se prejema z `BEST_EFFORT`
QoS profilom in globino 1, saj je za varnostni odziv pomembno, da se
obdeluje čim bolj svež okvir. Vozlišče se naroča tudi na:

- `/robot_state` - trenutno globalno stanje robota,
- `/yellow_line_enabled` - omogočanje ali onemogočanje varovala.

Izhodi vozlišča so:

- `/cmd_vel_unstamped` - hitrostni ukazi tipa `Twist`,
- `/cmd_vel` - hitrostni ukazi tipa `TwistStamped`,
- `/yellow_line_status` - tekstovni status z internim stanjem in številom
  rumenih pikslov v nevarnem območju,
- `/yellow_line/debug_image` - diagnostična slika z masko, konturami,
  nevarnim območjem in stanjem.

Najpomembnejši parametri so:

- `enabled=True` - osnovno omogočanje vozlišča,
- `camera_topic=/top_camera/rgb/preview/image_raw`,
- `publish_debug_image=True`,
- `danger_zone_top=0.65` in `danger_zone_bottom=0.95`,
- `roi_left=0.30` in `roi_right=0.70`,
- `danger_px_threshold=300`,
- `back_speed=0.12 m/s`,
- `back_duration=1.8 s`.

Nevarno območje je torej spodnji sredinski del slike. Navpično zajema
približno 65 % do 95 % višine slike, vodoravno pa 30 % do 70 % širine.
S tem se izločijo rumeni objekti ali odboji ob robovih slike, robot pa
reagira predvsem na rumeno črto neposredno pred seboj.

#### 3.7.3 Zaznavanje rumene barve

Zaznavanje poteka v metodi `_update()`, ki se izvaja vsakih 0,02 s
oziroma s frekvenco 50 Hz. Najprej se preveri, ali sme vozlišče sploh
delovati. Če ni omogočeno, če je bilo izklopljeno prek
`/yellow_line_enabled` ali če je robot v stanju, kjer rumeno varovalo ni
relevantno, se obdelava ne izvede.

Ko je obdelava dovoljena, vozlišče vzame zadnjo prejeto sliko in jo
pretvori iz BGR v HSV. Rumena maska je določena s pragom:

```text
YELLOW_LO = [18, 100, 80]
YELLOW_HI = [35, 255, 255]
```

Po barvni segmentaciji se nad masko izvedeta morfološko zapiranje in
odpiranje z eliptičnim jedrom velikosti 5x5. Zapiranje poveže manjše
prekinitve na črti, odpiranje pa odstrani droben šum. Nato se v izrezanem
nevarnem območju prešteje število rumenih pikslov:

```text
danger_px = countNonZero(mask[danger_zone, roi_left:roi_right])
```

Če je `danger_px` manjši od `danger_px_threshold`, se stanje ohrani kot
`CLEAR`. Če preseže prag, se rumena črta obravnava kot nevarno blizu.

#### 3.7.4 Stroj stanj in odziv robota

Interni stroj stanj ima dve stanji:

- `CLEAR` - rumena črta ni v nevarnem območju; vozlišče je pasivno in ne
  objavlja hitrostnih ukazov,
- `BACKING` - rumena črta je zaznana v nevarnem območju; vozlišče začasno
  prevzame nadzor in robota umakne nazaj.

V stanju `CLEAR` se ne objavlja noben ukaz hitrosti. To je pomembno, ker
varovalo tako ne moti Nav2, patrulje ali ročnega vodenja. Ko število
rumenih pikslov preseže prag, vozlišče najprej objavi ničelno hitrost,
izpiše opozorilo in poskusi izgovoriti `Prohibited` z enim od razpoložljivih
govornih programov (`espeak-ng`, `espeak` ali `spd-say`). Nato nastavi
stanje `BACKING` in izračuna čas zaključka umika.

Med `BACKING` vozlišče pri vsaki iteraciji objavlja vzvratno hitrost
`-back_speed` in ničelno kotno hitrost. Ko poteče `back_duration`, objavi
ničelno hitrost, ponastavi zastavico za govor in se vrne v stanje
`CLEAR`.

#### 3.7.5 Preglas hitrostnih ukazov

Posebnost implementacije je, da varovalo med umikom objavlja na dve temi:

- `/cmd_vel_unstamped` kot `geometry_msgs/Twist`,
- `/cmd_vel` kot `geometry_msgs/TwistStamped`.

Prva tema preglasi ukaze drugih vozlišč paketa `task1` in ročnega
vodenja, druga pa preglasi izhod Nav2 sistema po `collision_monitor`.
Ukazi se objavljajo pri 50 Hz, kar je hitreje od običajnih ukazov Nav2
ali teleoperacije. S tem varnostni odziv v praksi zmaga v situaciji, ko
več vozlišč hkrati objavlja hitrostne ukaze. Ko je stanje `CLEAR`, se
hitrosti ne objavljajo, zato sistem ostane pasiven.

Ob uničenju vozlišča `destroy_node()` še enkrat objavi ničelno hitrost na
obe temi. Enako naredi tudi testna skripta `yellow_line_avoider.sh` v
funkciji `cleanup()`, da robot po prekinitvi testa ne ostane z zadnjim
veljavnim hitrostnim ukazom.

#### 3.7.6 Pogoji za delovanje v različnih fazah

Funkcija `_should_run()` določa, kdaj se varovalo izvaja. Poleg parametra
`enabled` in signala `/yellow_line_enabled` preverja tudi globalno stanje
robota. Varovalo se ne izvaja v stanjih:

- `APPROACH_WORKSTATION` in `WORKSTATION`,
- `APPROACH_FINAL` in `FINISHING_ROUNDS`,
- `LINE_FOLLOWING`, `FOLLOW_BLUE_LINE`,
- `BLUE_LINE_SEARCH`, `BLUE_LINE_FOLLOW`, `BLUE_LINE_DEAD_END`.

To pomeni, da je rumeno varovalo namenjeno predvsem navigaciji v prvi
sobi, kjer robot patruljira po waypointih. Ob prehodu na delovno postajo
in ob prehodu proti drugi sobi `behavior_manager` dodatno objavi
`/yellow_line_enabled=False`. V fazi modre črte se `yellow_line_avoider`
tudi ugasne med ustavljanjem nepotrebnih vozlišč, zato ne tekmuje z
vizualnim sledenjem modri črti.

#### 3.7.7 Diagnostika in testiranje

Za diagnostiko vozlišče objavlja `/yellow_line/debug_image`. V tej sliki
je rumena maska narisana čez originalno sliko, zaznane konture so
obrobljene, nevarno območje je označeno s pravokotnikom, zraven pa sta
izpisana trenutno stanje in število rumenih pikslov v nevarnem območju.
Če število pikslov preseže prag, je pravokotnik obarvan opozorilno.

Tema `/yellow_line_status` objavlja zapis oblike:

```text
state=CLEAR danger_px=...
state=BACKING danger_px=...
```

To je uporabno pri nastavljanju praga `danger_px_threshold`, preverjanju
kamere in razumevanju, zakaj se je robot ustavil ali umaknil.

Skripta `yellow_line_avoider.sh` omogoča samostojno testiranje. Nastavi
okolje ROS 2, po potrebi zažene poslušalca ukazov roke, premakne roko v
položaj `look_for_spill`, odpre `rqt_image_view` na
`/yellow_line/debug_image` in zažene:

```text
ros2 run task1 yellow_line_avoider
```

Pri tem lahko prek okoljskih spremenljivk nastavimo kamero, hitrost umika
in trajanje umika (`YELLOW_LINE_CAMERA_TOPIC`, `YELLOW_LINE_BACK_SPEED`,
`YELLOW_LINE_BACK_DURATION`).

#### 3.7.8 Podporna orodja za vris rumenih črt v mapo

Poleg reaktivnega varovala obstajata še podporni skripti za delo z rumeno
črto na karti. To ni glavni runtime mehanizem izogibanja, ampak dodatna
možnost za pripravo karte ali analizo prepovedanih območij.

`make_yellow_layer.py` prebere koordinate iz `yellow_lines_coords.yaml` in
rumene črte vriše neposredno v `map2.pgm` kot zasedene celice. Koordinate
so podane v svetovnem koordinatnem sistemu, skripta pa jih s pomočjo
origin-a in resolucije iz `map2.yaml` pretvori v slikovne koordinate.
Pred prvim spreminjanjem naredi varnostno kopijo `map2.pgm.bak`, zato se
lahko vris ponavlja iz čiste osnovne karte.

`scan_yellow_lines.py` je polavtomatski način zajema. Med vožnjo robota po
prvi sobi bere sliko zgornje kamere in `CameraInfo`, s TF2 pridobi
transformacijo iz `top_camera_rgb_camera_optical_frame` v `map`, zaznane
rumene piksle projicira na talno ravnino in vodi mrežo zadetkov v
koordinatah karte. Celica se šteje kot potrjena po več zadetkih
(`HIT_THRESHOLD=3`). Ob prekinitvi programa se potrjene celice zapišejo v
`map2.pgm`. Skripta `scan_yellow_lines.sh` za ta postopek nastavi roko,
zažene skener in ob izhodu poskrbi za čiščenje pomožnih procesov.

V končni izvedbi je za sprotno varnost pomemben `yellow_line_avoider`,
ker ne potrebuje vnaprej popolno označene karte in se odzove na trenutno
sliko pred robotom. Vris v karto pa je uporaben kot dodatna podpora pri
testiranju, vizualizaciji ali pri poskusih, kjer želimo rumene linije
obravnavati kot zasedena območja že na ravni zemljevida.

### 3.9 Inšpektor ploščic

#### 3.9.1 Zaznavanje in pomnjenje lokacije delovne postaje

Že med patruliranjem (preden robot prejme kakršnokoli navodilo za
inšpekcijo) sistem sproti zaznava in si zapomni lokacije delovnih postaj.
Ta proces temelji na barvno označenih linijah na tleh ob vsaki delovni
postaji (rdeča ali zelena).

**LineLocalizator** (`line_localizator.py`) je prvo vozlišče v verigi.
Naroči se na OAK-D kamero: RGB sliko (`/oakd/rgb/preview/image_raw`),
globinsko sliko (`/oakd/rgb/preview/depth`) in oblak točk
(`/oakd/rgb/preview/depth/points`). Aktivno deluje le, kadar je robot v
stanjih patruliranja (ne v stanjih sledenja črti):

1.  HSV segmentacija se izvede za štiri barvne razrede (rdeča, zelena,
    modra, rumena), pri čemer rdeča uporablja dve ločeni maski za spodnji
    in zgornji del HSV obroča (0--10° in 170--180°). Zgornja polovica
    slike se ignorira, saj tam ni talnih oznak.
2.  Morfološko zapiranje z eliptičnim jedrom 5×5 px zapolni luknje v
    maskah.
3.  Konture se filtrirajo po minimalni površini (80 px) in minimalnem
    razmerju stranic (3:1), s čimer se izločijo drobni ali okrogli
    segmenti (npr. delci sodov).
4.  Vsaka kontura se dodatno razdeli na manjše linearne segmente s
    **skeletonizacijo** -- morfološko stanjšanjem maske do enopikslovnega
    skeleta, na katerem se z `HoughLinesP` poiščejo ravne črte.
5.  Za vsak segment se iz oblaka točk vzamejo 3D točke znotraj maske
    segmenta (filtrirane na razdaljo < 7 m in min. 15 veljavnih točk),
    nato se s PCA (analizo glavnih komponent) izračuna centroid in smerni
    vektor segmenta.
6.  **Filter ravnosti**: segment je veljaven le, če je standardni odklon
    vzdolž najmanjše lastne osi manjši od praga (`max_flat_std` = 0,03
    m), kar zagotavlja, da gre res za talno oznako in ne navpičen objekt.
7.  Veljavni segmenti se objavijo kot `Marker.LINE_STRIP` na temo
    `/line_markers` z ustreznim barvnim atributom (`color.r` / `color.g`).

**WorkstationRecorder** (`workstation_recorder.py`) se naroči na
`/line_markers`. Ob prejemu rdečega ali zelenega markerja:

1.  S TF2 transformira obe krajišči segmenta v `map` koordinatni sistem.
2.  **Filtrira kratke segmente**: linija mora biti dolga vsaj 1,5 m, s
    čimer se izločijo markerji sodov (ki so dolgi ~0,2 m).
3.  Izračuna **položaj za približevanje** (approach point): najprej
    izračuna centroid linije, nato od njega odmakne točko v smeri proti
    robotu za razdaljo `stop_distance` (privzeto 1 m). Yaw se izračuna
    kot smer od linije proti robotu (`atan2(-dy, -dx)`).
4.  Kandidate zbira, dokler ne doseže praga **CONFIRM_COUNT = 10**
    (deset potrditev), kar zagotavlja robustnost proti šumnim zaznavam.
5.  Po doseženem pragu izračuna **mediano** vseh kandidatov (centroidov)
    za x, y in yaw ter lokacijo **zaklene** (`red_locked` / `green_locked`).
6.  Zaklenjena lokacija se objavi kot `MarkerArray` z dvema markerjema:
    `CYLINDER` (barvni valj kot vizualna oznaka) in `TEXT_VIEW_FACING`
    (napis "WS") na temo `/workstation_markers`.

Ta tema je ključna, saj jo posluša **orkestrator**, ki bo kasneje
vedel, kam poslati robota (glej 3.9.2). V načinu `--mode toYAML` lahko
WorkstationRecorder lokacije tudi trajno shrani v datoteko YAML za
kasnejšo uporabo.

---

#### 3.9.2 Orkestracija inspekcijskega toka

**Orchestrator** (`orchestrator.py`) je centralno vozlišče, ki koordinira
celotno izvajanje inspekcije. Njegova vloga v inspekcijskem toku:

1.  **Pomnjenje waypointov**: ob prejemu `MarkerArray` na
    `/workstation_markers` (glej 3.9.1) iz CYLINDER markerjev izlušči
    barvo (rdečo ali zeleno) in si shrani (x, y, yaw) pod ustreznim
    ključem v slovar `_waypoints`.

2.  **Sprožitev prek QR kode**: ko robot prebere QR kodo z navodilom
    oblike `"defects red"` ali `"defects green"`, orkestrator to barvo
    doda v množico `_pending_defects`. Inspekcija se ne sproži takoj --
    čaka se na izpolnitev **vseh treh pogojev**:
    - patrol je končan (`/patrol_finished = True`),
    - robot je v stanju `"WORKSTATION"` (objavljeno na `/robot_state`),
    - waypoint za to barvo je že znan (tj. WorkstationRecorder ga je že
      zaklenil).

3.  **Zagon inspectorja**: ko so vsi trije pogoji izpolnjeni in ni
    drugega inspectorja v teku, orkestrator:
    - Objavi waypoint na `/orchestrator_out` (sporočilo `PoseStamped` z
      `header.frame_id` nastavljenim na barvo postaje).
    - Zažene `station_inspector.py` kot **ločen proces** (subprocess) s
      parametri `workstation:=<color>`, `use_yaml:=False`,
      `use_orchestrator:=True`. Proces se zažene s
      `preexec_fn` za ignoriranje SIGINT (da se ne ubije ob prekinitvi
      orkestratorja).

4.  **Spremljanje in zaključek**: orkestrator v svojem časovniku (0,1 s)
    spremlja izhodno kodo inspector procesa (`_inspector_proc.poll()`).
    Ko se inspector konča:
    - Objavi `Empty` sporočilo na `/workstation_done`, kar signalizira
      preostalemu sistemu, da je inspekcija končana in se lahko nadaljuje
      z naslednjo nalogo.
    - Če ima shranjeno prejšnjo pozo robota (`_saved_pose`), se robot
      vrne na to lokacijo prek NavigateToPose. To velja za primer, ko
      inspekcija poteka med patruliranjem (ne po patrolu).

---

#### 3.9.3 Station Inspector: celoten inspekcijski tok

**StationInspector** (`station_inspector.py`) je glavno vozlišče, ki
izvaja celotno sekvenco inspekcije. Njegovo delovanje je modelirano kot
končni avtomat s stanji: `INSPECTOR_INACTIVE`, `NAV_TO_WS`,
`FINE_POSITION` (z več podfazami). Vsako stanje ima natančno določene
pogoje za prehod v naslednje stanje.

##### Inicializacija in pridobitev ciljne lokacije

V stanju **INSPECTOR_INACTIVE** inspector potrebuje ciljno lokacijo
delovne postaje. Podprta sta dva načina:

- **YAML način** (`use_yaml=True`): naloži pristopno točko iz datoteke
  `test_workstation_locations.yaml`, ki vsebuje vnaprej posnete (x, y,
  yaw) koordinate. Če datoteka obstaja in vsebuje ključ za izbrano barvo,
  se takoj shrani v `self.approach_pose` in preide v NAV_TO_WS.

- **Orkestratorski način** (`use_orchestrator=True`): inspector najprej
  pošlje zahtevo `"get_{color}_waypoint"` na `/orchestrator_in` in nato
  čaka na odgovor na `/orchestrator_out`. Sporočilo vsebuje `PoseStamped`
  s `header.frame_id` enakim izbrani barvi. Ko ga prejme, si shrani
  (x, y, yaw) in preide v NAV_TO_WS.

##### Navigacija do delovne postaje (NAV_TO_WS)

V tem stanju inspector:

1.  Počaka, da je Nav2 akcijski strežnik `navigate_to_pose` na voljo.
2.  Pošlje `NavigateToPose` cilj na pristopno točko -- točko, ki je 1 m
    oddaljena od linije delovne postaje v smeri proti robotu.
3.  **Pogoj za prehod v naslednjo fazo**: `nav_goal_done == True` in
    `nav_succeeded == True`.
4.  Ob uspešni navigaciji:
    - **Zažene podprocesa** `tile_detect` in `tile_classifier` -- to sta
      ločeni ROS vozlišči, ki ju inspector upravlja kot subprocessa
      (terminirata se ob `destroy_node()` inspectorja).
    - Objavi barvo postaje na `/inspector_station`.
    - Pošlje ukaz roki (`/arm_command`) `"look_at_belt_left"` -- roka
      (top kamera) se usmeri na levi del tekočega traku.
    - Preide v stanje **FINE_POSITION** s podfazo 0.

Ob neuspešni navigaciji poskusi znova -- ponastavi `nav_goal_sent` in
ostane v NAV_TO_WS.

##### Fino pozicioniranje (FINE_POSITION)

To je najkompleksnejše stanje, razdeljeno na šest podfaz (0--5). Vsaka
podfaza ima svoj pogoj za prehod.

**Podfaza 0 -- Približevanje steni (tekočemu traku):**
- Robot vozi naravnost naprej proti delovni postaji s hitrostjo 0,15 m/s,
  ki se upočasni na 0,03 m/s, ko je oddaljenost manjša od 0,45 m.
- Oddaljenost meri z LiDARjem v smeri naprej (kotni stožec ±45° okoli
  smeri naravnost naprej, tj. center pri -π/2 glede na orientacijo
  LiDARja).
- **Pogoj za prehod**: `min_dist <= 0,30 m` -- robot se je dovolj
  približal tekočemu traku. Prehod v podfazo 1.

**Podfaza 1 -- Poravnava orientacije (rotacija v smeri traku):**
- Robot se zavrti na mestu, da se poravna vzporedno s tekočim trakom.
- Ciljni yaw je odvisen od barve postaje:
  - rdeča postaja: ciljni yaw = π rad (180°, obrnjen "nazaj" proti
    prostoru),
  - zelena postaja: ciljni yaw = π/2 rad (90°).
- Trenutni yaw se bere iz TF transformacije `map → base_link` (oz.
  `base_footprint`).
- P-regulator z ojačanjem 0,5, nasičen na ±0,4 rad/s.
- **Pogoj za prehod**: `abs(yaw_diff) <= 0,05 rad`. Prehod v podfazo 2.

**Podfaza 2 -- Fina poravnava s Houghovo transformacijo:**
- Uporablja **zgornjo kamero** (top camera, nameščeno na roki), ki gleda
  na ploščice od zgoraj. Slika se zajema v sivinskih tonih (`mono8`) iz
  teme `/top_camera/rgb/preview/image_raw`.
- Algoritem za zaznavo nagiba tekočega traku:
  1. Izračuna **vertikalni diferencial** (absolutna razlika sosednjih
     vrstic) zgornje polovice slike -- robovi ploščic na tekočem traku
     tvorijo horizontalne (oz. skoraj horizontalne) linije, ki se v
     top-down pogledu kažejo kot vertikalni gradienti.
  2. Diferencial se normalizira na [0, 255] in z Gaussovim glajenjem
     (3×3) zgladi.
  3. Otsu binarizacija loči robove od ozadja.
  4. `cv2.HoughLines` poišče linije s pragom 80 glasov. Zanimajo nas
     linije blizu vertikale: `abs(theta - π/2) < 20°`.
  5. Iz prve najdene linije se izračuna **kot nagiba** (tilt) v
     stopinjah: `(theta - π/2)`.
- P-regulator z ojačanjem 0,15 (in maksimalno hitrostjo 0,1 rad/s)
  minimizira nagib.
- **Pogoj za prehod**: `abs(tilt) <= 0,5°`. Robot je natančno poravnan
  s tekočim trakom. Prehod v podfazo 3.

**Podfaza 3 -- Vzvratna vožnja do začetka skeniranja:**
- Robot vozi nazaj s hitrostjo 0,15 m/s.
- Med vožnjo preverja **dva pogoja za ustavitev** (katerikoli je
  izpolnjen prej):
  1. **Zaznava rumene črte**: OAK-D kamera spodnji levi kot (zadnjih 30
     vrstic, leva polovica slike). V tem ROI se izračunajo maske za
     rumeno, rdečo in zeleno barvo (HSV). Pogoj za rumeno črto: število
     rumenih pikslov > (rdečih + zelenih) / 2. Rumeno črto se uporablja
     kot marker začetka območja skeniranja.
  2. **Zaznava ovire zadaj**: LiDAR v zadnjem stožcu (center pri 200°
     glede na orientacijo LiDARja, polovični kot 20°). Če je razdalja ≤
     0,40 m, se robot ustavi -- to je varnostni pogoj, če rumene črte ni
     mogoče zaznati.
- **Pogoj za prehod**: ko je eden od obeh pogojev izpolnjen, se robot
  ustavi, objavi podstanje `"SCAN_TILES"` in preide v podfazo 4.

**Podfaza 4 -- Skeniranje ploščic na tekočem traku:**
- Robot vozi počasi naprej vzdolž traku s hitrostjo 0,08 m/s.
- Med vožnjo **neprekinjeno preverja barvo delovne postaje** v spodnjem
  levem kotu OAK-D kamere (zadnjih 30 vrstic, leva polovica):
  - Za rdečo postajo: HSV maske za rdečo (dva obsega: 0--10° in
    170--180°),
  - Za zeleno postajo: HSV maska za zeleno (40--80° v odtenku).
  - Pogoj za prisotnost barve: razmerje barvnih pikslov > 15 % ROI.
- Ko barve **ni več** (`no_colour == True`), je robot prišel do konca
  delovne postaje. Takrat si zabeleži čas `_end_belt_start` in **vozi
  še 5 sekund naprej**, da zajame še zadnje ploščice, ki so morda še na
  traku.
- Med skeniranjem `tile_detect` vozlišče (opisano v 3.9.4) prepoznava
  ploščice in objavlja status na temo `/tile_status`:
  - `"TILE_FOUND"`: robot se **ustavi za 2--4 sekunde** (2 s normalno,
    4 s če smo že za koncem barvne oznake -- da klasifikator dobi dovolj
    časa za zajem čiste slike). V tem času `tile_detect` zajame in
    objavi warpano sliko ploščice (glej 3.9.4), `tile_classifier` pa
    izvede inferenco. Po preteku časa robot nadaljuje vožnjo.
  - `"TILE_LEFT"`: ploščica je zapustila vidno polje, robot lahko
    nadaljuje z iskanjem naslednje.
- **Pogoja za prehod v podfazo 5 (zaključek):**
  1. Barva ni več vidna (`no_colour == True`) IN zadnja ploščica je bila
     obdelana (počakal se je 4-sekundni interval po `TILE_FOUND`) ALI
  2. Barva ni več vidna IN 5 sekund ni bila zaznana nobena ploščica.
- Ob prehodu robot ustavi motorje, premakne roko s kamero v položaj
  `"look_for_spill"` (preverjanje razlitja) in preide v podfazo 5.

**Podfaza 5 -- Umik z delovne postaje (ESCAPING_WORKSTATION):**
- **Korak 0 (rotacija)**: robot se zavrti za 130° v smeri urinega
  kazalca (CW), da se obrne proč od tekočega traku. P-regulator z
  ojačanjem 0,5, nasičen na ±0,4 rad/s. Pogoj za prehod: razlika v
  kotu ≤ 0,05 rad.
- **Korak 1 (odmik naprej)**: robot vozi naravnost naprej s hitrostjo
  0,3 m/s **4 sekunde** -- dovolj, da se popolnoma odmakne od delovnega
  območja.
- **Korak 2 (zaključek)**: robot se ustavi, objavi `"finished"` na temo
  `/inspector_finish`, nato pa se po 0,5 sekunde vozlišče **samodejno
  ugasne** (`rclpy.shutdown()`).

---

#### 3.9.4 Detekcija in ekstrakcija čistih slik ploščic (TileDetect)

**TileDetect** (`tile_detect.py`) je ločeno vozlišče, ki se zažene kot
podproces inspectorja. Njegova naloga je iz top-down slike zgornje kamere
zaznati ploščico, določiti njene natančne robove in izrezati perspektivno
korigirano (warpano) sliko, ki gre nato v klasifikator. Vozlišče deluje
vseskozi, vendar **ploščice aktivno išče le med fazo skeniranja**
(`inspector_phase == 4`).

**Svetlostni sprožilec (brightness trigger):**
- Ker ploščice na tekočem traku odsevajo več svetlobe kot temno ozadje
  traku, se prisotnost ploščice najprej zazna prek svetlosti.
- Pregleduje se majhen ROI velikosti 20×10 px v sredini leve tretjine
  slike (`gray[h/2-5:h/2+5, w/3-10:w/3+10]`).
- Če je ≥ 50 % pikslov svetlejših od praga 100, se števec `_bright_hit`
  poveča. **Po 3 zaporednih pozitivnih okvirih** se sprožilec aktivira
  (`_bright_ready = True`).
- Če svetlost pade, se števec `_bright_missed` povečuje; **po 10
  zaporednih negativnih okvirih** se sprožilec deaktivira.
- S tem se izognemo lažnim sprožitvam zaradi trenutnih odsevov.

**Zaznava ploščice (`_find_tile`):**
1.  Slika se obreže -- zgornja petina se zavrže (ozadje, ki ni del
    tekočega traku).
2.  **Otsu binarizacija** na preostanku slike loči svetlo ploščico od
    temne podlage.
3.  Če je centralni piksel (center obrezane slike) črn, se binarna maska
    invertira -- to popravi primer, ko Otsu obrne vlogi ospredja in
    ozadja.
4.  **Morfološko zapiranje** z eliptičnim jedrom 5×5 px.
5.  Iskanje kontur s filtriranjem:
    - Površina med 5 % in 50 % celotne površine obrezane slike,
    - **Razmerje stranic ≤ 2,0** (da izločimo podolgovate objekte, ki
      niso kvadratne/okrogle ploščice).
6.  Izbere se kontura z največjo površino, ki ustreza pogojem. Iz nje se
    izračuna `minAreaRect` in pripadajoči 4-točkovni okvir (`box`).

**Zaznava natančnih robnih točk (`_get_tile_quad`):**
- Namesto `minAreaRect` okvirja (ki je lahko rotiran bounding box in ne
  nujno natančno sledi robovom) se za natančno določitev 4 oglišč
  uporabi:
  1. **Največja kontura** v polni maski.
  2. **Konveksna ogrinjača** (`convexHull`) -- zapolni morebitne
     konkavnosti.
  3. **Douglas-Peucker aproksimacija** (`approxPolyDP`) z epsilon = 2 %
     obsega ogrinjače. Rezultat je natančen štirikotnik le, če
     aproksimacija vrne natanko 4 točke. V nasprotnem primeru se
     uporabi kar `minAreaRect` okvir.

**Števec ploščic in časovna logika:**
- Ko je ploščica prvič zaznana (`_tile_was_visible == False` → `True`):
  - poveča se globalni števec `_count`,
  - shrani se okvir (`_last_box`) in natančne robne točke
    (`_last_corners`),
  - nastavi se `_capture_pending = True` in zabeleži čas prve zaznave,
  - objavi se `"TILE_FOUND"` na `/tile_status`.
- Ko ploščica izgine za **5 zaporednih okvirjev**
  (`_tile_missed >= 5`), se objavi `"TILE_LEFT"`, robne točke se
  izbrišejo, stanje se vrne v "ni ploščice".

**Perspektivna korekcija in objava warpa (`_warp_tile`):**
1.  **Urejanje točk** (`_order_points`): 4 točke se uredijo po vrstnem
    redu: zgoraj-levo, zgoraj-desno, spodaj-desno, spodaj-levo. Najprej
    se razdelijo po y-koordinati (zgornji/spodnji par), nato še po
    x-koordinati.
2.  **Inset (zarez)**: vsaka točka se premakne za **6 % proti obema
    sosednjima točkama** (`src[i] += 0.06*(prev-src[i]) +
    0.06*(next-src[i])`). To zmanjša območje za 6 % z vsake strani, s
    čimer se izreže morebitni rob, ki bi vključeval ozadje.
3.  **Ciljni pravokotnik**: širina = max razdalje med zgornjima in
    spodnjima točkama, višina = max razdalje med levima in desnima
    točkama.
4.  **Homografija**: `cv2.getPerspectiveTransform(src, dst)` izračuna
    perspektivno transformacijo, `cv2.warpPerspective` projicira
    ploščico v fronto-paralelni pogled.
5.  Warpana slika (BGR) se objavi na `/tile_warped` s `header.frame_id`
    nastavljenim na `str(self._count)` -- kar omogoča identifikacijo
    zaporedne številke ploščice.

**Časovna zakasnitev zajema:**
- Po prvi zaznavi ploščice se zajem **ne sproži takoj**, ampak šele **po
  0,5 sekunde** (`time_module.time() - trigger_time >= 0.5`). Ta
  zakasnitev je ključna: v tem času se robot ustavi (inspector ga je
  ustavil ob `TILE_FOUND`), tresljaji se umirijo in slika je stabilna --
  warpana ploščica je tako ostra in nepremaknjena.
- Zajem se izvede **natanko enkrat** na ploščico (`_capture_done = True`
  prepreči ponovne zajeme).

---

#### 3.9.5 Zaključek inspekcijskega toka

Ko se inspector konča (podfaza 5, korak 2), se:
1.  Objavi `"finished"` na `/inspector_finish`, kar je signal za vozlišče
    `report`, da lahko zabeleži rezultate inspekcije.
2.  Inspector vozlišče se samodejno ugasne (`rclpy.shutdown()`).
3.  Orkestrator zazna izhod procesa (`poll() != None`), objavi prazno
    sporočilo na `/workstation_done` in s tem signalizira preostanku
    sistema, da je inspekcija te delovne postaje zaključena.
4.  Podprocesa `tile_detect` in `tile_classifier` se terminirata ob
    `destroy_node()` inspectorja.

V primeru, da sta v vrsti dve inspekciji (npr. po VREDNE aktivnosti),
orkestrator po končani prvi samodejno sproži drugo (če so izpolnjeni vsi
trije pogoji iz 3.9.2).

------------------------------------------------------------------------

## 4. Rezultati

### 4.1 Simulacija (Task 2)

Na tekmovanju v simulaciji je robot uspešno: - zaznal in prepoznal
**\[XX\] od \[XX\]** obrazov, - zaznal **\[XX\] od \[XX\]** obročev s
pravilno barvo, - zaznal **\[XX\] od \[XX\]** sodov s pravilno barvo in
orientacijo, - sledil modri črti do cilja: **\[da/ne\]**, - zaznal
poškodbe na **\[XX\] od \[XX\]** ploščicah.

Skupno število točk: **\[XX\]**

*\[TUKAJ DODAJ SLIKO ROBOTA V SIMULACIJI ALI RVIZ SCREENSHOT\]*

### 4.2 Pravi robot (Task 1R)

Task 1R je bila implementacija prve naloge (Task 1) na fizičnem robotu in je zahteval zgolj zaznavanje obrazov in zaznavanje ter lokalizacijo obročev.

**Kamera.** Pravi robot namesto OAK-D kamere uporablja kamero Gemini RGBD. Vsi moduli so bili posodobljeni za teme `/gemini/color/image_raw/compressed`, `/gemini/depth/image_raw` in `/gemini/depth/points`. Ker smo imeli veliko problemov z zakasnjenostjo kamere smo morali narediti optimizacije. Stisnjene (compressed) teme so zmanjšale obremenitev WiFi za približno 15-kratnik. Lokalizacija objektov v 3D prostoru ne temelji na oblaku točk, temveč na back-projekciji prek intrinzičnih parametrov kamere (fx, fy, cx, cy), ki jih vozlišče samodejno prebere iz teme `/gemini/color/camera_info`.

**Zaznavanje obrazov.** Namesto Haar detektorja je bil na pravem robotu uporabljen specializiran model YOLOv8n (`yolov8n-face-lindevs.pt`), ki teče na GPU. Ker je prvotni detektor zaznaval osebe za ograjo (zunaj dovoljene cone), je bilo zaznavanje omejeno s parametroma `face_crop_top=0.30` in `face_crop_bottom=0.70` — sprejemajo se le obrazi v srednji tretjini slike. Dodana je bila preveritev uniformnosti globine znotraj bounding boxa (`FACE_DEPTH_STD_MAX = 0.20 m`): ker so bili obrazi nalepljene fotografije na ograji, je nizek standardni odklon globine potrdil ravno površino; visok odklon je detekcijo zavrnil. Zaznani obrazi se lokalizirajo s povprečno globino zaplate (20×20 px) in pinhole-projekcijo v 3D.

**Zaznavanje obročev.** Pri pravem robotu smo imeli kar nekaj težav z zaznavanjem obročev z implementacijo, ki je bila uporabljena na simulaciji. Zato smo natrenirali namenski model YOLOv8n za detekcijo obročev (`detect_rings_yolo.py`, model `~/ring_yolo/ring_det/weights/best.pt`). Učni nabor je bil generiran sintetično z orodjem `generate_ring_data.py`, ki je ustvarilo 3000 učnih in 600 validacijskih slik barvnih obročev (rdeča, zelena, modra, črna) na raznovrstnih ozadjih, skupaj z negativnimi primeri (zapolnjeni diski, sence, krožne oznake). Model je bil natreniran na 100 epohah z zgodnjim ustavljanjem (patience=20), velikost slike 416×416, na GPU.

Zadetki so bili potrjeni z dvema zaporednima okvirjema (`yolo_confirm_frames=2`) ter z globinskim testom votlosti (sredina obroča mora biti dlje kot rob). Kot varnostna mreža je bil vgrajen HSV-ellipse fallback detektor (`enable_colour_fallback=true`), ki je prevzel zaznavanje kadar YOLO ni zaznal očitnih barvnih obročev; ta zahteva 4 zaporedne potrditve (`colour_confirm_frames=4`).

Video delujočega robota si lahko ogledate na naslednji povezavi: [video](https://drive.google.com/drive/u/2/folders/1jx3x7DrQdfkEWfhQyGNvGYe20SZP5GIW)

------------------------------------------------------------------------

## 5. Razdelitev dela

**\[IME ČLANA 1\]** (\~\[XX\]% dela): - *\[dopolni\]*

**\[IME ČLANA 2\]** (\~\[XX\]% dela): - *\[dopolni\]*

**Lara Mehle** (~[XX]% dela):

- Zaznavanje in prepoznavanje obrazov: implementacija vozlišč `face_recognizer` in `face_localizator`, vključno z integracijo knjižnice face_recognition, Caffe modela za zaznavanje spola ter clustranjem zaznav v map frame.
- Zaznavanje in lokalizacija sodov: implementacija vozlišč `cylinder_localizator` in `barrel_inspector`, vključno z barvno klasifikacijo po HSV, določanjem prostorske orientacije.
- Zaznavanje poškodb ploščic: razvoj in treniranje U-Net modela (skripta `unet_train.py` in `unet_evaluate.py`) ter integracija v vozlišče `tile_classifier` za delovanje v realnem času v ROS 2.
- Izhodišče implementacije zaznavanja obročev: osnovna arhitektura rešitve, ki je bila kasneje nadgrajena s strani ekipe.
- Implementacija sistema na pravem robotu (Task 1R): prilagoditev zaznavanja obrazov in obročev za fizični hardware, kalibracija ter testiranje na realnem robotu.

------------------------------------------------------------------------

## 6. Zaključek

Razvili smo inteligentni robotski sistem, ki uspešno integrira
zaznavanje obrazov, obročev, sodov in poškodb ploščic v celovit
avtonomni sistem. Ključna arhitekturna odločitev je bila grozdevanje
vseh zaznav v globalnem koordinatnem sistemu, kar je zagotovilo
robustnost kljub šumnim senzoričnim vhodom in nenatančnostim
lokalizacije.

**Programske težave:**

- Zaznavanje obročev nam je povzročalo težave pri prvih dveh taskih. Prva implementacija v simulaciji je imela na določenih mapah veliko težav z zaznavanjem napačno pozitivnih obročev. Te probleme smo kasneje rešili z novim pristopom. Na pravem robotu (Task 1R) pa so bile težave še večje: različna osvetlitev, drugačna kamera (Gemini) in ozadje izven poligona so povzročali veliko lažnih zadetkov. Po večih preizkusih spreminjanja raznih pragov smo nazadnje pristopili k treniranju namenskega YOLO modela na sintetično generiranem učnem naboru, kar je bistveno izboljšalo robustnost.
- Pri Task 2 smo imeli precej težav z zaznavanjem sodov: HSV segmentacija je bila občutljiva na osvetlitev, podobne barve okolja so povzročale lažne zadetke, določanje orientacije (pokončen/ležeč) pa je zahtevalo večkratno prilagajanje pragov.
- Zaznavanje razlitja je bilo posebej zahtevno, saj je moral robot do vsakega ležečega soda posebej navigirati in se postaviti na ustrezno stran, da je kamera pokrivala tla ob sodu. To je pogosto povzročalo težave pri sodih, postavljenih ob ovirah ali v kotih prostora, kjer navigacijski sklad ni mogel najti ustrezne pristopne točke oziroma se robot ni mogel dovolj približati za zanesljivo detekcijo.

**Strojne težave:**

- Simulacijsko okolje in celoten ROS 2 sklad zahtevata zmogljiv zelo računalnik. Razvoj je bil zato sploh v kasnejših fazah razvoja večinoma omejen na laboratorijske računalnike, ki pa jih je bilo na voljo premalo glede na število skupin, kar je oteževalo vzporedno delo in testiranje.
- Posebej zahtevna je bila implementacija na pravem robotu. Roboti so bili pogosto zasedeni, se niso polnili, ali pa sploh niso delovali pravilno (Lidar se ni prižgal ipd.). Vsakodnevne težave s strojno opremo so bistveno upočasnile razvoj in testiranje IRL rešitve ter zmanjšale čas, ki smo ga imeli na voljo za kalibracijo in odpravljanje napak.



------------------------------------------------------------------------

## 7. Aneks

*\[Priložite generirano inšpekcijsko poročilo.\]*

