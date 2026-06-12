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
   se zaženejo Nav2, AMCL in vozlišča paketa `task1`. Robot začne
   pošiljati cilje šele, ko ima veljavno pozo iz `/amcl_pose`, saj so vsi
   cilji in zaznani objekti obravnavani v koordinatnem sistemu `map`.

2. **Patrulja po prvi sobi.** Osnovno raziskovanje prve sobe poteka po
   vnaprej določenih točkah iz `config/waypoints.yaml`. Vsaka točka
   vsebuje položaj `x`, `y`, orientacijo `yaw` in po potrebi premor
   `pause`. Nekatere točke imajo enak položaj in različne orientacije, zato
   robot na pomembnih mestih izvede pregled okolice v več smereh. Tako se
   prostor pokrije ponovljivo, zaznavni algoritmi pa imajo dovolj časa za
   zaznavanje oseb, obročev in cilindrov.

3. **Navigacija do obrazov.** Ko je potrjena lokacija obraza, se patrulja
   začasno ustavi. `behavior_manager` izračuna pristopno točko pred osebo
   in pošlje začasni cilj prek `NavigateToPose`. Po prihodu robot izvede
   interakcijo, cilj označi kot obdelan in nato nadaljuje patruljo z
   naslednjo točko.

4. **Obroči med patruljo.** Obroči se zaznavajo in lokalizirajo med
   patruljiranjem. Ker za obroče ni potrebna posebna navigacijska poza,
   robot praviloma ne prekine poti, temveč lokacijo in barvo obroča
   zabeleži med nadaljevanjem patrulje.

5. **Cilindri med patruljo.** Pokončni cilindri se ob zaznavi samo
   zabeležijo v globalni karti. Če je zaznan ležeči cilinder, lahko
   `behavior_manager` začasno ustavi patruljo in pošlje robota na varno
   pristopno točko ob cilindru. Po obdelavi cilindra se navigacija vrne v
   običajno patrolno zaporedje.

6. **Varovanje pred rumeno črto v prvi sobi.** Med navigacijo po prvi sobi
   se uporablja ločeno varovalo za rumeno črto. To ne zamenja Nav2
   planiranja poti, ampak deluje kot varnostni sloj: kamera spremlja
   rumeno oznako v bližini robota, ob nevarnem približanju pa robot ustavi
   oziroma se umakne nazaj. Na ta način se ohrani osnovna waypoint
   navigacija, hkrati pa robot ne nadaljuje čez prepovedano območje.

7. **Prehod do delovne postaje.** Ko robot iz QR navodila ali stanja
   naloge izve, da mora obiskati določeno delovno postajo, se po zaključku
   patrulje pošlje Nav2 cilj do ustrezne pristopne poze. Ta faza opisuje
   samo prihod do delovne postaje; natančna poravnava in pregled delovne
   celice sta obravnavana v ločenem poglavju.

8. **Prehod v drugo sobo in sledenje modri črti.** Po zaključku prvega
   dela naloge robot preide na navigacijo v drugi sobi. Tam se ciljno
   premikanje po karti zamenja z vizualnim vodenjem po modri črti, ki ga
   izvaja `blue_line_explorer`. Vozlišče se vključi prek
   `/blue_line_enabled`, išče modro črto, ji sledi in pri razcepih izbere
   ustrezno smer. Če črto izgubi ali naleti na slepo ulico, preide v
   obnovitveno fazo iskanja oziroma obračanja, dokler ponovno ne najde
   poti.

Trenutna faza robota se objavlja tudi kot stanje na temi `/robot_state`.
To stanje uporabljajo posamezna vozlišča, da vedo, ali smejo poseči v
gibanje robota. V stanju `IDLE` robot miruje in čaka na začetek naloge.
V stanju `PATROL` je aktivna navigacija po waypointih. Ob pristopu k
zaznanemu obrazu preide v `APPROACH_FACE`, med interakcijo pa v
`INTERACT_FACE`. Pri cilindrih se za ležeče cilindre uporablja analogni
prehod `APPROACH_BARREL` oziroma `INTERACT_BARREL`, medtem ko se pokončni
cilindri samo zabeležijo. Pri prehodu proti delovni postaji se uporabi
stanje `APPROACH_WORKSTATION`, vendar se nadaljnje stanje delovne celice
obravnava v ločenem poglavju. Po koncu prvega dela se robot premakne v
zaključno navigacijsko fazo `FINISHING_ROUNDS`, nato pa v `FOLLOW_BLUE_LINE`,
kjer nadzor prevzame sledenje modri črti. Varovalo rumene črte ima pri
tem ločeno interno stanje `CLEAR` oziroma `BACKING`, saj deluje kot
varnostni sloj nad običajnimi hitrostnimi ukazi.

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
-- tristan

------------------------------------------------------------------------

## 3. Implementacija in integracija

### 3.1 Pregled arhitekture

Sistem je implementiran kot sklop ROS 2 vozlišč v paketu `task1`.
Vozlišča komunicirajo asinhrono prek tematik (topics), akcij (actions)
in servisov (services). Spodnja tabela povzema ključna vozlišča.

  -----------------------------------------------------------------------
  Vozlišče                            Opis
  ----------------------------------- -----------------------------------
  `waypoint_navigator`                Obhodnja predefiniranih točk prek
                                      Nav2 `NavigateToPose` akcije

  `behavior_manager`                  Centralni nadzornik stanj (IDLE →
                                      PATROLLING → APPROACHING →
                                      INTERACTING)

  `face_recognizer`                   Zaznavanje in prepoznavanje obrazov
                                      z dlib + Caffe gender modelom

  `face_localizator`                  Grozdevanje in potrditev zaznav
                                      obrazov v map frame

  `detect_rings_v2`                   Zaznavanje obročev s Houghevo
                                      transformacijo na disparitetni
                                      sliki

  `ring_localizator`                  Grozdevanje in potrditev zaznav
                                      obročev

  `cylinder_localizator`              Grozdevanje zaznav sodov, določanje
                                      barve in orientacije

  `barrel_inspector`                  Inšpekcija sodov, preverjanje
                                      razlitja

  `line_localizator`                  Zaznavanje barvnih linij z HSV
                                      segmentacijo

  `blue_line_explorer`                Sledenje modri črti po zaključku
                                      patruliranja

  `tile_classifier`                   Zaznavanje poškodb ploščic z U-Net
                                      modelom

  `station_inspector`                 Nadzor inšpekcijskega procesa
                                      delovnih postaj

  `orchestrator`                      Visokonivojska koordinacija
                                      celotnega sistema
  -----------------------------------------------------------------------

### 3.2 Zaznavanje obrazov

`FaceRecognizer` se naroči na temo
`/``oakd``/``rgb``/``preview``/``image_raw`. Detekcija poteka pri 2 Hz
(omejitev CPE). Za vsak zaznan obraz:

1.  `face_recognition.face_locations``()` določi položaje obrazov v
    sliki (HOG detektor),
2.  `face_recognition.face_encodings``()` izračuna 128-dimenzionalni
    enkoder,
3.  Caffe gender model klasificira spol iz izrezanega območja obraza,
4.  Evklidska razdalja do referenčnih enkoderjev znanih oseb določi
    identiteto,
5.  Rezultat (ime, vloga, spol, zaupanje) se objavi na
    `/``recognized_person`.

`FaceLocalizator` grupira zaznave v oblake (cluster radius 0,6 m) in s
TF2 pretvori položaj v map frame. Obraz je potrjen po ≥5 zaznav.

### 3.3 Zaznavanje sodov

`CylinderLocalizator` sprejema segmentacije iz
`/``detected_cylinder_locations` (že v map frame). Za vsako zaznavo:

1.  Preveri razdaljo do vseh obstoječih grozdov,
2.  Doda točko obstoječemu grozdu ali ustvari novega,
3.  Po ≥10 zaznav grozd potrdi in objavi vizualizacijski marker v RViz2,
4.  **Barvna klasifikacija**: mediana RGB vrednosti pikslov → pretvorba
    v HSV → klasifikacija v enega od 8 barvnih razredov,
5.  **Orientacija**: glasovanje po analizi oblike točkovnega oblaka
    (pokončen/ležeč),
6.  `BarrelInspector` sproži `/``spill_check` servis po potrditvi.

### 3.4 Zaznavanje poškodb ploščic

`TileClassifier` se naroči na sliko armne kamere. Za vsako sliko:

1.  CLAHE predprocesiranje (clip_limit=4, tile_grid=8×8),
2.  Skaliranje na 512×512 in normalizacija z ImageNet parametri
    (μ=\[0.485, 0.456, 0.406\], σ=\[0.229, 0.224, 0.225\]),
3.  U-Net inferenca (segmentation_models_pytorch, ResNet34 backbone),
4.  Sigmoidna aktivacija + morfološka poobdelava,
5.  Poškodba zaznana, če razmerje poškodovanih pikselov \> 0,2 %;
    rezultat objavi na `/``tile_defect`.

### 3.5 Integracija v ROS 2

Vse komponente se zaženejo z `task1.launch.py`. Nav2 sklad se zažene
ločeno z `localization.launch.py`. Vozlišča komunicirajo prek: -
**Tematik (topics)**: zaznave, slike, markerji, stanja robota, - **Akcij
(actions)**: navigacija (`NavigateToPose`), - **Servisov (services)**:
inšpekcija razlitja (`/``spill_check`).

Za vizualizacijo in razhroščevanje smo v RViz2 prikazali vse zaznavne
markerje in stanje sistema.

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

Na fizičnem robotu TurtleBot4 je sistem uspešno demonstriral: -
zaznavanje in prepoznavanje obrazov v realnih svetlobnih pogojih, -
zaznavanje sodov z OAK-D kamero in klasifikacijo barve, - sledenje modri
črti s kamero in LiDARjem, - zaznavanje poškodb ploščic z armno kamero.

*\[TUKAJ DODAJ SLIKO PRAVEGA ROBOTA PRI DELU\]*

------------------------------------------------------------------------

## 5. Razdelitev dela

**\[IME ČLANA 1\]** (\~\[XX\]% dela): - *\[dopolni\]*

**\[IME ČLANA 2\]** (\~\[XX\]% dela): - *\[dopolni\]*

**Lara Mehle** (\~\[XX\]% dela): - Implementacija zaznavanja in
prepoznavanja obrazov: vozlišči `face_recognizer` in `face_localizator`,
vključno z integracijo knjižnice face_recognition, Caffe gender modela
in grozdevanjem v map frame. - Implementacija zaznavanja in lokalizacije
sodov/valjev: vozlišči `cylinder_localizator` in `barrel_inspector`,
vključno z barvno klasifikacijo po HSV, določanjem orientacije in
zaznavanjem razlitja. - Razvoj in treniranje U-Net modela za zaznavanje
poškodb ploščic: skripta `unet_train.py` in `unet_evaluate.py`,
integracija v vozlišče `tile_classifier` za realnočasno delovanje v ROS
2. - Izhodišče implementacije zaznavanja obročev (osnovna arhitektura
rešitve, ki je bila kasneje nadgrajena s strani ekipe). - Implementacija
sistema na pravem robotu (Task 1R): prilagoditev zaznavanja obrazov,
sodov in poškodb ploščic za fizični hardware, kalibracija in testiranje
na realnem robotu.

------------------------------------------------------------------------

## 6. Zaključek

Razvili smo inteligentni robotski sistem, ki uspešno integrira
zaznavanje obrazov, obročev, sodov in poškodb ploščic v celovit
avtonomni sistem. Ključna arhitekturna odločitev je bila grozdevanje
vseh zaznav v globalnem koordinatnem sistemu, kar je zagotovilo
robustnost kljub šumnim senzoričnim vhodom in nenatančnostim
lokalizacije.

**Programske težave:** - Zaznavanje obročev je bilo najzahtevnejša
komponenta: Hougheva transformacija je bila občutljiva na svetlobne
pogoje in ozadje v simulatorju. - Usklajevanje zaznavanja z navigacijo
je povzročalo občasne tekmovalne pogoje (race conditions) pri hitrih
zaporednih detekcijah. - *\[DOPOLNI Z OSTALIMI TEŽAVAMI\]*

**Strojne težave:** - Omejena računska moč TurtleBot4 je upočasnila
inferenco U-Net modela; na pravem robotu smo zmanjšali frekvenco
zaznavanja. - OAK-D kamera ima omejeno vidno polje pri majhnih
razdaljah, kar je oteževalo zaznavanje bližnjih objektov. - *\[DOPOLNI Z
OSTALIMI TEŽAVAMI\]*

Sistem je bil uspešno demonstriran na tekmovanju in dosegel **\[XX\]
točk**.

------------------------------------------------------------------------

## 7. Aneks

*\[Priložite generirano inšpekcijsko poročilo.\]*

------------------------------------------------------------------------

## Viri

\[1\] Macenski, S. et al., "Navigation2: The Next Generation Navigation
System for ROS", *IEEE IROS 2020*.\
\[2\] Geitgey, A., "face_recognition",
https://github.com/ageitgey/face_recognition\
\[3\] Levi, G. and Hassner, T., "Age and Gender Classification Using
Convolutional Neural Networks", *CVPR 2015*.\
\[4\] Ronneberger, O., Fischer, P., and Brox, T., "U-Net: Convolutional
Networks for Biomedical Image Segmentation", *MICCAI 2015*.
