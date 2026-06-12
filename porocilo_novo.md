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

### 2.10 Generiranje poročila

Generiranje inšpekcijskega poročila je osrednji zaključni proces celotne
misije, ki se sproži ob srečanju s tehničnim direktorjem (CEO) v drugi
sobi. Celoten sistem zbiranja podatkov in proženja poročila je odvisen od
**QR čitalca** -- brez njegovega delovanja nobena naloga ni aktivirana in
poročilo ne more biti generirano.

---

#### 2.10.1 QR čitalec kot centralni prožilec

QR čitalec (`qr_reader`) je vozlišče, ki teče neprekinjeno od začetka
misije in deluje v dveh načinih, ki se preklapljata glede na stanje
robota:

- **Način "face"**: aktiven, ko se robot približuje osebi ali z njo
  komunicira. V tem načinu QR čitalec bere navodila z delavčeve QR kode
  (npr. preštevanje obročev, inšpekcija sodov ali preverba anomalij na
  določeni postaji) in jih posreduje sistemu. Poleg tega ob zaznavi
  osebe prek prepoznave obraza objavi njeno identiteto, kar omogoča
  beleženje, kdo je zahteval posamezno nalogo.

- **Način "blue\_line"**: aktiven med sledenjem modri črti v drugi sobi.
  V tem načinu QR čitalec neprekinjeno pregleduje vse QR kode ob poti
  in čaka na tisto, ki vsebuje ključno besedo "report" -- to je koda
  tehničnega direktorja, ki sproži generiranje končnega poročila.

Za dekodiranje QR kod se uporablja kombinacija WeChat QR detektorja (za
majhne in poškodovane kode) in OpenCV `QRCodeDetector` z več strategijami
predprocesiranja (CLAHE izenačevanje histograma, Otsu binarizacija), kar
zagotavlja robustno branje v različnih svetlobnih pogojih.

---

#### 2.10.2 Zbiranje podatkov med misijo

**Poročilni upravljalnik** (`report_manager`) je vozlišče, ki teče
vzporedno z vsemi drugimi komponentami in skozi celotno misijo zbira
podatke iz številnih virov. Ne izvaja lastnega procesiranja, temveč deluje
kot centralno skladišče, v katerega druga vozlišča pošiljajo rezultate
svojega dela:

- **Štetje obročev**: vozlišče za lokalizacijo obročev objavlja potrjene
  lokacije obročev skupaj z njihovo barvo. Poročilni upravljalnik šteje
  obroče po barvnih razredih (rdeča, zelena, modra, rumena, oranžna) in
  si zapomni skupno število.

- **Inšpekcija sodov**: informacije prihajajo iz dveh virov:
  1. Lokalizator cilindrov sporoča barvo in orientacijo vsakega
     potrjenega soda (pokončen ali ležeč).
  2. Inšpektor sodov po opravljeni analizi razlitja za vsak sod sporoči,
     ali je bilo zaznano razlitje.
  Oba podatka se združita v evidenco z ID-jem soda, barvo, orientacijo
  in statusom razlitja.

- **Zaznavanje anomalij na ploščicah**: to je najkompleksnejši podatkovni
  tok:
  1. Inšpektor postaje sporoči, katero postajo trenutno pregleduje
     (rdečo ali zeleno).
  2. Detektor ploščic sporoči vsako novo zaznano ploščico in obenem
     objavi njeno perspektivno korigirano sliko.
  3. Klasifikator za vsako ploščico vrne rezultat ("OK" ali "DEFECT")
     ter prikaz napake v obliki toplotne karte (heatmap).
  4. Poročilni upravljalnik vse to shrani po postajah in ploščicah,
     vključno s slikami defektnih ploščic in pripadajočimi toplotnimi
     kartami.

- **Identiteta naročnika**: poleg samega štetja in inšpekcije sistem
  beleži tudi, kdo je posamezno nalogo zahteval. To poteka v dveh
  korakih: najprej QR čitalec ob približevanju obrazu sporoči začetek
  novega kroga, nato pa ob prepoznavi osebe objavi njeno ime, ki se
  pripiše vsem nalogam, zahtevanim v tem krogu.

Pomembno je, da se naloge aktivirajo **izključno prek QR kod**.
Brez prebrane QR kode z navodilom (npr. "rings", "barrels", "defects
red") poročilni upravljalnik sicer beleži podatke, vendar v končnem
poročilu za to nalogo izpiše "Task not requested". To velja za vse tri
tipe nalog.

---

#### 2.10.3 Generiranje in zgradba poročila

Ko robot v načinu sledenja modri črti prebere QR kodo s ključno besedo
"report", se sproži generiranje poročila:

1. **Generiranje PDF in Markdown**: poročilni upravljalnik iz vseh
   zbranih podatkov sestavi strukturiran dokument v dveh formatih --
   PDF (za uradno oddajo) in Markdown (za morebitno nadaljnjo obdelavo).

2. **Zgradba poročila** vključuje tri sekcije:

   - **Ring Counting**: skupno število zaznanih obročev, razčlenjeno po
     barvah, z navedbo osebe, ki je nalogo zahtevala. Če naloga ni bila
     zahtevana, se to izrecno zapiše.

   - **Barrel Inspection**: tabela vseh pregledanih sodov s stolpci ID,
     barva, orientacija (pokončen/ležeč) in zaznano razlitje (da/ne/?).
     Za sede z razlitjem se v poročilo vključijo tudi slikovni dokazi
     razlitja.

   - **Anomaly Detection**: za vsako pregledano delovno postajo posebej
     (rdečo/zeleno) se izpišejo: skupno število pregledanih ploščic,
     število brezhibnih in število defektnih. Sledi tabela s statusom
     vsake ploščice. Za defektne ploščice se v poročilo vstavijo pari
     slik: originalna warpana slika ploščice in pripadajoča toplotna
     karta (heatmap), ki prikazuje lokacijo in intenziteto poškodbe.
     Tudi tu se zabeleži identiteta naročnika.

3. **Avtomatsko ugašanje sistema**: 3 sekunde po proženju poročila QR
   čitalec samodejno ugasne vsa delovna vozlišča (zaznavanje obrazov,
   obročev, sodov, navigacijo, sledenje črti itd.). Edino vozlišče, ki
   ostane aktivno, je prav poročilni upravljalnik -- s čimer se
   zagotovi, da ima dovolj časa dokončati generiranje PDF dokumenta,
   preden se celoten sistem ustavi.

Izhodni datoteki se shranita v imenik `~/RINS_Project/reports/pdf/` z
zaporednim številčenjem (`report00.pdf`, `report01.md` itd.), slike
defektnih ploščic in razlitij pa v podimenike `img/defects/` in
`img/barrels/`.

------------------------------------------------------------------------

## 3. Implementacija in integracija

### 3.0 Arhitektura in ROS2

#### 3.0.1 Pregled arhitekture

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

#### 3.0.2 Integracija v ROS 2

Vse komponente se zaženejo z `task1.launch.py`. Nav2 sklad se zažene
ločeno z `localization.launch.py`. Vozlišča komunicirajo prek: -
**Tematik (topics)**: zaznave, slike, markerji, stanja robota, - **Akcij
(actions)**: navigacija (`NavigateToPose`), - **Servisov (services)**:
inšpekcija razlitja (`/``spill_check`).

Za vizualizacijo in razhroščevanje smo v RViz2 prikazali vse zaznavne
markerje in stanje sistema.

### 3.3 Zaznavanje obrazov

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

### 3.4 Zaznavanje sodov

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

### 3.5 Zaznavanje poškodb ploščic

`TileClassifier` se naroči na sliko armne kamere. Za vsako sliko:

1.  CLAHE predprocesiranje (clip_limit=4, tile_grid=8×8),
2.  Skaliranje na 512×512 in normalizacija z ImageNet parametri
    (μ=\[0.485, 0.456, 0.406\], σ=\[0.229, 0.224, 0.225\]),
3.  U-Net inferenca (segmentation_models_pytorch, ResNet34 backbone),
4.  Sigmoidna aktivacija + morfološka poobdelava,
5.  Poškodba zaznana, če razmerje poškodovanih pikselov \> 0,2 %;
    rezultat objavi na `/``tile_defect`.


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

### 3.10 Generiranje poročila

#### 3.10.1 QR čitalec (`qr_reader.py`)

QR čitalec je vstopna točka za celoten poročilni sistem -- brez njega
nobena naloga ni aktivirana. Vozlišče se naroči na OAK-D kamero
(`/oakd/rgb/preview/image_raw`), stanje robota (`/robot_state`) in
prepoznane osebe (`/recognized_person`). Objavlja na temi `/qr` (navodila
za naloge) in `/report_commands` (ukaz za generiranje poročila).

**Načini delovanja** se preklapljajo avtomatsko glede na stanje robota:
- `"face"` -- ko je robot v stanjih `APPROACH_FACE` ali `INTERACT_FACE`,
- `"blue_line"` -- ko je v stanjih sledenja modri črti (`FOLLOW_BLUE_LINE`,
  `LINE_FOLLOWING`, `BLUE_LINE_SEARCH`, `BLUE_LINE_FOLLOW`,
  `BLUE_LINE_DEAD_END`),
- `"off"` -- v vseh ostalih stanjih (QR čitalec ne procesira slik).

**Dekodiranje QR kod** uporablja kaskado detektorjev z naraščajočo
robustnostjo (in padajočo hitrostjo):
1. WeChat QR detektor (`cv2.wechat_qrcode_WeChatQRCode`) -- primeren za
   majhne, poškodovane ali slabo osvetljene kode.
2. OpenCV `QRCodeDetector.detectAndDecodeMulti` na originalni sivi sliki.
3. `QRCodeDetector.detectAndDecode` (posamezna koda).
4. CLAHE izenačevanje histograma (`clipLimit=3.0, tileGridSize=8x8`) +
   ponovitev korakov 2 in 3.
5. Otsu binarizacija (`cv2.THRESH_BINARY + cv2.THRESH_OTSU`) + ponovitev
   korakov 2 in 3.

**Obdelava vsebine QR kode** je odvisna od načina:
- V *face* načinu: `_check_defects()` prepozna ključne besede v
  prebranem nizu. Če vsebuje `"barrel"`, objavi `"barrels"` na `/qr`;
  če `"ring"`, objavi `"rings"`; če `"defect"` ali `"anomaly"`, objavi
  `"defects red"` ali `"defects green"` (privzeto zelena, razen če niz
  vsebuje `"red"`). Če vsebuje `"report"`, objavi `"make"` na
  `/report_commands`.

- V *blue_line* načinu: `_check_blue_line_qr()` reagira le na kode, ki
  vsebujejo `"report"` -- te sprožijo generiranje poročila prek
  `/report_commands` ("make") in nastavijo 3-sekundni odštevalnik za
  ugašanje sistema. Vse ostale QR kode v tem načinu se ignorirajo
  (beležijo se v log, vendar ne sprožijo akcij). Vsaka koda se obdela
  le enkrat (`_seen_blue_qr` množica).

**Objava identitete osebe**: v face načinu QR čitalec posluša
`/recognized_person`. Ko prejme JSON z imenom osebe, ga objavi na `/qr`
kot `"person <ime>"` -- vendar le enkrat na osebo (dokler se ne zamenja).

**Vizualizacija**: vsaka prebrana QR koda se prikaže v pojavnem oknu
("QR Code Text") z zelenim okvirjem in izpisom vsebine. Vsaka unikatna
koda se prikaže le enkrat (`_seen_popup_qr`).

**Ugašanje sistema po poročilu**: `_check_report_finish()` se kliče na
0,2 s. Ko preteče 3 s od zaznave "report" QR kode, QR čitalec z ukazom
`pkill -f <vzorec>` sistematično ugasne vsa delovna vozlišča (skupno 23
vzorcev, vključno z zaznavanjem obrazov, obročev, sodov, navigacijo,
sledenjem črti, orkestratorjem, inšpektorjem postaje ipd.). Edino
vozlišče, ki se ohrani, je `report_manager`.

---

#### 3.10.2 Poročilni upravljalnik (`report.py`)

`ReportManager` je centralno vozlišče, ki teče od začetka do konca
misije in zbira vse inšpekcijske podatke. Naročen je na 11 tem:

| Tema | Tip sporočila | Namen |
|---|---|---|
| `/detected_ring_locations` | `Marker` | Šteje potrjene obroče po barvah |
| `/detected_cylinder_locations` | `Marker` | Beleži sode (ID, barva, orientacija) |
| `/barrel_inspection_result` | `String` (JSON) | Beleži rezultat preverbe razlitja |
| `/qr` | `String` | Aktivira naloge in beleži naročnike |
| `/recognized_person` | `String` (JSON) | Pomožni log prepoznanih oseb |
| `/inspector_station` | `String` | Določa trenutno postajo ("red"/"green") |
| `/tile_status` | `String` | Zaznava nove ploščice ("TILE_FOUND") |
| `/tile_classification` | `String` | Rezultat klasifikacije ("OK" ali "DEFECT") |
| `/tile_warped` | `Image` | Shrani warpano sliko ploščice |
| `/tile_heatmap` | `Image` | Shrani toplotno karto poškodbe |
| `/report_commands` | `String` | Proži generiranje ("make") ali brisanje ("clear") |

**Beleženje naročnika naloge**: sistem sledi, katera oseba je zahtevala
posamezno nalogo, prek mehanizma "krogov":
1. Ob prejemu `"approaching face"` na `/qr` se začne nov krog
   (`_round_requester = "Unknown"`, `_round_tasks = {}`).
2. Vse naloge, aktivirane v tem krogu, se dodajo v `_round_tasks`.
3. Ko prispe `"person <ime>"`, se ime shrani kot `_round_requester` in
   pripiše vsem nalogam tega kroga.
4. Če ime ne prispe (npr. oseba ni bila prepoznana), ostane naročnik
   `"Unknown"`. Naloge za defekte imajo ločen slovar `_defect_requesters`
   (ključ je barva postaje).

**Aktivacija nalog** poteka izključno prek QR kod:
- `"rings"` na `/qr` → `self.rings_requested = True`
- `"barrels"` → `self.barrels_requested = True`
- `"defects <barva>"` → doda barvo v `self.defect_stations`
- Brez prejete aktivacije se v poročilu izpiše "Task not requested",
  razen če je vključen parameter `noqr:=True` (debug način, ki vedno
  prikaže vse sekcije).

**Zbiranje podatkov o obročih** (`_ring_callback`):
- Sprejema `Marker` z imenskim prostorom `"ring_confirmed"` ali
  `"ring_actionable"`.
- Iz `color.r/g/b` vrednosti markerja določi barvo (prek
  `_marker_to_color()`, ki primerja RGB z 8 preddefiniranimi barvami).
- Vsak marker se šteje le enkrat (preverba prek `_seen_ring_ids`).
- Števci se hranijo v `self.ring_counts[barva]`.

**Zbiranje podatkov o sodih** (`_barrel_callback`):
- Sprejema `Marker` z imenskim prostorom `"barrel_confirmed"` ali
  `"cylinder_confirmed"`.
- Iz markerja prebere barvo (enako kot pri obročih), orientacijo iz
  `marker.text` (`"horizontal"` ali `"vertical"`) in ID iz `marker.id`.
- Vsak sod se doda le enkrat v seznam `self.barrel_data` (preverba po
  `barrel_id`).

**Rezultati razlitja** (`_barrel_result_callback`):
- Prejme JSON z `barrel_id` in `leak_detected` (true/false).
- Posodobi ustrezen vnos v `self.barrel_data`.

**Zbiranje podatkov o ploščicah**:
- `_station_callback`: ob prejemu `"red"` ali `"green"` na
  `/inspector_station` nastavi `_current_station` in inicializira seznam
  ploščic za to postajo, če še ne obstaja.
- `_tile_status_callback`: ob `"TILE_FOUND"` doda nov vnos v seznam
  trenutne postaje (`tiles_per_station[station]`) z ID-jem (zaporedna
  številka), statusom `None` in praznima poljema za slike.
- `_tile_classification_callback`: prejme niz oblike `"OK:<tile_id>"` ali
  `"DEFECT:<tile_id>"`. Posodobi zadnjo ploščico v seznamu. Če je
  rezultat `"DEFECT"`, shrani warpano sliko in heatmapo na disk:
  - warpana slika: `img/defects/defect<XX>_<seq>_<station>.png`
  - heatmapa: `img/defects/heatmap<XX>_<seq>_<station>.png`
- `_tile_warped_callback` in `_tile_heatmap_callback`: shranjujeta slike
  v predpomnilnik (`_warped_cache`, `_heatmap_cache`), indeksirane po
  `header.frame_id` (ki ga `tile_detect` nastavi na zaporedno številko).
  Predpomnilnika se trimata na zadnjih 20 vnosov.

**Zakasnjeno shranjevanje slik** (`_pending_defect` mehanizem):
Ker warpana slika in heatmapa lahko prispeta v poljubnem vrstnem redu
glede na klasifikacijo, se ob DEFECT rezultatu preveri, ali sta obe
sliki že v predpomnilniku. Če ne, se shranita v `_pending_defect` z
2-sekundnim timeoutom -- `_check_defect_timeout` periodično preverja,
ali sta sliki medtem prispeli, in ju shrani, ko sta na voljo.

---

#### 3.10.3 Generiranje poročila

Generiranje se sproži s sporočilom `"make"` na `/report_commands` (pošlje
ga QR čitalec ob zaznavi "report" kode). Podprt je tudi ukaz `"clear"`,
ki izbriše vse prejšnje rezultate.

**Postopek generiranja** (`_make_report`):
1. Določi se ime poročila `report<XX>` (zaporedno številčenje, ki se
   nadaljuje od zadnjega obstoječega PDF-ja v izhodnem imeniku).
2. Ustvari se imenik `~/RINS_Project/reports/pdf/` in podimenika
   `img/barrels/` in `img/defects/`.
3. Poročilo se generira v dveh formatih: PDF (prek knjižnice `fpdf`) in
   Markdown (`.md` datoteka).

**PDF generiranje** (`_generate_pdf`):
- Uporablja razred `InspectionPDF(FPDF)` z glavo (naslov) in nogo
  (številka strani).
- Dokument vsebuje tri sekcije (vsaka na svoji strani):

1. **Ring Counting** -- prikaže se le, če je bila naloga aktivirana
   (`self.rings_requested == True` ali če so bili obroči dejansko
   zaznani). Vsebuje identiteto naročnika, skupno število obročev in
   razčlenitev po barvah.

2. **Barrel Inspection** -- vsebuje tabelo vseh sodov s stolpci Barrel
   ID, Colour, Orientation, Leak detected. Sledijo slike razlitij iz
   `img/barrels/` (datoteke z vzorcem `leak_*.jpg`), postavljene v
   mrežo 3×2 (3 slike na vrstico, vsaka širine 50 mm).

3. **Anomaly Detection** -- za vsako postajo (rdečo/zeleno) posebej:
   identiteta naročnika, statistika (skupaj/OK/DEFECT), tabela vseh
   ploščic s statusom. Za defektne ploščice se v parih (texture +
   heatmap) vstavijo slike velikosti 30 mm.

**Markdown generiranje** (`_write_markdown`):
- Vzporedno s PDF-jem se generira tudi Markdown datoteka z enako
  strukturo, ki vključuje inline slike (poti do `.png` datotek).

**Izhodni imenik**: `~/RINS_Project/reports/pdf/`. Vsak zagon ustvari
datoteki `report<XX>.pdf` in `report<XX>.md`, kjer se `<XX>` samodejno
povečuje.

------------------------------------------------------------------------

## 4. Rezultati

### 4.1 Simulacija (Task 2)

Na tekmovanju v simulaciji je robot uspešno: 
- zaznal in prepoznal **\[5\] od \[5\]** obrazov, 
- zaznal **\[4\] od \[4\]** obročev s pravilno barvo, 
- zaznal **\[8\] od \[8\]** sodov s pravilno barvo in orientacijo, pri čemer je en sod zaznal dvakrat,
- zaznal **\[0\] od \[1\]** razlitija ležečih sodov,
- uspešno sledil modri črti do cilja: **\[da\]**, 
- pravilno zaznal (odstonost) poškodbe na **\[4\] od \[4\]** ploščicah.

V poglavju 7 se nahaja pdf poročilo, ki ga je po ocenjenem obhodu zgeneriral robot.

### 4.2 Pravi robot (Task 1R)

Task 1R je bila implementacija prve naloge (Task 1) na fizičnem robotu in je zahteval zgolj zaznavanje obrazov in zaznavanje ter lokalizacijo obročev.

**Kamera.** Pravi robot namesto OAK-D kamere uporablja kamero Gemini RGBD. Vsi moduli so bili posodobljeni za teme `/gemini/color/image_raw/compressed`, `/gemini/depth/image_raw` in `/gemini/depth/points`. Ker smo imeli veliko problemov z zakasnjenostjo kamere smo morali narediti optimizacije. Stisnjene (compressed) teme so zmanjšale obremenitev WiFi za približno 15-kratnik. Lokalizacija objektov v 3D prostoru ne temelji na oblaku točk, temveč na back-projekciji prek intrinzičnih parametrov kamere (fx, fy, cx, cy), ki jih vozlišče samodejno prebere iz teme `/gemini/color/camera_info`.

**Zaznavanje obrazov.** Namesto Haar detektorja je bil na pravem robotu uporabljen specializiran model YOLOv8n (`yolov8n-face-lindevs.pt`), ki teče na GPU. Ker je prvotni detektor zaznaval osebe za ograjo (zunaj dovoljene cone), je bilo zaznavanje omejeno s parametroma `face_crop_top=0.30` in `face_crop_bottom=0.70` — sprejemajo se le obrazi v srednji tretjini slike. Dodana je bila preveritev uniformnosti globine znotraj bounding boxa (`FACE_DEPTH_STD_MAX = 0.20 m`): ker so bili obrazi nalepljene fotografije na ograji, je nizek standardni odklon globine potrdil ravno površino; visok odklon je detekcijo zavrnil. Zaznani obrazi se lokalizirajo s povprečno globino zaplate (20×20 px) in pinhole-projekcijo v 3D.

**Zaznavanje obročev.** Pri pravem robotu smo imeli kar nekaj težav z zaznavanjem obročev z implementacijo, ki je bila uporabljena na simulaciji. Zato smo natrenirali namenski model YOLOv8n za detekcijo obročev (`detect_rings_yolo.py`, model `~/ring_yolo/ring_det/weights/best.pt`). Učni nabor je bil generiran sintetično z orodjem `generate_ring_data.py`, ki je ustvarilo 3000 učnih in 600 validacijskih slik barvnih obročev (rdeča, zelena, modra, črna) na raznovrstnih ozadjih, skupaj z negativnimi primeri (zapolnjeni diski, sence, krožne oznake). Model je bil natreniran na 100 epohah z zgodnjim ustavljanjem (patience=20), velikost slike 416×416, na GPU.

Zadetki so bili potrjeni z dvema zaporednima okvirjema (`yolo_confirm_frames=2`) ter z globinskim testom votlosti (sredina obroča mora biti dlje kot rob). Kot varnostna mreža je bil vgrajen HSV-ellipse fallback detektor (`enable_colour_fallback=true`), ki je prevzel zaznavanje kadar YOLO ni zaznal očitnih barvnih obročev; ta zahteva 4 zaporedne potrditve (`colour_confirm_frames=4`).

Video delujočega robota si lahko ogledate na naslednji povezavi: [video](https://drive.google.com/drive/u/2/folders/1jx3x7DrQdfkEWfhQyGNvGYe20SZP5GIW)

------------------------------------------------------------------------

## 5. Razdelitev dela

**\[IME ČLANA 1\]** (\~ 1/3 dela): - *\[dopolni\]*

**\[Tristan Flander\]** (\~ 1/3 dela):
- Generiranje inšpekcijskega poročila: implementacija vozlišča `report_manager`, vključno z zbiranjem podatkov iz vseh zaznavnih podsistemov, sledenjem identitete naročnikov ter generiranjem PDF in Markdown poročil z vdelanimi slikami.
- Branje in parsanje QR kod: implementacija vozlišča `qr_reader`, vključno z rudarjenjem podatkov iz QR kod v dveh načinih (face in blue_line), dekodiranjem s kaskado WeChat in OpenCV detektorjev ter avtomatskim ugašanjem sistema po generiranju poročila.
- Zaznavanje obročev v simulaciji: prva implementacija s HSV barvno segmentacijo in aproksimacijo kontur z elipsami; kasneje nadgrajena s z metodo Hougheve transformacije na disparitetni sliki.
- Celoten inspekcijski tok delovnih postaj (razen klasifikatorja): implementacija vozlišč `station_inspector`, `tile_detect`, `workstation_recorder`, `line_localizator` in `orchestrator`, vključno s finim pozicioniranjem ob tekočem traku, detekcijo ploščic s perspektivno korekcijo ter orkestracijo inspekcijskih sekvenc.

**Lara Mehle** (~ 1/3 dela):

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
- Ker smo uporabljali navigacijo do izbrane točke, ki smo jo dobili na vajah, smo imeli pogosto težave z nepredvidljivostjo gibanja robota, včasih je zaradi tega v testiranjih spregledal obraze ali obroče, in smo zato morali robustificirati detektorje. Morda bi bilo celo bolje če bi napisali svoj sistem za navigacijo.

**Strojne težave:**

- Na lastnih prenosnih računalnikih smo imeli stalne težave z RViz in Gazebo okoljem, nekateri člani pa sploh niso imeli dovolj zmogljivega prenosnika. Razvoj je bil zato, sploh v kasnejših fazah razvoja,večinoma omejen na laboratorijske računalnike, ki pa jih je bilo na voljo premalo glede na število skupin, kar je oteževalo vzporedno delo in testiranje.
- Posebej zahtevna je bila implementacija na pravem robotu. Roboti so bili pogosto zasedeni, se niso polnili, ali pa sploh niso delovali pravilno (Lidar se ni prižgal ipd.). Vsakodnevne težave s strojno opremo so bistveno upočasnile razvoj in testiranje IRL rešitve ter zmanjšale čas, ki smo ga imeli na voljo za kalibracijo in odpravljanje napak.



------------------------------------------------------------------------

## 7. Aneks

*\[Priložite generirano inšpekcijsko poročilo.\]*

