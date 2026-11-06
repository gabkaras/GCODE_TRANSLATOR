# gcode2krl – slicer dla robota 6-osiowego i translator G-code → KUKA KRL

Narzędzie prowadzi od modelu do programu robota w dwóch krokach:

1. **Slicer** (własny, MIT): model STEP/STL/OBJ → ścieżka drukowania z
   warstwami płaskimi, stożkowymi (RotBot), konforemnymi do powierzchni albo
   wielokierunkowymi, z osią narzędzia I J K przy każdym punkcie.
2. **Translator**: ścieżka (ze slicera albo gotowy G-code z innego slicera /
   CAM) → program KRL dla KUKA KRC4 (KSS 8.x). Ciągi krótkich G1 są z powrotem
   rozpoznawane jako łuki (`SCIRC`/`CIRC`), reszta trafia do `SLIN`/`LIN`,
   fragmenty o zmiennej krzywiźnie i zmiennej orientacji do `SPL`; po drodze
   sprawdzany jest zasięg, limity osi i kolizje robota.

```
STEP/STL ──slicer──> ścieżka (G-code + I J K) ──parser──> ruchy ──łuki / orientacja──> SLIN / SCIRC / SPL ──writer──> PROG.src + PROG.dat
                       gotowy G-code (.nc) ───┘
```

![GUI gcode2krl](docs/gui.png)

## Instalacja

```bash
pip install -e .          # wymaga Python >= 3.10 i numpy
pip install -e .[test]    # dodatkowo pytest
pip install -e .[step]    # import STEP/IGES (OpenCASCADE)
pytest                    # 106 testów
```

## GUI

Bez instalacji: otwórz `docs/demo.html` w przeglądarce (statyczny podgląd z
gotowymi wynikami dla trzech przykładów; opcje nie przeliczają). Pełna wersja:

```bash
gcode2krl-gui                       # http://127.0.0.1:8765, otwiera przeglądarkę
gcode2krl-gui --port 9000 --no-browser
```

Lokalna aplikacja webowa bez żadnych zależności poza biblioteką standardową
Pythona i numpy (działa offline, np. na komputerze przy robocie):

* krok 1 (domyślny): wczytaj model w sekcji *Slicer*, ustaw parametry i tryb
  warstw, „Pokrój i tłumacz na KRL”; krok 2: zakładka *Gotowy G-code* przyjmuje
  plik z innego slicera / CAM (przeciągnij do okna albo wybierz przykład);
  wynik liczy się automatycznie po każdej zmianie opcji (suwak tolerancji,
  tryb, orientacja…),
* kafelki: ruchy wejściowe / KRL, kompresja, maksymalna odchyłka (zielona,
  gdy mieści się w tolerancji), czasy parsera / dopasowania / generowania,
* **podgląd 3D** na własnym rendererze WebGL: warstwy G-code wejściowy, G0,
  SLIN, SCIRC, SPL i punkty pomocnicze łuków; obrót (LPM), przesuwanie
  (PPM / Shift), zoom (kółko, pinch), widoki izo / z góry / z przodu,
  rzut ortogonalny, siatka z automatycznym krokiem, eksport PNG;
  skróty `F` dopasuj, `T` z góry, `I` izo, `Ctrl+Enter` tłumacz,
* zakładka **KRL** z podświetlaniem składni, wyborem pliku (program główny i
  podprogramy), kopiowaniem, pobraniem pojedynczego pliku lub `.zip`
  z kompletem `.src` + `.dat` (CRLF),
* zakładka **Ostrzeżenia** z komunikatami parsera (G28, G92, G41/G42,
  niezgodne promienie łuków, brak F…),
* opcje zapamiętywane w przeglądarce (`localStorage`),
* motyw ciemny (domyślny), jasny lub jak system (przełącznik w nagłówku);
  wszystkie kolory, fonty i promienie są tokenami CSS na początku
  `web/style.css`, a `web/brand.css` nakłada identyfikację Robco: paletę
  (granat `#101820`, panele `#151f29`, akcent `#00a3e0`, tekst `#f0eee9`),
  fonty Barlow i IBM Plex Mono (`web/fonts/`, licencja OFL), przyciski
  wielkimi literami z obrysem i poświatą, kafelki z niebieską krawędzią,
  chipy, teksturę siatki i logo (`web/brand/`), zdjęte z www.robco.technology.

![zakładka KRL](docs/gui_krl.png)

API JSON serwera (do własnej automatyzacji):

| metoda | ścieżka | opis |
|---|---|---|
| GET | `/api/examples`, `/api/example/<nazwa>` | przykładowe pliki |
| GET | `/api/robots` | modele robotów (wymiary, limity, prędkości) |
| POST | `/api/translate` | `{gcode, filename, options}` → statystyki, ostrzeżenia, pliki, geometria podglądu (float32 w base64), raport robota ze śladem osi |
| GET | `/api/zip/<token>` | zip z plikami ostatniego tłumaczenia |

## Slicer (model → G-code dla ekstrudera na robocie)

Własny slicer na licencji MIT, bez kodu z rodziny Slic3r/PrusaSlicer/Bambu/Orca
(AGPL). Operacje na wielokątach wykonuje Shapely (BSD-3; GEOS na LGPL przez
dynamiczne linkowanie), więc licencja projektu pozostaje MIT.

```bash
pip install -e .[slicer]
gcode2krl-slice czesc.step -o czesc.gcode --layer-height 1 --bead-width 2 --perimeters 2 --infill-density 0.2
gcode2krl czesc.gcode -o out/ --extruder-out 7 --robot KR90R2700PRO --base-frame 1400 0 500 0 0 0
```

Co robi:

* **cięcie siatki** (STL/OBJ/STEP/IGES) płaszczyznami warstw na zamknięte
  kontury; segmenty łączone z tolerancją, więc drobne szczeliny w siatce nie
  psują konturu, a otwarte łańcuchy są raportowane,
* **wielokąty warstwy** z zagnieżdżeniem parzystym (otwory), usuwanie cech
  cieńszych niż `min_feature`,
* **obrysy** (`perimeters`, od wewnątrz na zewnątrz), **wypełnienie**: linie
  (kąt zmienny co warstwę), siatka, koncentryczne; **warstwy pełne** górne i
  dolne wykrywane z pokrycia przez sąsiednie warstwy; nakładka wypełnienia na
  obrys,
* **kolejność** najbliższego sąsiada w blokach obrysy → skin → wypełnienie,
  szew najbliższy / wyrównany / losowy, skirt i brim,
* **G-code** z `;LAYER`, `;TYPE`, G0 na przejazdach, E bezwzględne (objętościowe
  w mm³, filament 1,75 mm albo bez E), retrakcja i z-hop opcjonalne, szacunek
  czasu.

### Druk 6-osiowy: tryby warstw i orientacja narzędzia

![warstwy stożkowe w podglądzie 3D](docs/gui_conical.png)

Slicer ma przełącznik *Tryb warstw* (w CLI `--layer-mode`), a cały łańcuch
(parser, translator, KRL, zasięg, kolizje) obsługuje orientację narzędzia
zmienną wzdłuż ścieżki:

| tryb | co robi | literatura |
|---|---|---|
| `planar` | warstwy płaskie, stała orientacja | |
| `conical` | warstwy stożkowe: model deformowany `z' = z ± tan(α)·r`, cięty płasko, ścieżki cofane; dysza pochylona o α na zewnątrz lub do środka, nawisy do 90° bez podpór | Wüthrich, Gubser 2021 (RotBot) |
| `bottom` | warstwy równoległe do powierzchni dolnej (druk na zakrzywionym podłożu, napawanie, naprawa): pole wysokości z siatki, `z' = z − h(x,y)` | Curved Layer FDM: Chakraborty 2008, Singamneni 2012 |
| `top` | warstwy dochodzące do powierzchni górnej bez schodków: `z' = z·H/h(x,y)` | Etienne i in. 2019 (CurviSlicer), Ahlers 2019 |
| `multi` | dekompozycja wielokierunkowa: segmenty ograniczone płaszczyznami, każdy drukowany płasko we własnym kierunku, kolejność z listy | Wu i in. 2017 (RoboFDM), Wu i in. 2019 |

W trybach nieplanarnych każdy punkt G-code dostaje oś narzędzia `I J K`
(wektor od detalu na zewnątrz, normalna warstwy), grubość ścieżki jest
korygowana o `cos(pochylenie)`, a nachylenia pola wysokości są ograniczane do
`max_slope`. Wskaźnik podpór liczy dla każdego trybu powierzchnię warstw
niepodpartą przez warstwę niżej w granicach `overhang_max` (porównanie
płaskie/stożkowe od ręki).

Parser rozumie `A B C` (orientacja KUKA, modalna) i `I J K` na G0/G1 (oś
narzędzia). Translator utrzymuje orientację per punkt (`SPL` z orientacją,
`$ORI_TYPE = #VAR`), łuki są odtwarzane tylko w łańcuchach o stałej
orientacji. Dla osi `I J K` wolny obrót wokół osi narzędzia dobiera
**solver orientacji** (Zhang i in., IEEE RA-L 2021, w wersji uproszczonej):
poprzedni obrót jest utrzymywany, dopóki pozycja jest osiągalna i |A5| jest
powyżej progu; inaczej próbkowane są obroty (i opcjonalnie dodatkowe
pochylenie w stożku) i wybierany jest najtańszy w sensie ruchu osi. Zasięg i
kolizje są liczone z orientacją per próbkę (interpolowaną wzdłuż SPL).

Ekstruder: oprócz wyjścia cyfrowego jest tryb analogowy proporcjonalny do
prędkości toru, `ANOUT ON $ANOUT[n] = k · $VEL_ACT` z `$VEL_FLT_OFF = TRUE`
(przykład dozowania z dokumentacji KRL); współczynnik `k` liczony jest z
E/mm segmentów slicera i wydajności ekstrudera przy sygnale 1,0, a przy
zmianie E/mm (np. inna wysokość warstwy) wyjście jest przeprogramowywane.

```bash
gcode2krl-slice czesc.step -o czesc.gcode --layer-mode conical --cone-angle 30
gcode2krl-slice czesc.step -o czesc.gcode --segments segmenty.txt      # tryb multi
gcode2krl czesc.gcode -o out/ --robot KR90R2700PRO --base-frame 1400 0 500 0 0 0 --tool-frame 0 0 150 0 0 0 \
    --extruder-anout 2 --extruder-flow-full 50 --spin-step 15 --singular-deg 8
```

Nie ma (jeszcze): optymalizacji pola skalarnego warstw (Dai 2018, S3-Slicer
2022, Neural Slicer 2024), automatycznej dekompozycji, osi zewnętrznych,
kolizji z już wydrukowanym materiałem (dodaj segmenty jako przeszkody
ręcznie).

W GUI sekcja *Slicer* kroi wgrany model i od razu wczytuje G-code do
translatora. Translator rozumie E (M82/M83, G92 E0) i z opcją *Ekstruder*
wstawia `$OUT[n] = TRUE/FALSE` wokół ruchów z ekstruzją, dzięki czemu KRL
steruje ekstruderem na robocie. Łuki, zasięg i kolizje działają na wyniku
slicera tak samo jak na G-code z CAM.

## Model robota (KR 90 R2700 pro i inne)

![robot w podglądzie 3D](docs/gui_robot.png)

Definicje robotów leżą w `gcode2krl/robots/<KLUCZ>.json` (plus `~/.gcode2krl/robots/`
dla własnych) razem z siatkami STL w `robots/meshes/`. Każdy plik ma pole
`sources` z pochodzeniem każdej grupy danych, pokazywane też w GUI w panelu *Robot*.

| klucz | model | geometria i siatki | dane osi |
|---|---|---|---|
| `KR90R2700PRO` (domyślny) | KR 90 R2700 pro (QUANTEC pro) | URDF KR 120 R2500 pro z ROS-Industrial (konwersja z CAD KUKA: 675 / 350 / 1150 / −41 / 215 mm), przedramię 1000 → 1200 mm zgodnie z zasięgiem 2696 mm; siatka przedramienia wydłużona o 200 mm | karta katalogowa KUKA KR 90 R2700 pro: A1 ±185° 136°/s, A2 −140/−5° 130°/s, A3 −120/+155° 120°/s, A4 ±350° 292°/s, A5 ±125° 258°/s, A6 ±350° 284°/s; 90 kg, 2696 mm, ±0,06 mm |
| `KR120R2500PRO` | KR 120 R2500 pro (QUANTEC pro) | URDF + STL ROS-Industrial | karta KR QUANTEC pro (limity/prędkości w URDF były skopiowane z KR 16-2 i zostały zastąpione) |
| `KR150R3100_2` | KR 150 R3100-2 (QUANTEC-2) | URDF + STL ROS-Industrial | URDF ROS-Industrial |
| `KR16_2` | KR 16-2 | URDF + STL ROS-Industrial | URDF, zgodny z kartą KR 16-2 (A2 +35/−155°, A3 +154/−130°, 156/156/156/330/330/615 °/s) |
| `KR6R900SIXX`, `KR6R700SIXX`, `KR10R1100SIXX`, `KR3R540` | KR AGILUS sixx | URDF + STL ROS-Industrial | URDF, zgodny z kartą KR AGILUS (np. KR 6 R900: ±170°, −190/+45°, −120/+156°, ±185°, ±120°, ±350°; 360/300/360/381/388/615 °/s) |
| `KR10R1420` | KR 10 R1420 | URDF + STL ROS-Industrial | URDF ROS-Industrial |

Siatki i origins stawów pochodzą z pakietów
[ROS-Industrial `kuka_experimental`](https://github.com/ros-industrial/kuka_experimental)
(Apache-2.0, pliki konwertowane z danych CAD KUKA; licencja i pochodzenie w
`gcode2krl/robots/NOTICE.md`). Konwencje osi w tych URDF są dokładnie
konwencjami KSS: A1 obraca wokół −Z, A4 i A6 wokół −X, `tool0` to kołnierz
obrócony o 90° wokół Y, więc HOME (A1 0, A2 −90, A3 90, A4 0, A5 0, A6 0)
daje dla KR 90 kołnierz w X 1765, Z 1784, B 90, jak na rzeczywistym KR QUANTEC.
Dane osi z URDF są oznaczone jako takie i należy je sprawdzić z kartą
katalogową własnej wersji robota (w sesji, w której to pisano, strony KUKA były
niedostępne; wartości dla KR 90 R2700 pro pochodzą z kilku zgodnych kopii karty
katalogowej KUKA).

**Własne dane z CAD KUKA.** Importer wczytuje URDF/xacro ze standardowym
łańcuchem KUKA i siatki STL (np. wyeksportowane z modeli STEP z KUKA Download
Center albo z KUKA.Sim) i buduje definicję z kapsułami kolizyjnymi:

```bash
gcode2krl-import --xacro kr90r2700pro_macro.xacro --key KR90R2700PRO --out ~/.gcode2krl/robots
gcode2krl-import --kuka-experimental /sciezka/do/kuka_experimental --out ~/.gcode2krl/robots
```

Bryły kolizyjne są liczone z siatek automatycznie: podstawa (nieruchoma)
dostaje obrócony prostopadłościan, każdy ruchomy człon zestaw kapsuł
(trójkąty dzielone do 16 mm, klastrowanie k-means, kapsuła na klaster z zapasem
połowy krawędzi, więc **cała powierzchnia siatki jest objęta**; test
`test_mesh_capsules_enclose_meshes` to sprawdza). Pary członów, które z
konstrukcji zawsze się stykają (np. obudowa A5 w końcówce przedramienia), są
zapisane w `disabled_pairs` i pomijane w autokolizjach.

Kinematyka: kinematyka prosta w konwencjach KSS i odwrotna w postaci
zamkniętej (8 rozwiązań: bark przód/tył, łokieć góra/dół, nadgarstek z
przewrotem i bez), wybór rozwiązania w limitach osi i najbliższego poprzedniej
pozie, sprawdzenie całej ścieżki. Testy obiegu FK → IK → FK przechodzą dla
wszystkich dziewięciu modeli.

**Stanowisko w GUI (sekcja *Stanowisko: TCP, baza, stół*).** TCP i bazę
ustawia się polami X Y Z A B C: TCP w układzie kołnierza (Z to długość
narzędzia), baza w układzie robota. Narzędzia można zapisywać jako presety
(wraz z promieniem uchwytu i ostrza), bazę wyznaczyć **metodą 3-punktową jak
w KSS** (początek, punkt na +X, punkt w płaszczyźnie XY, w układzie robota)
albo przyciskiem *detal przed robotem* (przesuwa bazę tak, by środek ścieżki
był w zadanej odległości od osi A1). Stół i materiał są generowane
automatycznie z bazy: prostopadłościan materiału pod obrysem ścieżki z górną
powierzchnią w Z = 0 bazy (ostrze może w niego wchodzić) i blat pod
materiałem sięgający do posadzki, jako wpisy `# auto` w przeszkodach; można je wyłączyć i
pisać ręcznie. Układ bazy i TCP są rysowane w podglądzie 3D. W CLI:
`--base-points OX OY OZ XX XY XZ PX PY PZ` liczy bazę z trzech punktów.

**Własne modele stołu, uchwytów i narzędzia (STL/OBJ).** Przycisk *dodaj
STL/OBJ* w sekcji *Stół roboczy* wgrywa plik do lokalnego magazynu
(`~/.gcode2krl/meshes`, zmienna `GCODE2KRL_MESH_DIR`) i dodaje go do
przeszkód jako wiersz `mesh NAZWA base X Y Z A B C ID jednostki [workpiece]`;
położenie edytuje się polami przy modelu lub w tekście przeszkód, a flaga
„materiał” zwalnia ostrze z kolizji z tym modelem. *Model narzędzia* wgrywa
siatkę w układzie kołnierza (mm, cm, m lub cale) i zastępuje parametryczną
kapsułę uchwytu; część w zasięgu „długości ostrza” od TCP jest traktowana
jako ostrze. Bryły kolizyjne są liczone z siatki (pudełko albo zestaw kapsuł,
zawsze z zapasem) i buforowane; siatki są rysowane w podglądzie, a przycisk
*kapsuły* pokazuje, co dokładnie jest sprawdzane. W CLI: `--tool-mesh
plik.step --tool-mesh-units mm` oraz wiersze `mesh` w pliku `--obstacles` ze
ścieżką do pliku zamiast ID.

**STEP i IGES.** Pliki `.step/.stp/.iges/.igs` (np. z KUKA Download Center
albo z własnego CAD) są wczytywane bezpośrednio i tesselowane kernelem
OpenCASCADE z zadaną tolerancją (pole *STEP: tolerancja tesselacji*, domyślnie
0,2 mm); jednostki są brane z pliku (mm, cale itd.), a siatka jest buforowana
jako STL w magazynie. Potrzebny jest jeden z kerneli:

```bash
pip install -e .[step]          # cadquery-ocp (OpenCASCADE), koła pip dla Windows, Linux, macOS
pip install gmsh                # alternatywa (na Linuksie wymaga libglu1-mesa)
```

Bez kernela GUI pokazuje komunikat, a STL/OBJ działają nadal.

Co robi sprawdzenie (`--robot`, w GUI panel *Robot*):

* `$BASE` podajesz w układzie robota (X Y Z A B C), `$TOOL` w układzie
  kołnierza; orientacja narzędzia na ścieżce to `--abc`. Oba układy można
  zapisać literalnie w programie zamiast `BASE_DATA[n]` / `TOOL_DATA[n]`.
* Dla próbek wzdłuż wygenerowanej ścieżki liczona jest kinematyka odwrotna.
  Raport: liczba próbek poza zasięgiem, poza limitami osi (z podziałem na
  osie) oraz zakres min/max każdej osi. CLI zwraca kod wyjścia 3, gdy coś
  jest poza zasięgiem.
* W GUI robot jest rysowany z siatek STL (gdy definicja je ma) albo z brył
  zastępczych, z kołnierzem, narzędziem, układem TCP i obwiednią zasięgu.
  Przycisk ▶ albo spacja animuje przejazd po ścieżce, suwak przewija próbki,
  panel *Osie* pokazuje kąty (na czerwono poza limitem) i pozwala ustawić
  robota ręcznie suwakami. Przycisk *cela* (`C`) dopasowuje widok do robota.

```bash
gcode2krl sciezka.nc -o out/ --robot KR90R2700PRO --base-frame 1200 0 500 0 0 0 --tool-frame 0 0 150 0 0 0
gcode2krl x.nc --robot list
```

### Kolizje

![zakładka Kolizje](docs/gui_collisions.png)

Sprawdzanie kolizji (`gcode2krl/collision.py`) jest liczone dla całej ścieżki
i ma być wiarygodne, dlatego:

* **Geometria dokładna, nie próbkowana.** Członki robota to kapsuły
  (odcinek + promień) obejmujące siatki STL z CAD, podstawa to obrócony
  prostopadłościan, narzędzie to kapsuły uchwytu i ostrza.
  Przeszkody to bryły wypukłe: pudełka (także obrócone), pionowe walce, kule i
  podłoga. Odległość kapsuła–kapsuła to dokładna odległość odcinek–odcinek,
  a kapsuła–przeszkoda to minimum wypukłej funkcji odległości wzdłuż odcinka
  (złoty podział, zbieżność do 1e-6 mm). Wszystko jest zwektoryzowane w numpy
  dla wszystkich próbek naraz.
* **Ostrze a materiał.** Przeszkoda oznaczona `workpiece` jest ignorowana
  tylko dla kapsuły ostrza (ostatnie `--cutter-len` mm narzędzia). Uchwyt,
  obudowa A6 i reszta robota muszą od materiału trzymać prześwit, więc za
  głębokie wejście albo za krótkie narzędzie wychodzi jako kolizja.
* **Autokolizje.** Pary członków nieprzylegających (przedramię, nadgarstek,
  A6, narzędzie i ostrze przeciw podstawie, kolumnie, ramieniu; narzędzie
  przeciw przedramieniu).
* **Ciągłość między próbkami.** Ścieżka jest próbkowana co `--path-step`
  (domyślnie 5 mm, wszystkie wierzchołki, łuki i G0 włącznie). Między
  sąsiednimi próbkami żaden punkt członu nie przemieszcza się dalej niż jego
  końce kapsuły (plus zapas na obrót), więc przedział jest *udowodniony* jako
  wolny, gdy `max(c_i, c_i+1) − δ_i ≥ prześwit`. Przedziały, których nie da się
  udowodnić, są dzielone na pół (IK w punkcie środkowym) do 6 razy; to, co
  nadal nie jest rozstrzygnięte, jest raportowane jako *nierozstrzygnięte*,
  nigdy nie jest po cichu uznane za bezpieczne. Test z cienką ścianką między
  dwiema próbkami odległymi o 200 mm pokazuje, że zagęszczanie ją znajduje.
* **Skoki konfiguracji i osobliwości.** Zmiana o ponad 30° na A1–A3 między
  sąsiednimi próbkami (łokieć/bark przeskoczył do innego rozwiązania, czego
  sterownik w bloku SPLINE nie wykona) i próbki z |A5| < 5° (osobliwość
  nadgarstka) są liczone i wypisywane w notatkach.
* **Raport.** Liczba próbek w kolizji i poniżej wymaganego prześwitu
  (`--margin`, domyślnie 10 mm), minimalny prześwit, tabela par (człon,
  przeszkoda, min. prześwit, próbka), wykres prześwitu wzdłuż ścieżki, punkty
  kolizji na czerwono w podglądzie, prześwit przy bieżącej pozie w panelu
  osi. CLI zwraca kod 3 przy kolizji lub nierozstrzygniętych przedziałach.

Przeszkody podaje się tekstem, jedna na linię, w układzie robota (`robot`)
albo obrabianego detalu (`base`):

```
floor posadzka robot 0
box stol robot 1300 0 240 900 900 480           # środek x y z, wymiary sx sy sz [a b c]
box material base 50 30 -10 140 100 20 workpiece
cylinder slupek base 150 30 60 15 120            # środek, promień, wysokość
sphere kamera robot 800 -600 1500 120
```

```bash
gcode2krl sciezka.nc -o out/ --robot KR90R2700PRO --base-frame 1200 0 500 0 0 0 \
    --tool-frame 0 0 150 0 0 0 --obstacles cela.txt --margin 10 --path-step 5
gcode2krl x.nc --obstacles help     # opis formatu
```

Czego model **nie** obejmuje: kolizji z kablami i osprzętem na ramieniu
(pakiety energetyczne nie są w siatkach), ruchów PTP (sprawdzane są tylko
ruchy po ścieżce) i dynamiki. Kapsuły obejmują siatki z zapasem, więc wynik
jest po stronie bezpiecznej. Przy własnym narzędziu ustaw `--tool-radius`,
`--cutter-len`, `--cutter-radius`; przy własnym robocie użyj
`gcode2krl-import`.

Dodanie własnego modelu: nowy wpis `RobotModel(...)` w `gcode2krl/robot.py`
(wymiary, limity, prędkości, podpowiedzi wizualne) pojawia się automatycznie
w CLI i w GUI.

## Wydajność

Dopasowanie okręgu jest liczone bez SVD (szybka ścieżka dla ścieżek płaskich,
rozkład własny macierzy 3×3 dla przestrzennych), rozszerzanie łuku używa
wyszukiwania podwajającego zamiast liniowego, a długie łańcuchy są cięte w
ostrych narożach (>60°, łuk i tak przez nie nie przejdzie) i liczone
równolegle w puli procesów. Skan powierzchni 3D z 60 000 wierzchołków:
3,8 s na 4 rdzeniach (kompresja 33×). Podgląd WebGL rysuje setki tysięcy
segmentów bez zacięć, bo renderuje tylko przy zmianie widoku.

## Użycie

```bash
gcode2krl sciezka.nc -o out/                      # bloki SPLINE (zalecane na KRC4)
gcode2krl sciezka.nc -o out/ --mode lin           # klasyczne LIN / CIRC z C_DIS
gcode2krl sciezka.nc -o out/ --tol 0.02 --abc 0 180 0 --tool 2 --base 3 --spindle-out 5
python -m gcode2krl.cli --help                    # pełna lista opcji
```

Przykładowy wynik dla `examples/fillet_rect.nc` (prostokąt z czterema
zaokrągleniami R10 zapisany jako 185 ruchów G1):

```
input : 188 moves (G0=3 G1=185 G2=0 G3=0), 194 lines
output: 12 motions (lines=5 arcs=4 spl points=0 in 0 runs, rapids=3)
compression 15.7x, max deviation 0.0018 mm (tolerance 0.05)
```

```
SPLINE WITH $VEL.CP = 0.0200, $ACC.CP = 2.00
  SLIN {X 90.000, Y 0.000, Z -1.000, A 0.000, B 180.000, C 0.000}
  SCIRC {X 97.071, Y 2.929, Z -1.000, A 0.000, B 180.000, C 0.000}, {X 100.000, Y 10.000, Z -1.000, A 0.000, B 180.000, C 0.000}
  SLIN {X 100.000, Y 50.000, Z -1.000, A 0.000, B 180.000, C 0.000}
  ...
ENDSPLINE C_SPL
```

## Jak działa dopasowanie łuków (`gcode2krl/arc_fitter.py`, `geometry.py`)

Łańcuch kolejnych ruchów G1 (ten sam posuw, bez przerw) jest dzielony na
odcinki proste i łuki tak, aby **cała ścieżka wyjściowa leżała w paśmie
tolerancji `--tol` wokół oryginalnej łamanej**, a liczba elementów była jak
najmniejsza. Kroki:

1. **Zachłanne rozszerzanie.** Od bieżącego wierzchołka rozszerzany jest
   niezależnie odcinek (dopóki wszystkie wierzchołki leżą w tolerancji od
   cięciwy) i łuk. Łuk jest po każdym dołożonym wierzchołku dopasowywany od
   nowa, więc błąd środka wyznaczonego z pierwszych trzech punktów nie tnie
   długiego łuku na kawałki. Łuk wygrywa, jeśli obejmuje więcej wierzchołków
   niż odcinek i ma ich co najmniej `--min-arc-points`.
2. **Dopasowanie okręgu.** Start algebraiczny metodą Taubina (Chernov), potem
   geometryczne dopasowanie najmniejszych kwadratów ze **środkiem na
   symetralnej cięciwy końcowej** (Gribov). Dzięki temu łuk przechodzi
   dokładnie przez pierwszy i ostatni wierzchołek, co jest wymagane przez
   `CIRC`/`SCIRC` (punkt startowy, pomocniczy i końcowy leżą na jednym okręgu).
   Okręgi są dopasowywane w 3D: najpierw płaszczyzna (SVD), potem okrąg w niej,
   więc łuki w dowolnej płaszczyźnie są obsługiwane.
3. **Walidacja łuku:** wierzchołki w tolerancji od płaszczyzny i okręgu,
   każda cięciwa w tolerancji od okręgu (dokładnie: `R - dist(środek, cięciwa)`,
   nie tylko strzałka), kąt rosnący monotonicznie (brak nawrotów), promień
   w `[--min-radius, --max-radius]`, kąt rozwarcia ≤ 360°. Limit promienia
   chroni przed zamianą prawie prostych odcinków na gigantyczne łuki.
4. **Korekta punktów podziału.** Dla każdej pary sąsiednich elementów wspólny
   punkt podziału jest przesuwany o ±`--refine-shift` wierzchołków i wybierany
   jest wariant z najmniejszym błędem. Naprawia to "kradzione" punkty na
   stycznych połączeniach odcinek/łuk i łuk/łuk o różnych promieniach.
5. **Scalanie.** Sąsiednie łuki, które da się opisać jednym okręgiem w
   tolerancji, są łączone, więc jeden duży łuk nigdy nie wychodzi jako ciąg
   małych.
6. **Pełne okręgi** (łańcuch zamknięty) są dzielone na dwa `SCIRC` po 180°.
7. **Punkty SPL.** Ciągi krótkich (≤ `--spl-max-segment`) dwuwierzchołkowych
   odcinków, których nie dało się złożyć w łuk ani w dłuższy odcinek
   (zmienna krzywizna: spirale, klotoidy, zaszumione skany), są przerzedzane
   algorytmem Douglasa-Peuckera z tolerancją `--tol` i zapisywane jako `SPL`.
   Sterownik KRC prowadzi przez nie własny splajn. `--no-spl` wyłącza to.

Wynik `max deviation` w raporcie to największe odchylenie wyjścia od
oryginalnej łamanej liczone wewnętrznie; funkcja `arc_fitter.max_deviation`
liczy je niezależnie (Hausdorff przez próbkowanie) i jest używana w testach.

## Mapowanie na KRL (`gcode2krl/krl_writer.py`)

| G-code | KRL (`--mode spline`) | KRL (`--mode lin`) |
|---|---|---|
| G0 | `LIN ... ` z `$VEL.CP = --rapid-vel` (C_DIS między kolejnymi G0) | tak samo |
| G1 (odcinek) | `SLIN {X,Y,Z,A,B,C}` w bloku `SPLINE` | `LIN ... C_DIS` |
| G1 (rozpoznany łuk), G2/G3 | `SCIRC {pkt pomocniczy}, {koniec}` | `CIRC ..., ... C_DIS` |
| G1 (mikrosegmenty, zmienna krzywizna) | `SPL {...}` | `LIN ... C_DIS` |
| G2/G3 helikalne | linearyzacja (`tol/2`), dalej jak G1 | jak G1 |
| F | `SPLINE WITH $VEL.CP = F/60000` (m/s) | `$VEL.CP = ...` |
| G4 P | `WAIT SEC` | |
| M3/M4/M5 | komentarz, `$OUT[--spindle-out]`, `$ANOUT[--spindle-anout]` | |
| M7/M8/M9 | komentarz, `$OUT[--coolant-out]` | |
| M6 | komentarz + `HALT` (`--no-halt-tool-change` wyłącza) | |
| M0/M1 | `HALT` | |
| M2/M30 | koniec programu | |

Zasady bloków `SPLINE`:

* blok jest planowany przez KRC jako jeden ruch, więc nie ma spadków prędkości
  między mikro-odcinkami, które występują przy tysiącach `LIN` (krótkie
  wyprzedzenie `$ADVANCE`);
* blok kończy się przy zmianie posuwu, w narożu (kąt skrętu >
  `--corner-angle`, domyślnie 10°) i po `--max-block-segments` segmentach;
* `ENDSPLINE C_SPL` wygładza przejście do następnego ruchu (`--no-block-approx`
  dla starszych wersji KSS, które nie znają `C_SPL`);
* w trybie `lin` naroża są domyślnie dokładne (bez `C_DIS`), `--round-corners`
  zaokrągla wszystko odległością `$APO.CDIS = --cdis`.

Pozycje są zapisywane jako `FRAME` ze stałą orientacją `--abc` (domyślnie
`A 0 B 180 C 0`, czyli oś Z narzędzia skierowana w dół w układzie BASE).
Układ współrzędnych G-code jest utożsamiany z `BASE_DATA[--base]`; dodatkowo
można dodać przesunięcie `--offset`. Narzędzie to `TOOL_DATA[--tool]`.

Programy dłuższe niż `--max-motions-per-file` ruchów są dzielone na
podprogramy `NAZWA_001.src`, `NAZWA_002.src`, ... wywoływane z `NAZWA.src`
(bloki `SPLINE` nigdy nie są przecinane). Pliki mają końce linii CRLF.

## Parser G-code (`gcode2krl/gcode_parser.py`)

Obsługuje: G0/G1/G2/G3 (modalne), łuki przez I/J/K i R (R ujemne = łuk >
180°), pełne okręgi (`G2 I-10`), helisy, G17/G18/G19, G20/G21, G90/G91,
G90.1/G91.1, G4, G94 (G93 ostrzeżenie), M0/M1/M2/M30/M3/M4/M5/M6/M7/M8/M9,
słowa F/S/T, komentarze `( )` i `;`, numery bloków N, `%`, `O1234`.
Pomijane z ostrzeżeniem: G28/G30, G92, G53, G54–G59 (współrzędne przechodzą
bez zmian), G41/G42 (kompensacja nie jest liczona), cykle G81–G89, splajny
G5.x. Niezgodność promienia początku i końca łuku jest korygowana przez
przesunięcie środka na symetralną cięciwy (z ostrzeżeniem powyżej 0,002 mm).

## Tolerancje i dokładność robota

Dokładność toru robota KUKA to dziesiąte części milimetra, więc domyślna
tolerancja 0,05 mm jest bezpieczna i daje mało fragmentacji. Przy
tolerancjach rzędu zaokrąglenia współrzędnych (0,001 mm) łuki będą się ciąć
na kawałki, bo CAM zapisuje wierzchołki z błędem do ±0,0007 mm.

## Ograniczenia

* Orientacja narzędzia jest stała (3-osiowy G-code). Brak obsługi osi A/B/C
  z G-code i kinematyk zewnętrznych (E1–E6).
* Model robota: siatki z ROS-Industrial (konwersja z CAD KUKA), kapsuły
  z zapasem zamiast dokładnych obudów, brak osprzętu na ramieniu; dane osi z
  URDF do sprawdzenia z kartą katalogową; nie zastępuje testu na sterowniku.
* `PTP $POS_ACT` jako ruch BCO na początku (`--no-bco` wyłącza); wejście na
  ścieżkę z pozycji HOME trzeba dopisać w programie nadrzędnym.
* Limit liczby punktów w bloku `SPLINE` zależy od wersji KSS; domyślne 500
  segmentów na blok jest ostrożne, sprawdź dokumentację swojej wersji.
* Składnia `SCIRC ... , CA kąt` nie jest używana; pełne okręgi są dzielone
  na dwa łuki.
* G93 (posuw odwrotny do czasu) i kompensacja promienia narzędzia nie są
  realizowane.

## Literatura

* Gribov, *Optimal Compression of a Polyline with Segments and Arcs* (2016)
  oraz *Approximate Fitting of a Circular Arc When Two Points Are Known* (2015)
* Al-Sharadqah, Chernov, *Error analysis for circle fitting algorithms*, EJS 2009
* Drysdale, Rote, Sturm, *Approximation of an open polygonal curve with a
  minimum number of circular arcs and biarcs*, CGTA 2008
* Horng, Li, *A dynamic programming approach for fitting digital planar curves
  with line segments and circular arcs*, PRL 2001
* Rosin, West, *Segmentation of edges into lines and arcs*, IVC 1989
* KUKA, *KSS 8.x Operating and Programming Instructions for System
  Integrators* (bloki SPLINE, SLIN, SCIRC, SPL, C_SPL)

## Struktura

```
gcode2krl/
  gcode_parser.py   parser modalny G-code -> ruchy i zdarzenia
  geometry.py       płaszczyzny, Taubin, dopasowanie z ustalonymi końcami, walidacja łuku 3D
  arc_fitter.py     segmentacja łamanej: odcinki + łuki, korekta podziałów, scalanie, pula procesów
  simplify.py       Douglas-Peucker (przerzedzanie punktów SPL)
  translator.py     łańcuchy G1 -> elementy ruchu, helisy, pełne okręgi, grupowanie SPL
  krl_writer.py     generowanie .src/.dat, bloki SPLINE, LIN/CIRC, podział na podprogramy
  robot.py          model KUKA z JSON (origins z URDF), kinematyka prosta/odwrotna, zasięg i limity
  robot_import.py   importer URDF/xacro + STL -> definicja JSON, kapsuły i OBB z siatek (gcode2krl-import)
  meshio.py         czytanie/zapis STL
  collision.py      kapsuły, przeszkody, dokładne odległości, sprawdzanie ciągłe, raport kolizji
  robots/           definicje robotów (JSON ze źródłami), siatki STL, LICENSE i NOTICE (Apache-2.0)
  cli.py            interfejs wiersza poleceń (gcode2krl)
  slicer/           slicer MIT: mesh_slice (kontury), toolpath (obrysy, wypełnienie), gcode_writer, cli (gcode2krl-slice)
  stepio.py         tesselacja STEP/IGES (OpenCASCADE przez OCP lub gmsh)
  meshstore.py      magazyn wgranych modeli (STL/OBJ/STEP)
  server.py         lokalny serwer HTTP + API JSON (gcode2krl-gui)
  web/              GUI: index.html, app.js, viewer.js (WebGL), robot.js (model robota), style.css
examples/           fillet_rect.nc, circle_g1.nc, mixed.nc
tests/              pytest (rdzeń, parser, writer, CLI, serwer)
.github/workflows/  CI: pytest na Ubuntu i Windows, Python 3.10 i 3.12
```
