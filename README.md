# gcode2krl – translator G-code → KUKA KRL z odtwarzaniem łuków

Zamienia programy G-code (G0/G1/G2/G3) na programy KRL dla sterowników KUKA
(KRC4, KSS 8.x). Główna funkcja: **ścieżki, które CAM wygenerował jako setki
krótkich odcinków G1, są z powrotem rozpoznawane jako łuki** i zapisywane jako
`SCIRC`/`CIRC`. Reszta trafia do `SLIN`/`LIN`, a fragmenty o zmiennej
krzywiźnie do punktów `SPL` wewnątrz bloków `SPLINE`.

```
G-code (.nc) ──parser──> ruchy ──dopasowanie łuków──> SLIN / SCIRC / SPL ──writer──> PROG.src + PROG.dat
```

![GUI gcode2krl](docs/gui.png)

## Instalacja

```bash
pip install -e .          # wymaga Python >= 3.10 i numpy
pip install -e .[test]    # dodatkowo pytest
pytest                    # 47 testów
```

## GUI

```bash
gcode2krl-gui                       # http://127.0.0.1:8765, otwiera przeglądarkę
gcode2krl-gui --port 9000 --no-browser
```

Lokalna aplikacja webowa bez żadnych zależności poza biblioteką standardową
Pythona i numpy (działa offline, np. na komputerze przy robocie):

* przeciągnij plik G-code do okna albo wybierz przykład; wynik liczy się
  automatycznie po każdej zmianie opcji (suwak tolerancji, tryb, orientacja…),
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
* opcje zapamiętywane w przeglądarce (`localStorage`).

![zakładka KRL](docs/gui_krl.png)

API JSON serwera (do własnej automatyzacji):

| metoda | ścieżka | opis |
|---|---|---|
| GET | `/api/examples`, `/api/example/<nazwa>` | przykładowe pliki |
| POST | `/api/translate` | `{gcode, filename, options}` → statystyki, ostrzeżenia, pliki, geometria podglądu (float32 w base64) |
| GET | `/api/zip/<token>` | zip z plikami ostatniego tłumaczenia |

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
  cli.py            interfejs wiersza poleceń (gcode2krl)
  server.py         lokalny serwer HTTP + API JSON (gcode2krl-gui)
  web/              GUI: index.html, app.js, viewer.js (WebGL), style.css
examples/           fillet_rect.nc, circle_g1.nc, mixed.nc
tests/              pytest (rdzeń, parser, writer, CLI, serwer)
.github/workflows/  CI: pytest na Ubuntu i Windows, Python 3.10 i 3.12
```
