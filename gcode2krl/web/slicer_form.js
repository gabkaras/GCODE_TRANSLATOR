'use strict';
// Slicer parameter form generated from a schema: every SliceParams field is here,
// grouped like the big desktop slicers, with a help text per field. Nothing is
// configured in files - the GUI is the only place.
(() => {
  const N = (k, label, help, o = {}) => ({ k, label, help, type: 'number', ...o });
  const C = (k, label, help, o = {}) => ({ k, label, help, type: 'check', ...o });
  const S = (k, label, help, opts, o = {}) => ({ k, label, help, type: 'select', opts, ...o });
  const T = (k, label, help, o = {}) => ({ k, label, help, type: 'textarea', ...o });
  const A = (k, label, help, buttons) => ({ k, label, help, type: 'actions', buttons, extra: true });

  const GROUPS = [
    { id: 'model', title: 'Model: położenie i orientacja', open: true, fields: [
      N('rot_x', 'Obrót X', ['Obrót modelu', 'Obrót wokół osi X, Y i Z stołu (w tej kolejności, stopnie). Po obrocie model jest stawiany na Z = 0 i centrowany.'], { unit: '°', step: 15, def: 0 }),
      N('rot_y', 'Obrót Y', ['Obrót Y', 'Obrót wokół osi Y stołu.'], { unit: '°', step: 15, def: 0 }),
      N('rot_z', 'Obrót Z', ['Obrót Z', 'Obrót wokół osi pionowej — ustawia model względem robota (np. długim bokiem wzdłuż X bazy).'], { unit: '°', step: 15, def: 0 }),
      N('scale', 'Skala', ['Skala', 'Jednorodna skala modelu (1 = wymiary z pliku; 0.001 dla modelu w metrach).'], { unit: '×', step: 0.1, min: 0.001, def: 1 }),
      C('mirror_x', 'Lustro X', ['Lustro', 'Odbicie modelu względem płaszczyzny YZ / XZ / XY; orientacja ścianek jest naprawiana automatycznie.'], { def: false }),
      C('mirror_y', 'Lustro Y', ['Lustro Y', 'Odbicie względem płaszczyzny XZ.'], { def: false }),
      C('mirror_z', 'Lustro Z', ['Lustro Z', 'Odbicie względem płaszczyzny XY (do góry nogami).'], { def: false }),
      S('auto_orient', 'Automatyczna orientacja', ['Automatyczna orientacja', 'Brak: obroty jak wyżej. Ułóż płasko: największa płaska ścianka bryły na stole. Minimalne nawisy: orientacja spośród ścianek otoczki o najmniejszej powierzchni wymagającej podpór (przy równych: większy styk ze stołem, mniejsza wysokość). Zastępuje ręczne obroty przy każdym krojeniu; przyciski niżej wpisują wynik do pól na stałe.'], [['none', 'brak (obroty ręczne)'], ['lay_flat', 'ułóż płasko'], ['min_overhang', 'minimalne nawisy']], { def: 'none' }),
      A('orient_actions', 'Oblicz orientację', ['Oblicz orientację', 'Liczy orientację wybranego modelu na serwerze i wpisuje obroty do pól X Y Z (można je potem poprawić ręcznie).'], [['orient_flat', 'ułóż płasko'], ['orient_min', 'min. nawisy'], ['orient_reset', 'zeruj']]),
      S('print_sequence', 'Kilka modeli', ['Kolejność drukowania', 'Gdy na stole jest kilka modeli (lista „Modele na stole”): warstwa po warstwie — wszystkie razem, jak jeden obiekt; obiekt po obiekcie — każdy model dokończony przed następnym (sprawdź prześwit uchwytu obok gotowych obiektów; raport kolizji nie zna wydrukowanego materiału).'], [['all', 'warstwa po warstwie'], ['sequential', 'obiekt po obiekcie']], { def: 'all' }),
    ] },
    { id: 'layers', title: 'Warstwy', open: false, fields: [
      N('layer_height', 'Wysokość warstwy', ['Wysokość warstwy', 'Odstęp między kolejnymi warstwami. Dla ekstruzji robotem typowo 1–3 mm przy dyszy 3–6 mm; mniejsza = dokładniej i wolniej.'], { unit: 'mm', step: 0.5, min: 0.05, def: 2 }),
      N('first_layer_height', 'Pierwsza warstwa', ['Pierwsza warstwa', 'Wysokość pierwszej warstwy. Zwykle nieco większa niż pozostałe, żeby wyrównać stół i poprawić przyczepność.'], { unit: 'mm', step: 0.5, min: 0.05, def: 2 }),
      C('adaptive_layers', 'Adaptacyjna wysokość warstw', ['Warstwy adaptacyjne', 'Wysokość warstwy dobierana automatycznie z nachylenia powierzchni: płaskie, pochyłe fragmenty dostają cieńsze warstwy (mniejsze schodki), pionowe ściany grubsze. Tylko warstwy płaskie.'], { def: false }),
      N('layer_height_min', 'Adapt.: min. wysokość', ['Minimalna wysokość (adaptacyjne)', 'Dolna granica wysokości warstwy przy doborze adaptacyjnym.'], { unit: 'mm', step: 0.5, min: 0.05, def: 1, show: v => v.adaptive_layers }),
      N('layer_height_max', 'Adapt.: maks. wysokość', ['Maksymalna wysokość (adaptacyjne)', 'Górna granica wysokości warstwy przy doborze adaptacyjnym (pionowe ściany).'], { unit: 'mm', step: 0.5, min: 0.05, def: 3, show: v => v.adaptive_layers }),
      N('adaptive_quality', 'Adapt.: maks. schodek', ['Jakość (adaptacyjne)', 'Największa dopuszczalna wysokość schodka na pochyłej powierzchni. Mniejsza wartość = cieńsze warstwy na skosach.'], { unit: 'mm', step: 0.1, min: 0.01, def: 0.5, show: v => v.adaptive_layers }),
      N('z_offset', 'Przesunięcie Z', ['Przesunięcie Z', 'Dodawane do wszystkich wysokości w G-code (np. korekta zera narzędzia nad stołem).'], { unit: 'mm', step: 0.1, def: 0 }),
    ] },
    { id: 'ranges', title: 'Zakresy wysokości i haki KRL', fields: [
      T('ranges', 'Zakresy wysokości', ['Zakresy wysokości', 'Nadpisania parametrów w przedziale Z, jedna linia: „z_od z_do klucz=wartość …”, np. „0 20 layer_height=1 speed_outer=10 infill_density=0.5”. Dozwolone klucze: wysokość warstwy, szerokości, obrysy, wypełnienie, góra/dół, prędkości, mnożnik, pochylenia, fuzzy, ironing, gęstość podpór. Późniejsze linie wygrywają.'], { def: '', rows: 3, placeholder: '0 20 layer_height=1 speed_outer=10\n20 200 infill_pattern=gyroid infill_density=0.15' }),
      T('layer_krl', 'Haki KRL na warstwach', ['Haki KRL', 'Własne linie programu robota wstawiane przed warstwą: „layer N: …” (numer warstwy od 0), „z Z: …” (pierwsza warstwa na wysokości ≥ Z mm), „every: …” lub „every N: …” (co N warstw). Np. „z 100: HALT” (pauza na wymianę materiału), „every 10: WAIT FOR $IN[5]”. Linie trafiają do KRL dosłownie, między ścieżkami (ekstruder wyłączony).'], { def: '', rows: 3, placeholder: 'layer 5: WAIT SEC 10\nz 100: HALT\nevery 10: $OUT[12] = TRUE' }),
      N('layer_wait', 'Czekanie między warstwami', ['Czekanie między warstwami', 'WAIT SEC przed każdą warstwą poza pierwszą — czas wiązania / stygnięcia materiału (beton, glina, duże przekroje). Robot stoi w miejscu startu warstwy.'], { unit: 's', step: 5, min: 0, def: 0 }),
    ] },
    { id: 'walls', title: 'Ściany', fields: [
      N('bead_width', 'Szerokość ścieżki', ['Szerokość ścieżki', 'Szerokość wytłoczonej ścieżki (bead) — z reguły średnica dyszy ekstrudera na robocie (3–10 mm). Używana wszędzie, gdzie szerokość per funkcja jest 0. Wyznacza mm³ materiału na mm ruchu.'], { unit: 'mm', step: 0.5, min: 0.1, def: 4 }),
      N('outer_wall_width', 'Szer. obrysu zewn.', ['Szerokość obrysu zewnętrznego', 'Osobna szerokość dla obrysu zewnętrznego (0 = domyślna). Węższy obrys zewnętrzny daje dokładniejsze wymiary.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('inner_wall_width', 'Szer. obrysów wewn.', ['Szerokość obrysów wewnętrznych', 'Osobna szerokość dla obrysów wewnętrznych (0 = domyślna). Szersze obrysy wewnętrzne skracają czas.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('perimeters', 'Liczba obrysów', ['Obrysy', 'Liczba pętli konturu na każdej warstwie (zewnętrzna + wewnętrzne). Więcej = mocniejsza ściana.'], { step: 1, min: 0, def: 2 }),
      S('wall_order', 'Kolejność obrysów', ['Kolejność obrysów', 'Wewnętrzne → zewnętrzny: zewnętrzny drukowany na końcu, ładniejsza powierzchnia. Zewnętrzny → wewnętrzne: lepsze wymiary i nawisy. Najpierw wypełnienie: wypełnienie i warstwy pełne przed obrysami (obrysy opierają się o wypełnienie, lepsze nawisy, gorsza powierzchnia).'], [['inner_outer', 'wewnętrzne → zewnętrzny'], ['outer_inner', 'zewnętrzny → wewnętrzne'], ['infill_first', 'najpierw wypełnienie']], { def: 'inner_outer' }),
      S('seam', 'Szew', ['Szew', 'Gdzie zaczynać każdą pętlę obrysu. Najbliższy: od punktu najbliższego poprzedniej pozycji. Wyrównany: zawsze w tym samym miejscu (szew w jednej linii). Z tyłu: po stronie +Y. Najostrzejszy narożnik: szew ukryty w narożu. Losowy: rozrzucone starty.'], [['nearest', 'najbliższy'], ['aligned', 'wyrównany'], ['rear', 'z tyłu (+Y)'], ['corner', 'najostrzejszy narożnik'], ['random', 'losowy']], { def: 'nearest' }),
      S('fuzzy_skin', 'Fuzzy skin (chropowata powierzchnia)', ['Fuzzy skin', 'Losowe odchylanie punktów obrysu w poziomie — chropowata, matowa faktura ściany (ukrywa linie warstw). Zewnętrzny: tylko obrys zewnętrzny. Wszystkie: wszystkie obrysy. Pierwsza warstwa zostaje gładka.'], [['none', 'wyłączone'], ['outer', 'obrys zewnętrzny'], ['all', 'wszystkie obrysy']], { def: 'none' }),
      N('fuzzy_thickness', 'Fuzzy: grubość', ['Grubość fuzzy skin', 'Rozrzut punktów (od szczytu do szczytu) w mm.'], { unit: 'mm', step: 0.2, min: 0.1, def: 0.6, show: v => v.fuzzy_skin !== 'none' }),
      N('fuzzy_point_distance', 'Fuzzy: odstęp punktów', ['Odstęp punktów fuzzy skin', 'Co ile mm wstawiany jest losowy punkt; mniejszy = drobniejsza faktura, więcej punktów SPL.'], { unit: 'mm', step: 0.5, min: 0.3, def: 3, show: v => v.fuzzy_skin !== 'none' }),
      C('spiral_vase', 'Tryb spiralny (wazon)', ['Tryb spiralny', 'Jeden ciągły obrys zewnętrzny bez szwu: Z rośnie płynnie wzdłuż ścieżki (helisa), bez wypełnienia i warstw górnych powyżej warstw dolnych. Idealny do dużych cienkościennych form drukowanych robotem.'], { def: false }),
      N('small_perimeter_length', 'Małe obrysy: próg', ['Małe obrysy', 'Pętle krótsze niż ta długość (np. małe otwory) są drukowane z prędkością „małe obrysy”. 0 = wyłączone.'], { unit: 'mm', step: 5, min: 0, def: 0 }),
      C('gap_fill', 'Wypełniaj szczeliny i cienkie ściany', ['Gap fill', 'Miejsca za wąskie na kolejny obrys lub wypełnienie (szczeliny między obrysami, cienkie żebra, ostre końcówki) są drukowane pojedynczą, zwężoną ścieżką zamiast zostawać puste.'], { def: true }),
      N('gap_fill_min_width', 'Gap fill: min. szerokość', ['Minimalna szerokość szczeliny', 'Ułamek szerokości ścieżki; węższe szczeliny są pomijane (ekstruder nie zwęzi ścieżki bardziej).'], { unit: '× szer.', step: 0.05, min: 0.05, max: 1, def: 0.25, show: v => v.gap_fill }),
      N('xy_compensation', 'Kompensacja XY', ['Kompensacja XY', 'Powiększa (+) lub pomniejsza (−) każdy kontur o podaną wartość. Koryguje rozlewanie materiału: detal za duży → wartość ujemna.'], { unit: 'mm', step: 0.05, def: 0 }),
      N('hole_compensation', 'Kompensacja otworów', ['Kompensacja otworów', 'Powiększa otwory o podaną wartość (otwory zwykle wychodzą za małe).'], { unit: 'mm', step: 0.05, def: 0 }),
      N('elephant_foot', 'Stopa słonia', ['Stopa słonia', 'Zwęża kontur pierwszej warstwy o tę wartość, bo dociśnięta pierwsza warstwa rozlewa się na boki.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('min_feature', 'Min. element', ['Minimalny element', 'Fragmenty przekroju cieńsze niż ta wartość są odrzucane jako szum siatki.'], { unit: 'mm', step: 0.1, min: 0, def: 0.3 }),
    ] },
    { id: 'infill', title: 'Wypełnienie', fields: [
      N('infill_density', 'Gęstość', ['Gęstość wypełnienia', 'Procent wypełnienia wnętrza (0 = pusty, 100 = pełny).'], { unit: '%', step: 5, min: 0, max: 100, def: 20, pct: true }),
      S('infill_pattern', 'Wzór', ['Wzór wypełnienia', 'Linie: naprzemienne kierunki co warstwę, najszybsze. Zygzak: linie połączone na brzegu (mniej przejazdów). Siatka: dwa kierunki w warstwie. Trójkąty: trzy kierunki, sztywne. Sześcienne: trzy kierunki przesuwane z Z, równa wytrzymałość w 3D. Gyroid (fale): faliste linie zazębiające się między warstwami. Plaster miodu: sześciokąty. Koncentryczne: pętle równoległe do obrysu, elastyczne.'], [['lines', 'linie'], ['zigzag', 'zygzak'], ['grid', 'siatka'], ['triangles', 'trójkąty'], ['cubic', 'sześcienne'], ['gyroid', 'gyroid (fale)'], ['honeycomb', 'plaster miodu'], ['concentric', 'koncentryczne'], ['none', 'brak']], { def: 'lines' }),
      N('infill_angle', 'Kąt', ['Kąt wypełnienia', 'Kierunek linii pierwszej warstwy; dla wzorów liniowych kolejne warstwy obracają się o 90°.'], { unit: '°', step: 15, def: 45 }),
      N('infill_width', 'Szer. ścieżki wypełnienia', ['Szerokość wypełnienia', 'Osobna szerokość ścieżki dla wypełnienia (0 = domyślna). Szersza = szybciej.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('infill_overlap', 'Zakładka z obrysem', ['Zakładka wypełnienia', 'O ile (ułamek szerokości ścieżki) wypełnienie wchodzi w najbardziej wewnętrzny obrys, żeby się z nim związało.'], { unit: '× szer.', step: 0.05, min: 0, max: 1, def: 0.15 }),
      N('infill_combine', 'Łącz warstwy wypełnienia', ['Łączenie wypełnienia', 'Rzadkie wypełnienie drukowane co N warstw z N-krotnym wydatkiem (grubsze linie, krótszy czas). 1 = na każdej warstwie.'], { step: 1, min: 1, max: 5, def: 1 }),
      N('solid_threshold_area', 'Małe obszary jako pełne', ['Próg pełnego wypełnienia', 'Obszary rzadkiego wypełnienia mniejsze niż ta powierzchnia są drukowane jako pełne (0 = wyłączone).'], { unit: 'mm²', step: 10, min: 0, def: 0 }),
    ] },
    { id: 'skin', title: 'Góra, dół i mosty', fields: [
      N('top_layers', 'Warstwy górne', ['Warstwy górne', 'Ile ostatnich warstw (i warstw nad pustką) wypełnić w 100 %.'], { step: 1, min: 0, def: 2 }),
      N('bottom_layers', 'Warstwy dolne', ['Warstwy dolne', 'Ile pierwszych warstw (i warstw pod pustką / nad nawisem) wypełnić w 100 %.'], { step: 1, min: 0, def: 2 }),
      S('top_pattern', 'Wzór górny', ['Wzór powierzchni górnej', 'Linie: zygzak po liniach. Zygzak: linie połączone (mniej przejazdów). Monotoniczny: wszystkie linie w jednym kierunku, najładniejsza powierzchnia. Koncentryczne: pętle od brzegu.'], [['lines', 'linie'], ['zigzag', 'zygzak'], ['monotonic', 'monotoniczny'], ['concentric', 'koncentryczne']], { def: 'zigzag' }),
      S('bottom_pattern', 'Wzór dolny', ['Wzór powierzchni dolnej', 'Jak wzór górny, dla warstw dolnych i powierzchni nad pustką (poza mostami).'], [['lines', 'linie'], ['zigzag', 'zygzak'], ['monotonic', 'monotoniczny'], ['concentric', 'koncentryczne']], { def: 'zigzag' }),
      N('top_bottom_width', 'Szer. ścieżki góra/dół', ['Szerokość warstw pełnych', 'Osobna szerokość ścieżki dla powierzchni górnych, dolnych i mostów (0 = domyślna).'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      C('bridges', 'Wykrywaj mosty', ['Mosty', 'Powierzchnia nad pustką zakotwiczona z obu stron jest mostem: drukowana prostymi liniami w kierunku łączącym podpory, z własną prędkością i wydatkiem, zamiast jako nawis.'], { def: true }),
      N('bridge_flow', 'Wydatek na mostach', ['Wydatek na mostach', 'Mnożnik ekstruzji dla linii mostów (mniej materiału = lepiej napięte nitki).'], { unit: '×', step: 0.05, min: 0.2, max: 2, def: 1, show: v => v.bridges }),
      C('ironing', 'Prasowanie (ironing) powierzchni górnych', ['Ironing', 'Dodatkowe przejście po gotowych powierzchniach górnych: gęste, monotoniczne linie z bardzo małym wydatkiem i małą prędkością wygładzają wierzch (dysza wygrzewa i rozprowadza materiał).'], { def: false }),
      N('ironing_flow', 'Ironing: wydatek', ['Wydatek ironingu', 'Mnożnik ekstruzji podczas prasowania (ułamek normalnego).'], { unit: '×', step: 0.05, min: 0, max: 1, def: 0.15, show: v => v.ironing }),
      N('ironing_spacing', 'Ironing: odstęp linii', ['Odstęp linii ironingu', 'Odstęp między liniami prasowania (znacznie mniejszy niż szerokość ścieżki).'], { unit: 'mm', step: 0.5, min: 0.2, def: 1, show: v => v.ironing }),
      N('ironing_speed', 'Ironing: prędkość', ['Prędkość ironingu', 'Prędkość TCP podczas prasowania.'], { unit: 'mm/s', step: 5, min: 1, def: 15, show: v => v.ironing }),
    ] },
    { id: 'support', title: 'Nawisy i podpory', fields: [
      C('overhang_detect', 'Zwalniaj na nawisach', ['Wykrywanie nawisów', 'Każdy odcinek obrysu dostaje stopień „nad pustką” z odległości od warstwy niżej (lub podpory). Odcinki nad powietrzem są drukowane wolniej (prędkość „nawisy”), interpolowanie od 25 % do 75 % szerokości ścieżki bez podparcia.'], { def: true }),
      N('overhang_max', 'Maks. nawis bez podpór', ['Kąt nawisu', 'Największy kąt ściany względem pionu, który drukuje się bez podpór. Powyżej: obszar liczony jako wymagający podpór (wskaźnik) i generowane są podpory, jeśli włączone.'], { unit: '°', step: 5, min: 0, max: 90, def: 45 }),
      S('support', 'Podpory', ['Generowanie podpór', 'Miejsca podpór są wykrywane automatycznie: obszary warstwy, których warstwa niżej nie niesie w granicach kąta nawisu. Brak: tylko wskaźnik. Zwykłe: kolumny rzutowane prosto w dół (od stołu lub modelu), z przerwą Z i XY i warstwami łącznikowymi. Drzewiaste: końcówki gałęzi na wysepkach nawisu, gałęzie schodzą pod kątem, łączą się w pnie, omijają model z odstępem XY i mogą opierać się na modelu — mniej materiału, łatwiejsze usuwanie, mniej śladów.'], [['none', 'brak (tylko wskaźnik)'], ['normal', 'zwykłe (kolumny)'], ['tree', 'drzewiaste']], { def: 'none' }),
      N('tree_branch_angle', 'Drzewo: kąt gałęzi', ['Kąt gałęzi', 'Maksymalne odchylenie gałęzi od pionu na każdej warstwie (ruch w bok = wysokość warstwy × tg kąta). Większy kąt = gałęzie szybciej się łączą i omijają model, ale muszą być drukowalne bez podparcia (ok. 40–50°).'], { unit: '°', step: 5, min: 0, max: 70, def: 40, show: v => v.support === 'tree' }),
      N('tree_tip_diameter', 'Drzewo: średnica końcówki', ['Średnica końcówki', 'Średnica gałęzi tuż pod nawisem (0 = dwie szerokości ścieżki).'], { unit: 'mm', step: 1, min: 0, def: 0, show: v => v.support === 'tree' }),
      N('tree_trunk_diameter', 'Drzewo: maks. średnica pnia', ['Średnica pnia', 'Górna granica średnicy po połączeniu gałęzi i po rozroście z wysokością.'], { unit: 'mm', step: 5, min: 2, def: 40, show: v => v.support === 'tree' }),
      N('tree_diameter_growth', 'Drzewo: przyrost średnicy', ['Przyrost średnicy', 'Kąt rozszerzania się gałęzi ku dołowi (promień rośnie o tg kąta na mm wysokości) — stabilność wysokich drzew.'], { unit: '°', step: 1, min: 0, max: 30, def: 5, show: v => v.support === 'tree' }),
      N('tree_tip_spacing', 'Drzewo: odstęp końcówek', ['Odstęp końcówek', 'Odstęp siatki końcówek na obszarze nawisu (0 = auto ≈ 1,8 średnicy końcówki). Mniejszy = gęstsze podparcie powierzchni.'], { unit: 'mm', step: 1, min: 0, def: 0, show: v => v.support === 'tree' }),
      C('support_on_model', 'Drzewo: gałęzie mogą opierać się na modelu', ['Podpory na modelu', 'Gałąź, która nie może ominąć modelu, kończy się na nim (odstęp XY zachowany). Wyłączone = gałąź jest obcinana przez model.'], { def: true, show: v => v.support === 'tree' }),
      N('support_density', 'Gęstość podpór', ['Gęstość podpór', 'Procent wypełnienia kolumn podpór.'], { unit: '%', step: 5, min: 5, max: 100, def: 20, pct: true, show: v => v.support !== 'none' }),
      S('support_pattern', 'Wzór podpór', ['Wzór podpór', 'Linie: jeden kierunek na wszystkich warstwach (łatwe usuwanie). Siatka: dwa kierunki, sztywniejsze. Ściana + linie: obrys każdej kolumny / gałęzi plus rzadkie linie w środku — zalecane dla drzew.'], [['lines', 'linie'], ['grid', 'siatka'], ['walls', 'ściana + linie']], { def: 'lines', show: v => v.support !== 'none' }),
      N('support_angle', 'Kierunek linii podpór', ['Kierunek podpór', 'Kąt linii podpór; łącznik jest do nich prostopadły.'], { unit: '°', step: 15, def: 0, show: v => v.support !== 'none' }),
      N('support_z_gap', 'Przerwa Z', ['Przerwa Z', 'Liczba pustych warstw między szczytem podpory a modelem, żeby podpora się odrywała.'], { unit: 'warstw', step: 1, min: 0, max: 3, def: 1, show: v => v.support !== 'none' }),
      N('support_xy_gap', 'Przerwa XY', ['Przerwa XY', 'Odstęp między podporą a ścianami modelu w poziomie.'], { unit: 'mm', step: 0.5, min: 0, def: 1, show: v => v.support !== 'none' }),
      N('support_interface_layers', 'Warstwy łącznikowe', ['Warstwy łącznikowe (interface)', 'Ile górnych warstw podpory (pod modelem) drukować gęściej, żeby powierzchnia nad podporą była gładsza.'], { step: 1, min: 0, max: 5, def: 2, show: v => v.support !== 'none' }),
      N('support_interface_density', 'Gęstość łącznika', ['Gęstość łącznika', 'Procent wypełnienia warstw łącznikowych.'], { unit: '%', step: 10, min: 10, max: 100, def: 60, pct: true, show: v => v.support !== 'none' && v.support_interface_layers > 0 }),
      N('support_min_area', 'Min. obszar nawisu', ['Minimalny obszar', 'Wysepki nawisu mniejsze niż ta powierzchnia nie dostają podpór (drobiazgi radzą sobie same).'], { unit: 'mm²', step: 1, min: 0, def: 4, show: v => v.support !== 'none' }),
      T('support_modifiers', 'Blokady i wymuszenia podpór', ['Modyfikatory podpór', 'Pudełka w układzie modelu (po wyśrodkowaniu, Z od stołu), jedna linia: „block cx cy cz sx sy sz” usuwa podpory wewnątrz pudełka (np. otwór, który ma zostać czysty), „force cx cy cz sx sy sz” dodaje podpory wszędzie w pudełku poza modelem.'], { def: '', rows: 3, placeholder: 'block 0 0 20 40 40 30\nforce 60 0 10 20 20 20', show: v => v.support !== 'none' }),
    ] },
    { id: 'extrusion', title: 'Ekstruzja i retrakcja', fields: [
      S('extrusion_mode', 'Materiał w ścieżce', ['Ilość materiału', 'Objętościowo: każdy odcinek niesie mm³ materiału (szerokość × wysokość × długość); z tego KRL wylicza wydatek analogowy, czasy cofnięcia i dopuszczenia oraz masę. Bez materiału: sama geometria — ekstruder sterowany poza programem.'], [['volumetric', 'objętościowo (mm³)'], ['none', 'bez materiału']], { def: 'volumetric' }),
      N('extrusion_multiplier', 'Mnożnik wydatku', ['Mnożnik wydatku', 'Skaluje ilość materiału na mm ścieżki (a więc sygnał analogowy ekstrudera). Kalibracja: ścianka za cienka → zwiększ.'], { unit: '×', step: 0.02, min: 0.2, max: 3, def: 1 }),
      N('first_layer_flow', 'Wydatek 1. warstwy', ['Wydatek pierwszej warstwy', 'Mnożnik ekstruzji tylko dla pierwszej warstwy (więcej materiału = lepsza przyczepność).'], { unit: '×', step: 0.05, min: 0.5, max: 2, def: 1 }),
      N('max_volumetric_speed', 'Wydajność ekstrudera', ['Wydajność ekstrudera', 'Największy wydatek mm³/s, jaki daje ekstruder na robocie. Prędkość TCP każdej ścieżki jest obcinana tak, żeby szerokość × wysokość × prędkość nie przekroczyły tej wartości. 0 = bez limitu.'], { unit: 'mm³/s', step: 5, min: 0, def: 0 }),
      N('material_density', 'Gęstość materiału', ['Gęstość materiału', 'Do oszacowania masy wydruku (PLA ≈ 1,24, PETG ≈ 1,27, ABS ≈ 1,05, beton/glina wg receptury).'], { unit: 'g/cm³', step: 0.01, min: 0.1, def: 1.24 }),
      N('retract_length', 'Cofnięcie przed przejazdem', ['Cofnięcie ekstrudera', 'Ile mm³ cofnąć przed przejazdem, żeby nie ciągnąć nitek. W KRL: impuls na wyjściu rewersu o czasie mm³ / wydajność, a po przejeździe dopuszczenie tej samej ilości (WAIT SEC z włączonym ekstruderem). 0 = brak.'], { unit: 'mm³', step: 5, min: 0, def: 0 }),
      N('retract_extra_prime', 'Dodatkowe dopuszczenie', ['Dodatkowe dopuszczenie', 'Ekstra mm³ po przejeździe, kompensuje materiał, który wyciekł podczas ruchu.'], { unit: 'mm³', step: 1, def: 0, show: v => v.retract_length > 0 }),
      N('min_travel_for_retract', 'Min. przejazd do retrakcji', ['Minimalny przejazd', 'Retrakcja i z-hop tylko przy przejazdach dłuższych niż ta wartość.'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.retract_length > 0 || v.z_hop > 0 }),
      C('retract_layer_change', 'Cofnięcie przy zmianie warstwy', ['Cofnięcie przy zmianie warstwy', 'Cofa ekstruder na czas przejścia na kolejną warstwę.'], { def: false, show: v => v.retract_length > 0 }),
      N('z_hop', 'Podniesienie na przejeździe', ['Podniesienie na przejeździe', 'Robot podnosi dyszę o tę wartość na czas przejazdu; chroni wydruk przed zahaczeniem.'], { unit: 'mm', step: 1, min: 0, def: 0 }),
      N('wipe_distance', 'Wipe (wycieranie)', ['Wipe', 'Po zamkniętym obrysie dysza cofa się po ścieżce o tę długość w czasie cofania ekstrudera — ukrywa kroplę na szwie.'], { unit: 'mm', step: 1, min: 0, def: 0, show: v => v.retract_length > 0 }),
      N('coast_length', 'Coasting', ['Coasting', 'Ostatnie milimetry zamkniętego obrysu z wyłączonym ekstruderem: ciśnienie w dyszy dokańcza ścieżkę, nie ma zgrubienia na szwie.'], { unit: 'mm', step: 1, min: 0, def: 0 }),
    ] },
    { id: 'speeds', title: 'Prędkości', fields: [
      N('speed_outer', 'Obrys zewnętrzny', ['Prędkość obrysu zewnętrznego', 'Wolniej = ładniejsza powierzchnia i dokładniejsze wymiary.'], { unit: 'mm/s', step: 5, min: 1, def: 20 }),
      N('speed_perimeter', 'Obrysy wewnętrzne', ['Prędkość obrysów wewnętrznych', 'Mogą być szybsze niż zewnętrzny.'], { unit: 'mm/s', step: 5, min: 1, def: 30 }),
      N('speed_infill', 'Wypełnienie', ['Prędkość wypełnienia', 'Rzadkie wypełnienie zwykle najszybciej.'], { unit: 'mm/s', step: 5, min: 1, def: 40 }),
      N('speed_top', 'Góra / dół', ['Prędkość warstw pełnych', 'Powierzchnie górne i dolne; wolniej = gładsza góra.'], { unit: 'mm/s', step: 5, min: 1, def: 20 }),
      N('speed_bridge', 'Mosty', ['Prędkość mostów', 'Linie mostów; wolno i równo, żeby nitki się nie zrywały.'], { unit: 'mm/s', step: 5, min: 1, def: 15 }),
      N('speed_overhang', 'Nawisy', ['Prędkość nawisów', 'Docelowa prędkość odcinków w pełni nad powietrzem (interpolowana od prędkości normalnej).'], { unit: 'mm/s', step: 5, min: 1, def: 10 }),
      N('speed_gap', 'Szczeliny', ['Prędkość gap fill', 'Pojedyncze ścieżki w szczelinach i cienkich ścianach.'], { unit: 'mm/s', step: 5, min: 1, def: 20 }),
      N('speed_support', 'Podpory', ['Prędkość podpór', 'Podpory i warstwy łącznikowe.'], { unit: 'mm/s', step: 5, min: 1, def: 40 }),
      N('small_perimeter_speed', 'Małe obrysy', ['Prędkość małych obrysów', 'Dla pętli krótszych niż próg „małe obrysy” (sekcja Ściany).'], { unit: 'mm/s', step: 5, min: 1, def: 10 }),
      N('speed_first_layer', 'Pierwsza warstwa', ['Prędkość pierwszej warstwy', 'Wszystkie ścieżki pierwszej warstwy (przyczepność).'], { unit: 'mm/s', step: 5, min: 1, def: 15 }),
      N('speed_travel', 'Przejazdy', ['Prędkość przejazdów', 'Przejazdy bez materiału (w KRL LIN z $VEL.CP przejazdów).'], { unit: 'mm/s', step: 10, min: 1, def: 150 }),
      N('min_layer_time', 'Min. czas warstwy', ['Minimalny czas warstwy', 'Jeśli warstwa trwałaby krócej, wszystkie jej prędkości są skalowane w dół, żeby poprzednia zdążyła zastygnąć (małe wierzchołki). 0 = wyłączone.'], { unit: 's', step: 5, min: 0, def: 0 }),
      N('min_speed', 'Min. prędkość', ['Minimalna prędkość', 'Dolna granica przy spowalnianiu z powodu minimalnego czasu warstwy.'], { unit: 'mm/s', step: 1, min: 0.5, def: 5, show: v => v.min_layer_time > 0 }),
    ] },
    { id: 'adhesion', title: 'Przyczepność: skirt, brim, raft', fields: [
      N('skirt_loops', 'Skirt: pętle', ['Skirt', 'Luźne pętle wokół modelu na pierwszej warstwie, rozruch ekstrudera (0 = brak).'], { step: 1, min: 0, def: 0 }),
      N('skirt_distance', 'Skirt: odstęp', ['Odstęp skirtu', 'Odległość skirtu od modelu (i brimu).'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.skirt_loops > 0 }),
      N('skirt_layers', 'Skirt: warstwy', ['Warstwy skirtu', 'Na ilu pierwszych warstwach drukować skirt; wysoki skirt działa jak osłona przed przeciągiem.'], { step: 1, min: 1, def: 1, show: v => v.skirt_loops > 0 }),
      N('prime_line_length', 'Linia rozruchowa', ['Linia rozruchowa (prime line)', 'Prosta linia przed modelem drukowana jako pierwsza — rozrusza ekstruder i zdejmuje nadmiar materiału (0 = brak).'], { unit: 'mm', step: 10, min: 0, def: 0 }),
      N('prime_line_offset', 'Linia rozruchowa: odstęp', ['Odstęp linii rozruchowej', 'Odległość linii od modelu (za brimem i skirtem), po stronie −Y.'], { unit: 'mm', step: 5, min: 0, def: 20, show: v => v.prime_line_length > 0 }),
      N('brim_width', 'Brim: szerokość', ['Brim', 'Obwódka przyklejona do modelu, poprawia przyczepność małych podstaw (0 = brak).'], { unit: 'mm', step: 1, min: 0, def: 0 }),
      N('raft_layers', 'Raft: warstwy', ['Raft', 'Tratwa pod modelem: rzadka warstwa podstawy, warstwy pośrednie i pełna warstwa górna; model stoi na niej zamiast na stole (0 = brak).'], { step: 1, min: 0, max: 5, def: 0 }),
      N('raft_margin', 'Raft: margines', ['Margines raftu', 'O ile raft wystaje poza obrys modelu (i podpór).'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.raft_layers > 0 }),
    ] },
    { id: 'multiaxis', title: 'Orientacja dyszy i tryb warstw (6 osi)', fields: [
      N('tilt_lead', 'Pochylenie w kierunku ruchu', ['Pochylenie dyszy w kierunku ruchu', 'Oś dyszy pochylona o ten kąt do przodu (+) lub do tyłu (−) względem kierunku ruchu, jak palnik w WAAM albo pochylona dysza przy druku betonem. Zapisywane jako oś I J K przy każdym punkcie; obrót wokół osi dobiera translator.'], { unit: '°', step: 5, min: -60, max: 60, def: 0 }),
      N('tilt_side', 'Pochylenie w bok', ['Pochylenie dyszy w bok', 'Oś dyszy pochylona w lewo (+) lub w prawo (−) od kierunku ruchu — np. odchylenie od sąsiedniej ścianki albo lepszy dostęp uchwytu.'], { unit: '°', step: 5, min: -60, max: 60, def: 0 }),
      S('layer_mode', 'Warstwy', ['Tryb warstw', 'Płaskie: klasyczne poziome warstwy. Stożkowe (RotBot): warstwy na stożku, nawisy bez podpór. Konforemne: warstwy kopiują powierzchnię dolną lub górną modelu. Wielokierunkowe: segmenty drukowane w różnych kierunkach (definicja niżej). Podpory, raft, spirala i warstwy adaptacyjne działają tylko dla warstw płaskich.'], [['planar', 'płaskie'], ['conical', 'stożkowe (RotBot)'], ['bottom', 'konforemne do powierzchni dolnej'], ['top', 'konforemne do powierzchni górnej'], ['multi', 'wielokierunkowe (segmenty)']], { def: 'planar' }),
      N('cone_angle', 'Stożek: kąt', ['Kąt stożka', 'Kąt między warstwą a poziomem; robot pochyla narzędzie o tyle samo.'], { unit: '°', step: 5, min: 1, max: 80, def: 30, show: v => v.layer_mode === 'conical' }),
      S('cone_outward', 'Stożek: kierunek', ['Kierunek stożka', 'Na zewnątrz: warstwy opadają od osi (nawisy na zewnątrz). Do środka: wznoszą się od osi (nawisy nad pustką w środku).'], [['true', 'na zewnątrz'], ['false', 'do środka']], { def: 'true', show: v => v.layer_mode === 'conical' }),
      N('cone_cx', 'Oś stożka X', ['Oś stożka', 'Punkt X, Y (układ modelu po wyśrodkowaniu), przez który przechodzi oś stożka. Puste = środek modelu.'], { unit: 'mm', step: 5, def: '', placeholder: 'auto', show: v => v.layer_mode === 'conical' }),
      N('cone_cy', 'Oś stożka Y', ['Oś stożka Y', 'Jak wyżej, współrzędna Y. Puste = środek.'], { unit: 'mm', step: 5, def: '', placeholder: 'auto', show: v => v.layer_mode === 'conical' }),
      N('hf_cell', 'Pole wysokości: komórka', ['Komórka pola wysokości', 'Rozdzielczość siatki próbkującej powierzchnię (mniejsza = dokładniej, wolniej).'], { unit: 'mm', step: 1, min: 0.5, def: 5, show: v => v.layer_mode === 'bottom' || v.layer_mode === 'top' }),
      N('max_slope', 'Maks. nachylenie warstw', ['Maksymalne nachylenie', 'Ogranicza pochylenie warstw konforemnych (a więc narzędzia), żeby robot nie wchodził w nieosiągalne orientacje.'], { unit: '°', step: 5, min: 1, max: 85, def: 45, show: v => v.layer_mode === 'bottom' || v.layer_mode === 'top' }),
      T('segments', 'Segmenty', ['Segmenty', 'Jedna definicja na linię. „segment NAZWA dx dy dz” zaczyna segment drukowany w kierunku (dx, dy, dz); kolejne „plane px py pz nx ny nz” to płaszczyzny tnące — do segmentu należy strona wskazana przez normalną. Segmenty drukowane są po kolei.'], { def: '', rows: 5, placeholder: 'segment korpus 0 0 1\nplane 0 0 40 0 0 -1\nsegment ramie 1 0 0\nplane 0 0 40 0 0 1', show: v => v.layer_mode === 'multi', extra: true }),
      C('tool_axis', 'Zapisuj oś narzędzia I J K', ['Oś narzędzia', 'Przy każdym punkcie ścieżki o zmiennej lub pochylonej orientacji zapisywany jest wektor osi dyszy (I J K); translator ustawia z niego orientację robota (ramka $TOOL, także dla dyszy zamontowanej pod kątem). Bez tego ścieżka jest tłumaczona ze stałą orientacją A B C.'], { def: true }),
    ] },
    { id: 'misc', title: 'Inne', fields: [
      C('comments', 'Komentarze ;TYPE / ;LAYER w ścieżce', ['Komentarze', 'Zapisuje ;LAYER, ;Z, ;HEIGHT i ;TYPE w eksportowanej ścieżce G-code (czytelność). Własne linie KRL ustawia się w sekcji „Program KRL”.'], { def: true }),
      C('origin_center', 'Ustaw model na środku stołu', ['Wyśrodkowanie', 'Przesuwa model tak, aby środek jego obrysu XY był na środku blatu (w bazie $BASE: długość/2, szerokość/2 – baza leży w narożniku stołu); wyłącz, żeby zachować współrzędne XY z pliku. Modele z platera mają pozycje podane w platerze.'], { def: true }),
    ] },
  ];

  const host = document.getElementById('slicer-form');
  if (!host) return;
  const HELP = {};
  const inputs = {};
  const fieldDefs = {};

  function build() {
    for (const g of GROUPS) {
      const det = document.createElement('details');
      det.className = 'sf-group'; det.id = 'sf-' + g.id; if (g.open) det.open = true;
      const sum = document.createElement('summary'); sum.textContent = g.title; det.appendChild(sum);
      for (const f of g.fields) {
        fieldDefs[f.k] = f;
        const id = 'sl_' + f.k;
        HELP[id] = f.help;
        let row;
        if (f.type === 'check') {
          row = document.createElement('label'); row.className = 'row check';
          const inp = document.createElement('input'); inp.type = 'checkbox'; inp.id = id; inp.checked = !!f.def;
          const sp = document.createElement('span'); sp.textContent = f.label;
          row.append(inp, sp); inputs[f.k] = inp;
        } else if (f.type === 'actions') {
          row = document.createElement('div'); row.className = 'row actions-row';
          const sp = document.createElement('span'); sp.textContent = f.label;
          const wrap = document.createElement('span'); wrap.className = 'btns';
          for (const [act, txt] of f.buttons) {
            const b = document.createElement('button'); b.type = 'button'; b.className = 'mini'; b.textContent = txt; b.dataset.action = act;
            b.addEventListener('click', () => window.dispatchEvent(new CustomEvent('slicer-action', { detail: act })));
            wrap.appendChild(b);
          }
          row.append(sp, wrap); inputs[f.k] = { value: '', type: 'actions' };
        } else if (f.type === 'textarea') {
          row = document.createElement('label'); row.className = 'col';
          const sp = document.createElement('span'); sp.textContent = f.label;
          const ta = document.createElement('textarea'); ta.id = id; ta.rows = f.rows || 3; ta.spellcheck = false; if (f.placeholder) ta.placeholder = f.placeholder;
          row.append(sp, ta); inputs[f.k] = ta;
        } else {
          row = document.createElement('div'); row.className = 'row';
          const sp = document.createElement('span'); sp.textContent = f.label;
          if (f.unit) { const u = document.createElement('small'); u.textContent = ' [' + f.unit + ']'; sp.appendChild(u); }
          let inp;
          if (f.type === 'select') {
            inp = document.createElement('select'); inp.id = id;
            for (const [v, t] of f.opts) { const o = document.createElement('option'); o.value = v; o.textContent = t; inp.appendChild(o); }
            inp.value = f.def;
          } else {
            inp = document.createElement('input'); inp.type = 'number'; inp.id = id;
            if (f.step != null) inp.step = f.step; if (f.min != null) inp.min = f.min; if (f.max != null) inp.max = f.max;
            if (f.placeholder) inp.placeholder = f.placeholder;
            inp.value = f.def;
          }
          row.append(sp, inp); inputs[f.k] = inp;
        }
        row.dataset.field = f.k;
        det.appendChild(row);
      }
      host.appendChild(det);
    }
  }

  function rawValues() {
    const v = {};
    for (const [k, el] of Object.entries(inputs)) {
      if (el.type === 'actions') continue;
      if (el.type === 'checkbox') v[k] = el.checked;
      else if (el.tagName === 'SELECT') v[k] = el.value === 'true' ? true : el.value === 'false' ? false : el.value;
      else if (el.type === 'number') v[k] = el.value === '' ? '' : Number(el.value);
      else v[k] = el.value;
    }
    return v;
  }

  function updateVisibility() {
    const v = rawValues();
    for (const [k, f] of Object.entries(fieldDefs)) {
      if (!inputs[k].closest) continue;
      const row = inputs[k].closest('[data-field]');
      if (row) row.classList.toggle('hidden', f.show ? !f.show(v) : false);
    }
  }

  function params() {
    const v = rawValues();
    const out = {};
    for (const [k, f] of Object.entries(fieldDefs)) {
      if (f.extra) continue;  // not a SliceParams field (segments go separately)
      let val = v[k];
      if (f.type === 'number') {
        if (val === '') { if (k === 'cone_cx' || k === 'cone_cy') out[k] = null; continue; }
        if (f.pct) val = val / 100;
      }
      out[k] = val;
    }
    return out;
  }

  function extras() { return { segments: inputs.segments ? inputs.segments.value : '' }; }

  const KEY = 'gcode2krl.slicer';
  function save() { try { localStorage.setItem(KEY, JSON.stringify(rawValues())); } catch (e) {} }
  function load() {
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem(KEY) || 'null'); } catch (e) {}
    if (!saved) return;
    for (const [k, el] of Object.entries(inputs)) {
      if (!(k in saved) || el.type === 'actions') continue;
      const val = saved[k];
      if (el.type === 'checkbox') el.checked = !!val;
      else if (el.tagName === 'SELECT') el.value = String(val);
      else el.value = val === null || val === undefined ? '' : val;
    }
  }
  function reset() {
    for (const [k, el] of Object.entries(inputs)) {
      const f = fieldDefs[k];
      if (el.type === 'actions') continue;
      if (el.type === 'checkbox') el.checked = !!f.def;
      else el.value = f.def === null || f.def === undefined ? '' : String(f.def);
    }
    save(); updateVisibility();
  }

  build(); load(); updateVisibility();
  host.addEventListener('input', () => { save(); updateVisibility(); });
  host.addEventListener('change', () => { save(); updateVisibility(); });
  const resetBtn = document.getElementById('btn-slicer-reset');
  if (resetBtn) resetBtn.addEventListener('click', reset);

  function setValue(k, val) { const el = inputs[k]; if (!el || el.type === 'actions') return; if (el.type === 'checkbox') el.checked = !!val; else el.value = String(val); }
  window.SLICER_FORM = { params, extras, reset, help: HELP, groups: GROUPS, inputs, setValue, save, updateVisibility };
})();
