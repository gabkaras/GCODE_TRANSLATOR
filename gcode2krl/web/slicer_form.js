'use strict';
// Slicer parameter form generated from a schema: every SliceParams field is here,
// grouped like the big desktop slicers, with a help text per field. Nothing is
// configured in files - the GUI is the only place.
(() => {
  const N = (k, label, help, o = {}) => ({ k, label, help, type: 'number', ...o });
  const C = (k, label, help, o = {}) => ({ k, label, help, type: 'check', ...o });
  const S = (k, label, help, opts, o = {}) => ({ k, label, help, type: 'select', opts, ...o });
  const T = (k, label, help, o = {}) => ({ k, label, help, type: 'textarea', ...o });

  const GROUPS = [
    { id: 'layers', title: 'Warstwy', open: true, fields: [
      N('layer_height', 'Wysokość warstwy', ['Wysokość warstwy', 'Odstęp między kolejnymi warstwami. Dla ekstruzji robotem typowo 1–3 mm przy dyszy 3–6 mm; mniejsza = dokładniej i wolniej.'], { unit: 'mm', step: 0.1, min: 0.05, def: 1 }),
      N('first_layer_height', 'Pierwsza warstwa', ['Pierwsza warstwa', 'Wysokość pierwszej warstwy. Zwykle nieco większa niż pozostałe, żeby wyrównać stół i poprawić przyczepność.'], { unit: 'mm', step: 0.1, min: 0.05, def: 1 }),
      C('adaptive_layers', 'Adaptacyjna wysokość warstw', ['Warstwy adaptacyjne', 'Wysokość warstwy dobierana automatycznie z nachylenia powierzchni: płaskie, pochyłe fragmenty dostają cieńsze warstwy (mniejsze schodki), pionowe ściany grubsze. Tylko warstwy płaskie.'], { def: false }),
      N('layer_height_min', 'Adapt.: min. wysokość', ['Minimalna wysokość (adaptacyjne)', 'Dolna granica wysokości warstwy przy doborze adaptacyjnym.'], { unit: 'mm', step: 0.1, min: 0.05, def: 0.5, show: v => v.adaptive_layers }),
      N('layer_height_max', 'Adapt.: maks. wysokość', ['Maksymalna wysokość (adaptacyjne)', 'Górna granica wysokości warstwy przy doborze adaptacyjnym (pionowe ściany).'], { unit: 'mm', step: 0.1, min: 0.05, def: 2, show: v => v.adaptive_layers }),
      N('adaptive_quality', 'Adapt.: maks. schodek', ['Jakość (adaptacyjne)', 'Największa dopuszczalna wysokość schodka na pochyłej powierzchni. Mniejsza wartość = cieńsze warstwy na skosach.'], { unit: 'mm', step: 0.05, min: 0.01, def: 0.3, show: v => v.adaptive_layers }),
      N('z_offset', 'Przesunięcie Z', ['Przesunięcie Z', 'Dodawane do wszystkich wysokości w G-code (np. korekta zera narzędzia nad stołem).'], { unit: 'mm', step: 0.1, def: 0 }),
    ] },
    { id: 'walls', title: 'Ściany', fields: [
      N('bead_width', 'Szerokość ścieżki', ['Szerokość ścieżki', 'Domyślna szerokość wytłoczonej ścieżki (bead). Używana wszędzie, gdzie szerokość per funkcja jest 0. Wyznacza ilość materiału E na mm ruchu.'], { unit: 'mm', step: 0.1, min: 0.1, def: 2 }),
      N('outer_wall_width', 'Szer. obrysu zewn.', ['Szerokość obrysu zewnętrznego', 'Osobna szerokość dla obrysu zewnętrznego (0 = domyślna). Węższy obrys zewnętrzny daje dokładniejsze wymiary.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('inner_wall_width', 'Szer. obrysów wewn.', ['Szerokość obrysów wewnętrznych', 'Osobna szerokość dla obrysów wewnętrznych (0 = domyślna). Szersze obrysy wewnętrzne skracają czas.'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      N('perimeters', 'Liczba obrysów', ['Obrysy', 'Liczba pętli konturu na każdej warstwie (zewnętrzna + wewnętrzne). Więcej = mocniejsza ściana.'], { step: 1, min: 0, def: 2 }),
      S('wall_order', 'Kolejność obrysów', ['Kolejność obrysów', 'Wewnętrzne → zewnętrzny: zewnętrzny drukowany na końcu, ładniejsza powierzchnia. Zewnętrzny → wewnętrzne: lepsze wymiary i nawisy, bo zewnętrzny jest podparty tylko warstwą niżej.'], [['inner_outer', 'wewnętrzne → zewnętrzny'], ['outer_inner', 'zewnętrzny → wewnętrzne']], { def: 'inner_outer' }),
      S('seam', 'Szew', ['Szew', 'Gdzie zaczynać każdą pętlę obrysu. Najbliższy: od punktu najbliższego poprzedniej pozycji. Wyrównany: zawsze w tym samym miejscu (szew w jednej linii). Z tyłu: po stronie +Y. Losowy: rozrzucone starty.'], [['nearest', 'najbliższy'], ['aligned', 'wyrównany'], ['rear', 'z tyłu (+Y)'], ['random', 'losowy']], { def: 'nearest' }),
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
      N('top_layers', 'Warstwy górne', ['Warstwy górne', 'Ile ostatnich warstw (i warstw nad pustką) wypełnić w 100 %.'], { step: 1, min: 0, def: 3 }),
      N('bottom_layers', 'Warstwy dolne', ['Warstwy dolne', 'Ile pierwszych warstw (i warstw pod pustką / nad nawisem) wypełnić w 100 %.'], { step: 1, min: 0, def: 3 }),
      S('top_pattern', 'Wzór górny', ['Wzór powierzchni górnej', 'Linie: zygzak po liniach. Zygzak: linie połączone (mniej przejazdów). Monotoniczny: wszystkie linie w jednym kierunku, najładniejsza powierzchnia. Koncentryczne: pętle od brzegu.'], [['lines', 'linie'], ['zigzag', 'zygzak'], ['monotonic', 'monotoniczny'], ['concentric', 'koncentryczne']], { def: 'zigzag' }),
      S('bottom_pattern', 'Wzór dolny', ['Wzór powierzchni dolnej', 'Jak wzór górny, dla warstw dolnych i powierzchni nad pustką (poza mostami).'], [['lines', 'linie'], ['zigzag', 'zygzak'], ['monotonic', 'monotoniczny'], ['concentric', 'koncentryczne']], { def: 'zigzag' }),
      N('top_bottom_width', 'Szer. ścieżki góra/dół', ['Szerokość warstw pełnych', 'Osobna szerokość ścieżki dla powierzchni górnych, dolnych i mostów (0 = domyślna).'], { unit: 'mm', step: 0.1, min: 0, def: 0 }),
      C('bridges', 'Wykrywaj mosty', ['Mosty', 'Powierzchnia nad pustką zakotwiczona z obu stron jest mostem: drukowana prostymi liniami w kierunku łączącym podpory, z własną prędkością i wydatkiem, zamiast jako nawis.'], { def: true }),
      N('bridge_flow', 'Wydatek na mostach', ['Wydatek na mostach', 'Mnożnik ekstruzji dla linii mostów (mniej materiału = lepiej napięte nitki).'], { unit: '×', step: 0.05, min: 0.2, max: 2, def: 1, show: v => v.bridges }),
    ] },
    { id: 'support', title: 'Nawisy i podpory', fields: [
      C('overhang_detect', 'Zwalniaj na nawisach', ['Wykrywanie nawisów', 'Każdy odcinek obrysu dostaje stopień „nad pustką” z odległości od warstwy niżej (lub podpory). Odcinki nad powietrzem są drukowane wolniej (prędkość „nawisy”), interpolowanie od 25 % do 75 % szerokości ścieżki bez podparcia.'], { def: true }),
      N('overhang_max', 'Maks. nawis bez podpór', ['Kąt nawisu', 'Największy kąt ściany względem pionu, który drukuje się bez podpór. Powyżej: obszar liczony jako wymagający podpór (wskaźnik) i generowane są podpory, jeśli włączone.'], { unit: '°', step: 5, min: 0, max: 90, def: 45 }),
      S('support', 'Podpory', ['Generowanie podpór', 'Brak: tylko wskaźnik, ile powierzchni wymaga podpór. Zwykłe: kolumny podpór pod nawisami, od stołu lub od modelu, z przerwą Z i XY i warstwami łącznikowymi; drukowane rzadkim wzorem po modelu w każdej warstwie.'], [['none', 'brak (tylko wskaźnik)'], ['normal', 'zwykłe']], { def: 'none' }),
      N('support_density', 'Gęstość podpór', ['Gęstość podpór', 'Procent wypełnienia kolumn podpór.'], { unit: '%', step: 5, min: 5, max: 100, def: 20, pct: true, show: v => v.support !== 'none' }),
      S('support_pattern', 'Wzór podpór', ['Wzór podpór', 'Linie: jeden kierunek na wszystkich warstwach (łatwe usuwanie). Siatka: dwa kierunki, sztywniejsze.'], [['lines', 'linie'], ['grid', 'siatka']], { def: 'lines', show: v => v.support !== 'none' }),
      N('support_angle', 'Kierunek linii podpór', ['Kierunek podpór', 'Kąt linii podpór; łącznik jest do nich prostopadły.'], { unit: '°', step: 15, def: 0, show: v => v.support !== 'none' }),
      N('support_z_gap', 'Przerwa Z', ['Przerwa Z', 'Liczba pustych warstw między szczytem podpory a modelem, żeby podpora się odrywała.'], { unit: 'warstw', step: 1, min: 0, max: 3, def: 1, show: v => v.support !== 'none' }),
      N('support_xy_gap', 'Przerwa XY', ['Przerwa XY', 'Odstęp między podporą a ścianami modelu w poziomie.'], { unit: 'mm', step: 0.5, min: 0, def: 1, show: v => v.support !== 'none' }),
      N('support_interface_layers', 'Warstwy łącznikowe', ['Warstwy łącznikowe (interface)', 'Ile górnych warstw podpory (pod modelem) drukować gęściej, żeby powierzchnia nad podporą była gładsza.'], { step: 1, min: 0, max: 5, def: 2, show: v => v.support !== 'none' }),
      N('support_interface_density', 'Gęstość łącznika', ['Gęstość łącznika', 'Procent wypełnienia warstw łącznikowych.'], { unit: '%', step: 10, min: 10, max: 100, def: 60, pct: true, show: v => v.support !== 'none' && v.support_interface_layers > 0 }),
      N('support_min_area', 'Min. obszar nawisu', ['Minimalny obszar', 'Wysepki nawisu mniejsze niż ta powierzchnia nie dostają podpór (drobiazgi radzą sobie same).'], { unit: 'mm²', step: 1, min: 0, def: 4, show: v => v.support !== 'none' }),
    ] },
    { id: 'extrusion', title: 'Ekstruzja i retrakcja', fields: [
      S('extrusion_mode', 'Sposób liczenia E', ['Ekstruzja E', 'Objętościowa: E w mm³ (pompy, ekstrudery granulatu). Filament: E w mm filamentu jak w FDM. Bez E: tylko geometria, ekstruder sterowany osobno (np. analogowo z KRL).'], [['volumetric', 'objętościowa (mm³)'], ['filament', 'filament (mm)'], ['none', 'bez E']], { def: 'volumetric' }),
      N('filament_diameter', 'Średnica filamentu', ['Średnica filamentu', 'Do przeliczenia objętości na mm filamentu w trybie „filament”.'], { unit: 'mm', step: 0.05, min: 0.5, def: 1.75, show: v => v.extrusion_mode === 'filament' }),
      N('extrusion_multiplier', 'Mnożnik ekstruzji', ['Mnożnik ekstruzji (flow)', 'Skaluje całe E. Kalibracja: ścianka za cienka → zwiększ.'], { unit: '×', step: 0.02, min: 0.2, max: 3, def: 1 }),
      N('first_layer_flow', 'Wydatek 1. warstwy', ['Wydatek pierwszej warstwy', 'Mnożnik ekstruzji tylko dla pierwszej warstwy (więcej materiału = lepsza przyczepność).'], { unit: '×', step: 0.05, min: 0.5, max: 2, def: 1 }),
      N('max_volumetric_speed', 'Maks. wydatek objętościowy', ['Limit objętościowy', 'Górna granica mm³/s, jaką podaje ekstruder. Posuw każdej ścieżki jest obcinany tak, żeby szerokość × wysokość × prędkość nie przekroczyły tej wartości. 0 = bez limitu.'], { unit: 'mm³/s', step: 5, min: 0, def: 0 }),
      N('material_density', 'Gęstość materiału', ['Gęstość materiału', 'Do oszacowania masy wydruku (PLA ≈ 1,24, PETG ≈ 1,27, ABS ≈ 1,05, beton/glina wg receptury).'], { unit: 'g/cm³', step: 0.01, min: 0.1, def: 1.24 }),
      N('retract_length', 'Retrakcja', ['Retrakcja', 'Cofnięcie materiału (E ujemne) przed przejazdem, żeby nie ciągnąć nitek. 0 = brak.'], { unit: 'mm E', step: 0.5, min: 0, def: 0 }),
      N('retract_speed', 'Prędkość retrakcji', ['Prędkość retrakcji', 'Posuw ruchu E przy cofaniu i ponownym podaniu.'], { unit: 'mm/min', step: 100, min: 60, def: 1800, show: v => v.retract_length > 0 }),
      N('retract_extra_prime', 'Dodatkowe podanie', ['Dodatkowe podanie po retrakcji', 'Ekstra E po przejeździe, kompensuje materiał wyciekły podczas ruchu.'], { unit: 'mm E', step: 0.1, def: 0, show: v => v.retract_length > 0 }),
      N('min_travel_for_retract', 'Min. przejazd do retrakcji', ['Minimalny przejazd', 'Retrakcja i z-hop tylko przy przejazdach dłuższych niż ta wartość.'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.retract_length > 0 || v.z_hop > 0 }),
      C('retract_layer_change', 'Retrakcja przy zmianie warstwy', ['Retrakcja przy zmianie warstwy', 'Cofa materiał na czas przejścia na kolejną warstwę.'], { def: false, show: v => v.retract_length > 0 }),
      N('z_hop', 'Z-hop', ['Z-hop', 'Podniesienie narzędzia na czas przejazdu; chroni wydruk przed zahaczeniem dyszą.'], { unit: 'mm', step: 0.5, min: 0, def: 0 }),
      N('wipe_distance', 'Wipe (wycieranie)', ['Wipe', 'Po zamkniętym obrysie narzędzie cofa się po ścieżce o tę długość podczas retrakcji — ukrywa kropkę na szwie.'], { unit: 'mm', step: 0.5, min: 0, def: 0, show: v => v.retract_length > 0 }),
      N('coast_length', 'Coasting', ['Coasting', 'Ostatnie milimetry zamkniętego obrysu bez ekstruzji: ciśnienie w dyszy dokańcza ścieżkę, nie ma zgrubienia na szwie.'], { unit: 'mm', step: 0.5, min: 0, def: 0 }),
    ] },
    { id: 'speeds', title: 'Prędkości', fields: [
      N('speed_outer', 'Obrys zewnętrzny', ['Prędkość obrysu zewnętrznego', 'Wolniej = ładniejsza powierzchnia i dokładniejsze wymiary.'], { unit: 'mm/min', step: 100, min: 10, def: 1200 }),
      N('speed_perimeter', 'Obrysy wewnętrzne', ['Prędkość obrysów wewnętrznych', 'Mogą być szybsze niż zewnętrzny.'], { unit: 'mm/min', step: 100, min: 10, def: 1800 }),
      N('speed_infill', 'Wypełnienie', ['Prędkość wypełnienia', 'Rzadkie wypełnienie zwykle najszybciej.'], { unit: 'mm/min', step: 100, min: 10, def: 2400 }),
      N('speed_top', 'Góra / dół', ['Prędkość warstw pełnych', 'Powierzchnie górne i dolne; wolniej = gładsza góra.'], { unit: 'mm/min', step: 100, min: 10, def: 1200 }),
      N('speed_bridge', 'Mosty', ['Prędkość mostów', 'Linie mostów; wolno i równo, żeby nitki się nie zrywały.'], { unit: 'mm/min', step: 100, min: 10, def: 900 }),
      N('speed_overhang', 'Nawisy', ['Prędkość nawisów', 'Docelowa prędkość odcinków w pełni nad powietrzem (interpolowana od prędkości normalnej).'], { unit: 'mm/min', step: 100, min: 10, def: 600 }),
      N('speed_gap', 'Szczeliny', ['Prędkość gap fill', 'Pojedyncze ścieżki w szczelinach i cienkich ścianach.'], { unit: 'mm/min', step: 100, min: 10, def: 1200 }),
      N('speed_support', 'Podpory', ['Prędkość podpór', 'Podpory i warstwy łącznikowe.'], { unit: 'mm/min', step: 100, min: 10, def: 2400 }),
      N('small_perimeter_speed', 'Małe obrysy', ['Prędkość małych obrysów', 'Dla pętli krótszych niż próg „małe obrysy” (sekcja Ściany).'], { unit: 'mm/min', step: 100, min: 10, def: 600 }),
      N('speed_first_layer', 'Pierwsza warstwa', ['Prędkość pierwszej warstwy', 'Wszystkie ścieżki pierwszej warstwy (przyczepność).'], { unit: 'mm/min', step: 100, min: 10, def: 900 }),
      N('speed_travel', 'Przejazdy', ['Prędkość przejazdów', 'Ruchy bez ekstruzji (G0).'], { unit: 'mm/min', step: 500, min: 10, def: 6000 }),
      N('min_layer_time', 'Min. czas warstwy', ['Minimalny czas warstwy', 'Jeśli warstwa trwałaby krócej, wszystkie jej prędkości są skalowane w dół, żeby poprzednia zdążyła zastygnąć (małe wierzchołki). 0 = wyłączone.'], { unit: 's', step: 5, min: 0, def: 0 }),
      N('min_speed', 'Min. prędkość', ['Minimalna prędkość', 'Dolna granica przy spowalnianiu z powodu minimalnego czasu warstwy.'], { unit: 'mm/min', step: 50, min: 10, def: 300, show: v => v.min_layer_time > 0 }),
    ] },
    { id: 'adhesion', title: 'Przyczepność: skirt, brim, raft', fields: [
      N('skirt_loops', 'Skirt: pętle', ['Skirt', 'Luźne pętle wokół modelu na pierwszej warstwie, rozruch ekstrudera (0 = brak).'], { step: 1, min: 0, def: 0 }),
      N('skirt_distance', 'Skirt: odstęp', ['Odstęp skirtu', 'Odległość skirtu od modelu (i brimu).'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.skirt_loops > 0 }),
      N('brim_width', 'Brim: szerokość', ['Brim', 'Obwódka przyklejona do modelu, poprawia przyczepność małych podstaw (0 = brak).'], { unit: 'mm', step: 1, min: 0, def: 0 }),
      N('raft_layers', 'Raft: warstwy', ['Raft', 'Tratwa pod modelem: rzadka warstwa podstawy, warstwy pośrednie i pełna warstwa górna; model stoi na niej zamiast na stole (0 = brak).'], { step: 1, min: 0, max: 5, def: 0 }),
      N('raft_margin', 'Raft: margines', ['Margines raftu', 'O ile raft wystaje poza obrys modelu (i podpór).'], { unit: 'mm', step: 1, min: 0, def: 5, show: v => v.raft_layers > 0 }),
    ] },
    { id: 'multiaxis', title: 'Tryb warstw (robot 6-osiowy)', fields: [
      S('layer_mode', 'Warstwy', ['Tryb warstw', 'Płaskie: klasyczne poziome warstwy. Stożkowe (RotBot): warstwy na stożku, nawisy bez podpór. Konforemne: warstwy kopiują powierzchnię dolną lub górną modelu. Wielokierunkowe: segmenty drukowane w różnych kierunkach (definicja niżej). Podpory, raft, spirala i warstwy adaptacyjne działają tylko dla warstw płaskich.'], [['planar', 'płaskie'], ['conical', 'stożkowe (RotBot)'], ['bottom', 'konforemne do powierzchni dolnej'], ['top', 'konforemne do powierzchni górnej'], ['multi', 'wielokierunkowe (segmenty)']], { def: 'planar' }),
      N('cone_angle', 'Stożek: kąt', ['Kąt stożka', 'Kąt między warstwą a poziomem; robot pochyla narzędzie o tyle samo.'], { unit: '°', step: 5, min: 1, max: 80, def: 30, show: v => v.layer_mode === 'conical' }),
      S('cone_outward', 'Stożek: kierunek', ['Kierunek stożka', 'Na zewnątrz: warstwy opadają od osi (nawisy na zewnątrz). Do środka: wznoszą się od osi (nawisy nad pustką w środku).'], [['true', 'na zewnątrz'], ['false', 'do środka']], { def: 'true', show: v => v.layer_mode === 'conical' }),
      N('cone_cx', 'Oś stożka X', ['Oś stożka', 'Punkt X, Y (układ modelu po wyśrodkowaniu), przez który przechodzi oś stożka. Puste = środek modelu.'], { unit: 'mm', step: 5, def: '', placeholder: 'auto', show: v => v.layer_mode === 'conical' }),
      N('cone_cy', 'Oś stożka Y', ['Oś stożka Y', 'Jak wyżej, współrzędna Y. Puste = środek.'], { unit: 'mm', step: 5, def: '', placeholder: 'auto', show: v => v.layer_mode === 'conical' }),
      N('hf_cell', 'Pole wysokości: komórka', ['Komórka pola wysokości', 'Rozdzielczość siatki próbkującej powierzchnię (mniejsza = dokładniej, wolniej).'], { unit: 'mm', step: 1, min: 0.5, def: 5, show: v => v.layer_mode === 'bottom' || v.layer_mode === 'top' }),
      N('max_slope', 'Maks. nachylenie warstw', ['Maksymalne nachylenie', 'Ogranicza pochylenie warstw konforemnych (a więc narzędzia), żeby robot nie wchodził w nieosiągalne orientacje.'], { unit: '°', step: 5, min: 1, max: 85, def: 45, show: v => v.layer_mode === 'bottom' || v.layer_mode === 'top' }),
      T('segments', 'Segmenty', ['Segmenty', 'Jedna definicja na linię. „segment NAZWA dx dy dz” zaczyna segment drukowany w kierunku (dx, dy, dz); kolejne „plane px py pz nx ny nz” to płaszczyzny tnące — do segmentu należy strona wskazana przez normalną. Segmenty drukowane są po kolei.'], { def: '', rows: 5, placeholder: 'segment korpus 0 0 1\nplane 0 0 40 0 0 -1\nsegment ramie 1 0 0\nplane 0 0 40 0 0 1', show: v => v.layer_mode === 'multi', extra: true }),
      C('tool_axis', 'Zapisuj oś narzędzia I J K', ['Oś narzędzia', 'Przy każdym punkcie G-code zapisywany jest wektor osi narzędzia (I J K); translator ustawia z niego orientację robota. Bez tego ścieżka jest tłumaczona z narzędziem pionowym.'], { def: true }),
    ] },
    { id: 'gcode', title: 'G-code i inne', fields: [
      T('start_gcode', 'G-code startowy', ['G-code startowy', 'Linie wstawiane na początku pliku, po G21/G90/M82/G92 (np. własne M-kody; translator przenosi nieznane do komentarzy).'], { def: '', rows: 3, placeholder: '; np. M104 S210' }),
      T('end_gcode', 'G-code końcowy', ['G-code końcowy', 'Linie wstawiane na końcu pliku, przed M30.'], { def: '', rows: 3, placeholder: '; np. M104 S0' }),
      C('comments', 'Komentarze ;TYPE / ;LAYER', ['Komentarze', 'Zapisuje ;LAYER, ;Z, ;HEIGHT i ;TYPE przed każdą ścieżką (czytelność, filtry w podglądzie).'], { def: true }),
      C('origin_center', 'Wyśrodkuj model w XY', ['Wyśrodkowanie', 'Przesuwa model tak, aby środek jego obrysu XY był w (0, 0) bazy; wyłącz, żeby zachować współrzędne z pliku.'], { def: true }),
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
      if (!(k in saved)) continue;
      const val = saved[k];
      if (el.type === 'checkbox') el.checked = !!val;
      else if (el.tagName === 'SELECT') el.value = String(val);
      else el.value = val === null || val === undefined ? '' : val;
    }
  }
  function reset() {
    for (const [k, el] of Object.entries(inputs)) {
      const f = fieldDefs[k];
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

  window.SLICER_FORM = { params, extras, reset, help: HELP, groups: GROUPS, inputs };
})();
