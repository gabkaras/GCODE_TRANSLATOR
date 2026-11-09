'use strict';
// Parameter help: a "?" next to each option that explains what it does.
// Keys are input ids (the help lands next to that row's label) or CSS selectors
// (the help lands inside the matched element, e.g. a section header).
(() => {
  const HELP = {
    // ---- arc fitting ----------------------------------------------------
    opt_tol: ['Tolerancja', 'Największa dopuszczalna odległość między oryginalną łamaną a dopasowanym łukiem lub odcinkiem. Mniejsza = wierniej, ale więcej ruchów KRL. Kafelek „Maks. odchyłka” pokazuje faktyczny wynik.'],
    opt_max_radius: ['Maks. promień', 'Łuki o większym promieniu są zapisywane jako odcinki SLIN — bardzo duże promienie są numerycznie niepewne i nie dają już oszczędności.'],
    opt_min_radius: ['Min. promień', 'Łuki mniejsze niż ta wartość nie są odtwarzane (np. zaokrąglenia naroży z CAM, które KRC4 i tak wygładzi przez C_DIS).'],
    opt_min_arc_points: ['Min. punktów na łuk', 'Ile kolejnych wierzchołków łamanej musi leżeć na okręgu, żeby uznać je za łuk. Większa wartość = mniej fałszywych łuków na prostych z szumem.'],
    opt_refine_shift: ['Korekta podziałów', 'Po wstępnym podziale punkty graniczne między elementami są przesuwane o ± tyle wierzchołków w poszukiwaniu mniejszego błędu dopasowania.'],
    opt_corner_angle: ['Kąt naroża', 'Zmiana kierunku większa niż ten kąt jest traktowana jako ostre naroże: tu łuk nigdy nie przechodzi, a blok SPLINE jest przerywany, żeby robot nie zaokrąglał naroża.'],
    opt_spl_max_segment: ['SPL: maks. segment', 'Fragmenty, których nie da się opisać łukiem, idą jako punkty SPL w bloku SPLINE. Dłuższe odcinki są dzielone tak, by żaden segment SPL nie przekraczał tej długości.'],
    opt_fit_arcs: ['Odtwarzaj łuki', 'Włącza rozpoznawanie łuków w ciągach G1 i zapis jako SCIRC / CIRC. Wyłączone = każdy odcinek zostaje odcinkiem.'],
    opt_use_spl: ['Mikrosegmenty jako SPL', 'Krótkie odcinki o zmiennej krzywiźnie trafiają do punktów SPL (spline) zamiast setek SLIN. Daje płynny ruch i mniejszy program.'],
    // ---- KRL program ----------------------------------------------------
    opt_mode: ['Tryb ruchów', 'SPLINE: bloki SPLINE z SLIN / SCIRC / SPL (zalecane na KRC4, najpłynniejszy ruch). LIN/CIRC: klasyczne instrukcje z C_DIS dla starszych sterowników lub prostszych programów.'],
    opt_name: ['Nazwa programu', 'Nazwa plików .src / .dat i procedury głównej w KRL (DEF NAZWA()). Puste = z nazwy pliku wejściowego.'],
    opt_tool: ['TOOL_DATA[n]', 'Numer narzędzia w $CONFIG.DAT. Program ustawia $TOOL = TOOL_DATA[n]; wartości z sekcji „Stanowisko” trafiają tam, jeśli włączysz zapis literalny.'],
    opt_base: ['BASE_DATA[n]', 'Numer bazy w $CONFIG.DAT. Program ustawia $BASE = BASE_DATA[n]; współrzędne G-code są interpretowane w tej bazie.'],
    opt_abc: ['Orientacja A B C', 'Stała orientacja narzędzia (kąty KUKA A, B, C w stopniach) dla ścieżek bez własnej orientacji. 0 180 0 = narzędzie pionowo w dół w typowej bazie na stole.'],
    opt_offset: ['Przesunięcie', 'Wektor dodawany do wszystkich współrzędnych G-code przed zapisem (np. przesunięcie zera programu w bazie).'],
    opt_start: ['Pozycja startowa', 'Punkt, do którego robot dojeżdża PTP/LIN przed pierwszym ruchem ścieżki (bezpieczna pozycja nad detalem). Puste = pierwszy punkt ścieżki.'],
    // ---- cell -----------------------------------------------------------
    '#cell-setup .fb-head:nth-of-type(1) > span': ['TCP · $TOOL', 'Ramka narzędzia względem kołnierza: X Y Z końcówki w mm oraz A B C orientacji osi dyszy (Z narzędzia). Dysza nie musi leżeć w osi A6: ekstruder pochylony 45°, boczny 90° lub przesunięty opisuje się tu wprost (presety „Montaż dyszy”). Kinematyka, zasięg, kolizje (uchwyt do początku dyszy, dysza wzdłuż własnej osi), podgląd 3D i solver obrotu liczą z pełnej ramki. W podglądzie 3D: uchwyt TCP przesuwa X Y Z, uchwyt na końcu osi dyszy (niebieska oś Z narzędzia) pochyla narzędzie (A B C), uchwyt na osi X obraca je wokół dyszy; kąty przyciągane co 5° przy włączonym snapie. Zapisane narzędzia są na liście.'],
    '#cell-setup .fb-head:nth-of-type(2) > span': ['Podstawa do druku · $BASE', 'Położenie bazy stołu względem podstawy robota ($ROBROOT): X Y Z w mm, A B C w stopniach. Punkt (0,0,0) bazy to narożnik blatu, a osie +X i +Y bazy wskazują na blat — współrzędne ścieżki i modelu są liczone od tego narożnika. Można podać wprost, wyznaczyć metodą 3-punktową jak w KSS albo przeciągnąć w 3D.'],
    '#cell-setup .fb-head:nth-of-type(3) > span': ['Wymiary podstawy i materiał', 'Wymiary podstawy do druku i blok materiału; z nich automatycznie powstają przeszkody do kontroli kolizji: płyta podstawy pod bazą i blok materiału (grubość + zapas wokół ścieżki). Wyłącz, jeśli opisujesz stanowisko ręcznie w „Przeszkody”.'],
    opt_zones: ['Strefy bezpieczeństwa', 'Jedna strefa na linię: zone NAZWA allowed|forbidden tcp|robot robot|base cx cy cz sx sy sz [a b c]. Dozwolona (allowed): punkt roboczy (tcp) albo wszystkie człony robota (robot) muszą zostać w pudełku — np. przestrzeń nad podstawą do druku. Zabroniona (forbidden): nie wolno do niego wejść — np. stanowisko operatora, szafa sterownicza. Środek, wymiary i obrót w układzie robota albo bazy. Naruszenia na ścieżce i na ruchach HOME są liczone, zaznaczane w 3D (fiolet) i trafiają do ostrzeżeń.'],
    opt_zones_krl: ['$WORKSPACE w KRL', 'Zapisuje strefy jako przestrzenie robocze KSS: $WORKSPACE[1..8] z BASE = ramka strefy, X1…Z2 = ± pół wymiaru, MODE #OUTSIDE_STOP dla dozwolonej (stop, gdy TCP wyjdzie) i #INSIDE_STOP dla zabronionej (stop, gdy TCP wejdzie). Sterownik pilnuje tylko TCP; dla zakresu „robot” pełną kontrolę daje raport z tej aplikacji.'],
    tbl_sx: ['Podstawa', 'Wymiary podstawy do druku w mm (wzdłuż X i Y bazy). Podstawa jest przeszkodą; jej górna powierzchnia leży na Z = −grubość materiału.'],
    '#plater-head > span': ['Modele na stole', 'Każdy wgrany model stoi na stole jak w zwykłym slicerze: X Y to środek jego obrysu w bazie stołu (narożnik blatu = 0 0), obroty X Y Z działają wokół osi stołu w tej kolejności, lustro odbija bryłę, skala jest jednorodna. Po zmianie model jest stawiany na Z = 0. Przeciągnij uchwyt nad modelem w podglądzie 3D albo wpisz wartości; „ułóż płasko” i „min. nawisy” liczą orientację automatycznie. Krojone są wszystkie modele: warstwa po warstwie albo obiekt po obiekcie (sekcja „Modele na stole: kolejność”).'],
    '#sec-sim > summary': ['Cięcie: symulacja', 'Podgląd pokrojonej i przetłumaczonej ścieżki: suwak „warstwa” odsłania ścieżkę do wybranej wysokości, ▶ odtwarza ruch robota po ścieżce (ścieżka nad dyszą jest ukryta, jak postęp druku). Zakładki „Kolizje” i „Ostrzeżenia” nad widokiem pokazują raport zasięgu, limitów osi, kolizji i stref; sekcja „Kontrola robota” niżej steruje tym raportem.'],
    tbl_cx: ['Środek blatu', 'Wyliczany z wymiarów: początek bazy leży w narożniku blatu, więc środek to (dł./2, szer./2). Tu trafia model przy krojeniu.'],
    mat_t: ['Materiał', 'Grubość: wysokość bloku materiału (detalu) pod ścieżką; górna powierzchnia na Z = 0 bazy. Zapas: o ile blok wystaje poza obrys ścieżki z każdej strony.'],
    'step-deflection': ['Tesselacja STEP', 'Dokładność zamiany powierzchni STEP/IGES na trójkąty (maks. odchyłka). Mniejsza wartość = gładszy model, więcej trójkątów, wolniejsze liczenie.'],
    // ---- robot ----------------------------------------------------------
    opt_robot: ['Model robota', 'Kinematyka, limity i prędkości osi oraz geometria kolizyjna z oficjalnych danych KUKA. Używane do kontroli zasięgu, limitów osi i kolizji, a także jako domyślna pozycja HOME.'],
    opt_base_literal: ['$BASE literalnie', 'Zamiast BASE_DATA[n] program wpisuje $BASE = {X …, Y …, …} z wartości z sekcji „Stanowisko”. Program jest wtedy niezależny od $CONFIG.DAT.'],
    opt_tool_literal: ['$TOOL literalnie', 'Zamiast TOOL_DATA[n] program wpisuje $TOOL = {X …, …} z sekcji „Stanowisko” (TCP).'],
    opt_path_step: ['Krok próbkowania', 'Co ile mm ścieżka jest próbkowana do kontroli zasięgu, limitów osi i kolizji. Mniejszy krok = dokładniej, wolniej (liczba próbek jest ograniczona polem „Próbki IK”).'],
    opt_orient_solve: ['Dobór obrotu wokół osi narzędzia', 'Dla ścieżek z osią I J K wolny obrót wokół osi narzędzia jest dobierany tak, by robot trzymał się z dala od osobliwości nadgarstka (A5 ≈ 0) i ruszał osiami jak najmniej. Wyłączone = obrót z orientacji odniesienia.'],
    opt_spin_step: ['Parametry solvera orientacji', 'Krok obrotu: co ile stopni próbowane są obroty wokół osi narzędzia. Min |A5|: minimalny kąt osi A5, poniżej którego konfiguracja jest uznana za osobliwą. Dod. pochylenie: o ile stopni wolno odchylić narzędzie od zadanej osi, jeśli bez tego nie da się uniknąć osobliwości (0 = nigdy).'],
    opt_spin_mode: ['Obrót wokół osi dyszy', 'Przy ścieżce z osią I J K obrót narzędzia wokół własnej osi jest wolny. Solver: wybiera go tak, by unikać osobliwości i ruszać osiami jak najmniej. Stały kąt: trzyma zadany obrót (np. przewody ekstrudera zawsze z tyłu), odstępuje tylko gdy robot nie sięga lub wchodzi w osobliwość. X narzędzia wzdłuż ścieżki: oś X ramki $TOOL podąża za kierunkiem ruchu plus przesunięcie — dla dysz asymetrycznych, podajników drutu, głowic z kablem.'],
    opt_spin_fixed: ['Kąt obrotu', 'Stały kąt: obrót wokół osi dyszy względem orientacji odniesienia A B C. Przesunięcie: dodawane do kierunku ścieżki w trybie „wzdłuż ścieżki” (180° = X narzędzia do tyłu, czyli element ciągnięty).'],
    opt_printed_check: ['Kolizje z wydrukiem', 'Druk planarny: materiał już położony wypełnia obrys części do wysokości bieżącej warstwy. Każda kapsuła uchwytu narzędzia lub członu robota (poza samą dyszą), która schodzi poniżej tego poziomu wewnątrz obrysu, to kolizja z wydrukiem — typowo przy pochylonej dyszy, montażu pod kątem albo szerokim korpusie ekstrudera. Obrys = otoczka wypukła ścieżki plus margines.'],
    opt_printed_margin: ['Margines wydruku', 'O ile poszerzyć obrys części przy kontroli kolizji z wydrukiem (rozlewanie materiału, niepewność).'],
    opt_auto_spin: ['Auto obrót narzędzia', 'Przy stałej orientacji (ścieżka bez I J K) obrót narzędzia wokół osi dyszy — kąt A — jest wolny. Włączony: sprawdzane są kandydaci co podany krok (zasięg, kolizje z przeszkodami, z wydrukiem, prześwit) i wybierany najlepszy; A trafia do programu i do raportu. Pozwala odsunąć korpus ekstrudera od ściany, stołu czy sąsiednich obiektów. Ścieżki z osią I J K używają solvera obrotu z sekcji wyżej.'],
    opt_status_turn: ['Status i Turn', 'Z kinematyki odwrotnej liczone są bity Status (nadgarstek za A1, A3 względem wyprostowanego ramienia, znak A5) i Turn (ujemne osie). Pierwszy dojazd do ścieżki i każdy przejazd ze zmianą Statusu to PTP {…, S, T}, więc ruchy ciągłe trzymają zaplanowaną, nieosobliwą konfigurację. Wyłączone = same LIN bez S/T (sterownik dobiera konfigurację sam).'],
    opt_max_refine: ['Rundy zagęszczania', 'Kontrola ciągła kolizji: ile razy odcinki między próbkami są dzielone na pół (kinematyka w punkcie środkowym), gdy ruch członów mógłby schować kolizję między próbkami. Więcej = pewniejszy dowód, wolniej. 0 = tylko próbki.'],
    opt_robot_samples: ['Próbki IK', 'Górna granica liczby punktów ścieżki, w których liczona jest kinematyka odwrotna. Chroni przed długim liczeniem bardzo gęstych ścieżek.'],
    opt_margin: ['Wymagany prześwit', 'Minimalna odległość między członami robota / narzędziem a przeszkodami i między członami wzajemnie. Mniejsza odległość = ostrzeżenie „poniżej prześwitu”, przecięcie = kolizja.'],
    opt_tool_radius: ['Promień uchwytu', 'Promień walca reprezentującego uchwyt / ekstruder od kołnierza do ostrza, gdy nie ma wgranego modelu narzędzia.'],
    opt_cutter_len: ['Ostrze', 'Długość i promień samej części roboczej (frez, dysza) na końcu uchwytu. Ostrze może wchodzić w materiał (detal) bez zgłaszania kolizji — uchwyt nie.'],
    opt_obstacles: ['Przeszkody', 'Lista brył do kontroli kolizji, jedna na linię: box, cylinder, sphere, floor, capsule albo mesh (wgrany model). Współrzędne w układzie robota lub bazy. Składnia jest opisana pod polem; „#” zaczyna komentarz.'],
    // ---- speeds ---------------------------------------------------------
    opt_default_feed: ['Posuw domyślny', 'Prędkość TCP w mm/s dla ruchów G1 bez słowa F (G-code z CAM bez posuwu). Ścieżki ze slicera zawsze mają własne prędkości.'],
    opt_rapid_vel: ['G0: $VEL.CP', 'Prędkość ruchów szybkich G0 zapisanych jako SLIN / LIN, w m/s.'],
    opt_max_vel: ['Maks. $VEL.CP', 'Górne ograniczenie prędkości z posuwów F (przeliczone z mm/min na m/s). Chroni przed zbyt dużymi F z CAM.'],
    opt_acc: ['$ACC.CP', 'Przyspieszenie ruchów kartezjańskich w m/s² ustawiane na początku programu.'],
    opt_cdis: ['$APO.CDIS', 'Odległość zaokrąglania (approximation) dla LIN / CIRC z C_DIS oraz dla bloków SPLINE: robot może odchodzić od narożnika o tyle mm, żeby się nie zatrzymywać.'],
    opt_advance: ['$ADVANCE', 'Ile instrukcji naprzód sterownik planuje ruch (1–5). Większa wartość = płynniejsze zaokrąglanie, ale instrukcje I/O wykonują się wcześniej względem ruchu.'],
    opt_max_block_segments: ['Maks. segmentów w bloku', 'Blok SPLINE jest dzielony po tylu segmentach — KRC4 ma limit długości bloku, a krótsze bloki szybciej się planują.'],
    opt_max_motions_per_file: ['Maks. ruchów na plik', 'Długie programy są dzielone na podprogramy (PROG_1.src, PROG_2.src…) wywoływane z programu głównego, żeby nie przekraczać limitów pamięci edytora KSS.'],
    opt_block_approx: ['ENDSPLINE C_SPL', 'Zaokrągla przejście między kolejnymi blokami SPLINE (bez zatrzymania). Wyłącz, jeśli robot ma dokładnie zatrzymać się na końcu każdego bloku.'],
    opt_round_corners: ['LIN: C_DIS', 'W trybie LIN/CIRC dodaje C_DIS do ruchów, więc robot zaokrągla naroża zamiast zatrzymywać się w każdym punkcie.'],
    opt_bco: ['BCO na starcie', 'Pierwszy ruch to PTP $POS_ACT — spełnia wymóg KSS dojazdu do pozycji (BCO) przy starcie programu z dowolnego miejsca.'],
    // ---- M-codes --------------------------------------------------------
    opt_spindle_out: ['Wrzeciono $OUT[n]', 'Wyjście cyfrowe załączane przy M3/M4 i wyłączane przy M5. 0 = tylko komentarz w KRL.'],
    opt_spindle_anout: ['Obroty $ANOUT[n]', 'Wyjście analogowe, na które trafia prędkość obrotowa S z G-code, przeskalowana do 0…1 według pola niżej.'],
    opt_spindle_max_rpm: ['Obroty dla 1.0', 'Prędkość obrotowa odpowiadająca pełnemu wysterowaniu wyjścia analogowego (1.0 = 10 V).'],
    opt_coolant_out: ['Chłodziwo $OUT[n]', 'Wyjście cyfrowe dla M7/M8 (włącz) i M9 (wyłącz).'],
    opt_halt_tool_change: ['HALT przy M6', 'Przy zmianie narzędzia M6 program zatrzymuje się (HALT) i czeka na operatora.'],
    opt_extruder: ['Sterowanie ekstruderem', 'Na ruchach z materiałem program KRL włącza wyjścia ekstrudera, na przejazdach je wyłącza; cofnięcia i dopuszczenia ze slicera stają się impulsem rewersu i WAIT SEC. Slicer włącza tę opcję automatycznie.'],
    opt_extruder_out: ['Ekstruder $OUT[n]', 'Wyjście cyfrowe uruchamiające ekstruder (zezwolenie / start silnika). 0 = tylko komentarze w KRL, gdy ekstruder jest sterowany z zewnątrz. Przy wydatku analogowym to wyjście bramkuje sygnał na przejazdach.'],
    opt_extruder_analog: ['Wydatek analogowo', 'Prędkość ekstrudera proporcjonalna do faktycznej prędkości TCP: ANOUT ON $ANOUT[n] = k × $VEL_ACT. Robot zwalnia w narożach i na nawisach, a ekstruder razem z nim — stała szerokość ścieżki. k wynika z mm³/mm ścieżki i wydajności przy 1.0.'],
    opt_extruder_anout: ['Parametry wydatku analogowego', '$ANOUT[n]: numer wyjścia analogowego (0…1 = 0…10 V). mm³/s przy 1.0: wydajność ekstrudera przy pełnym wysterowaniu — skaluje współczynnik k i czasy cofnięcia / dopuszczenia. Opóźnienie: przesunięcie sygnału w czasie względem ruchu (ANOUT DELAY, ujemne = wcześniej), kompensuje bezwładność ekstrudera.'],
    opt_extruder_trigger: ['Przełączanie przez TRIGGER', 'Włączenie wyjścia jako TRIGGER WHEN DISTANCE=0 przed blokiem SPLINE (start ekstruzji), wyłączenie jako TRIGGER WHEN PATH=0 (koniec bloku). Sterownik wykonuje je w locie, bez zatrzymania wyprzedzania ($ADVANCE) — ruch pozostaje ciągły. Wyłączone = zwykłe $OUT[n] = TRUE/FALSE, które zatrzymują robota na każdym przełączeniu.'],
    opt_extruder_on_delay: ['Opóźnienia włączenia i wyłączenia', 'DELAY triggerów w ms. Włączenie: ujemne = ekstruder rusza przed startem ruchu (kompensacja rozruchu ślimaka), wyłączenie: dodatnie = wyjście gaśnie po końcu ścieżki. Dobierz tak, by początek i koniec ścieżki miały pełną szerokość.'],
    opt_extruder_reverse_out: ['Wyjście rewersu', 'Wyjście cyfrowe, które cofa ekstruder (bieg wsteczny). Cofnięcie ze slicera (mm³) zamienia się w impuls: $OUT=TRUE, WAIT SEC t, $OUT=FALSE, gdzie t = mm³ / wydajność przy 1.0. 0 = tylko komentarz.'],
    opt_home_start: ['HOME na początku', 'Program zaczyna się od PTP do pozycji HOME — standard KUKA: pierwszy ruch jest zarazem dojazdem BCO. Ruch HOME → pierwszy punkt ścieżki to LIN z prędkością przejazdów; oba są sprawdzane pod kątem kolizji (interpolacja w osiach dla PTP).'],
    opt_home_end: ['HOME na końcu', 'Po ostatnim ruchu i wyłączeniu ekstrudera robot wraca PTP do HOME; ruch jest sprawdzany pod kątem kolizji.'],
    opt_home_mode: ['Definicja HOME', 'XHOME: zmienna HOME z $CONFIG.DAT sterownika (PTP XHOME). Pozycja osi: literalna pozycja A1…A6 wpisana niżej — program nie zależy od konfiguracji sterownika.'],
    opt_home_axes: ['Osie HOME', 'Kąty A1…A6 w stopniach dla trybu „pozycja osi” i do kontroli kolizji ruchów HOME. Puste = HOME z modelu robota (dla KUKA zwykle A2 −90°, A3 90°, reszta 0).'],
    opt_home_vel: ['Prędkość PTP HOME', 'Procent prędkości osi dla ruchów HOME (BAS #VEL_PTP).'],
    opt_krl_start: ['Własne linie KRL na starcie', 'Wstawiane po ustawieniach ($TOOL, $BASE, $VEL.CP, $ADVANCE), przed pierwszym ruchem — np. włączenie grzałek, podajnika, czekanie na gotowość ekstrudera (WAIT FOR $IN[n]).'],
    opt_krl_end: ['Własne linie KRL na końcu', 'Wstawiane po ostatnim ruchu i wyłączeniu ekstrudera, przed END — np. wyłączenie grzałek, odjazd do pozycji serwisowej.'],
    opt_dwell_ms: ['G4 P w ms', 'Interpretacja G4 P: zaznaczone = milisekundy (Marlin, drukarki), odznaczone = sekundy (Fanuc / większość CAM).'],
  };

  let pop = null;
  function closePop() { if (pop) { pop.remove(); pop = null; } }
  function showPop(btn, title, text) {
    const same = pop && pop.dataset.for === btn.dataset.help;
    closePop();
    if (same) return;
    pop = document.createElement('div');
    pop.className = 'help-pop'; pop.dataset.for = btn.dataset.help;
    const b = document.createElement('b'); b.textContent = title;
    const p = document.createElement('div'); p.textContent = text;
    pop.append(b, p);
    document.body.appendChild(pop);
    const r = btn.getBoundingClientRect();
    const w = Math.min(340, window.innerWidth - 24);
    pop.style.width = w + 'px';
    let left = r.right + 8; if (left + w > window.innerWidth - 8) left = Math.max(8, r.left - w - 8);
    let top = r.top - 6; if (top + pop.offsetHeight > window.innerHeight - 8) top = Math.max(8, window.innerHeight - pop.offsetHeight - 8);
    pop.style.left = left + 'px'; pop.style.top = top + 'px';
  }
  document.addEventListener('click', e => { if (pop && !pop.contains(e.target) && !e.target.classList.contains('help')) closePop(); });
  document.addEventListener('keydown', e => { if (e.key === 'Escape') closePop(); });
  document.addEventListener('scroll', closePop, true);

  let n = 0;
  const ALL = Object.assign({}, window.SLICER_FORM ? window.SLICER_FORM.help : {}, HELP);
  for (const [key, [title, text]] of Object.entries(ALL)) {
    let host;
    if (/^[#.]/.test(key)) host = document.querySelector(key);
    else {
      const el = document.getElementById(key); if (!el) continue;
      const row = el.closest('.row, .col, .fb-row');
      host = row ? row.querySelector(':scope > span') : null;
      if (!host && row) host = row;
    }
    if (!host || host.querySelector(':scope > .help')) continue;
    const btn = document.createElement('button');
    btn.type = 'button'; btn.className = 'help'; btn.textContent = '?'; btn.title = text;
    btn.dataset.help = String(++n); btn.setAttribute('aria-label', 'pomoc: ' + title);
    btn.addEventListener('click', e => { e.preventDefault(); e.stopPropagation(); showPop(btn, title, text); });
    host.appendChild(btn);
  }
})();
