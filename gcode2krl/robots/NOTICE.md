# Pochodzenie danych robotów

* Siatki STL w `meshes/` oraz origins stawów pochodzą z pakietów
  [ROS-Industrial kuka_experimental](https://github.com/ros-industrial/kuka_experimental)
  (licencja Apache-2.0, pliki konwertowane z danych CAD KUKA; patrz LICENSE
  w tym katalogu). Pliki w `meshes/` są kopiami STL kolizyjnych z tego
  repozytorium w milimetrach; `kr90r2700pro/link_3.stl` jest przedramieniem
  KR 120 R2500 pro wydłużonym o 200 mm.
* Dane osi (zakresy, prędkości), udźwig, zasięg i masa: karty katalogowe KUKA
  wskazane w polu `sources` każdego pliku JSON. Pola oznaczone jako "URDF"
  pochodzą z pakietu ROS-Industrial i należy je sprawdzić z kartą katalogową
  danej wersji robota.
* Importer `gcode2krl-import` pozwala wczytać własne dane: URDF/xacro i STL
  wyeksportowane z oficjalnych modeli CAD KUKA (Download Center) albo z KUKA.Sim.
