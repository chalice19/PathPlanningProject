# PathPlanning Project SIPP

![comics](./Images/comics.png)

В данном проекте реализован алгоритм [SIPP (Safe Interval Path
Planning)](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf) для планирования траектории агента (автономно движущегося объекта) в динамическом окружении, то есть при наличии передвигающихся препятствий с известными заранее траекториями. На вход принимается карта, координаты начальной и конечной точки, между которыми нужно найти самый быстрый маршрут. Имеются статические препятствия и динамические. Статические отмечены на карте, для динамических требуется указать маршрут - список координат с пометкой времени. В ходе движения по построенной траектории агент не должен оказаться в одной клетке ни с каким из препятствий, также не должно происходить смен местами с препятствиями.

Была проведена серия экспериментов и созданы визуализации работы алгоритма. Визуализации можно найти в `Experiments/*/animation.gif`. На них пустые клетки карты обозначаются `-`, статические препятствия --- `+`, динамические --- `x`, агент выделен красным квадратиком. При отрисовывании карты на каждом шагу агент имеет меньший приоритет по сравнению с препятствиями. Это значит, что если красный квадратик виден, то в этой клетке нет других препятствий.

## Входные и выходные данные

Входные данные принимаются в файле формата .xml (см. `Examples/`). В нём содержатся:
* `<map />` прямоугольная карта
    - `<{weight|height} />` ширина и высота карты
    - `<cellsize />` размер клеки карты
    - `<{start|finish}{x|y} />` стартовые и конечные координаты
    - `<grid />` сетка карты, состоит из 0 или 1 — имеется ли препятствие в клетке
* `<algorithm />` опции поиска:
    - `<searchtype />` алгоритм: Dijkstra, A* или SIPP
    - `<metrictype />` тип метрики для подсчета оценки расстояния до конечной точки
    - `<breakingties />` порядок раскрытия вершин при равенстве f-значения
    - опции движения:
        - `<allowdiagonal />` разрешено ли ходить по диагонали
        - `<cutcorners />` если разрешено по диагонали, можно ли ходить по диагонали рядом с препят-
ствием
        - `<allowsqueeze />` если разрешено по диагонали рядом с препятствием, можно ли идти, когда с
обеих сторон препятствие
* `<options />` дополнительные параметры логирования
* `<dynamicobstacles />` количество и траектории динамических препятствий


Выходные данные также записываются в xml-файл. В нём содержатся все данные из
входного файла и информация о результате работы алгоритма:
* `<mapfilename />` входной xml-файл 
* `<summary />` найден ли путь, если да, то его длина, отмасштабированная длина, время, которое работал алгоритм, число итераций, количество созданных вершин
* `<path />` найденный путь на карте (`*` означает путь; число, большее 1 означает ожидание в течение указанного количества шагов)
* `<lplevel />` список вершин, входящих в найденный путь
* `<hplevel />` список ключевых точек найденного пути (начало-конец прямого участка)

## Требования к ПО

### Linux
- CMake 3.2 или выше;
- GCC 4.9 или выше;
- Make

### Mac
- CMake 3.2 или выше;
- Apple LLVM version 10.0.0 (clang-1000.11.45.5) или выше;
- Make

### Windows
- CMake 3.2 или выше;
- MinGW-w64 5.0.3 или выше (должен быть добавлен в переменную среды Path);


### Сборка и запуск

При использовании CMake сборка и запуск может производиться как из командной строки, так и при помощи различных IDE (например JetBrains CLion). Ниже приведены скрипты сборки и запуска с использованием командной строки.

### Linux и Mac
Release сборка:
```bash
cd PathPlanningProject
cd Build
cd Release
cmake ../../ -DCMAKE_BUILD_TYPE="Release"
make
make install
```

Debug сборка:
```bash
cd PathPlanningProject
cd Build
cd Debug
cmake ../../ -DCMAKE_BUILD_TYPE="Debug"
make
make install
```

Запуск:
```bash
cd ../../Bin/{Debug|Release}/
./Alexandra_Pilipyuk_SIPP ../../Experiments/RealMap/real_map.xml
```

### Windows
Release сборка:
```cmd
cd PathPlanningProject
cd Build
cd Release
set PATH
cmake ../../ -DCMAKE_BUILD_TYPE="Release" -G "MinGW Makefiles"
mingw32-make
mingw32-make install
```

Debug сборка:
```cmd
cd PathPlanningProject
cd Build
cd Debug
set PATH
cmake ../../ -DCMAKE_BUILD_TYPE="Debug" -G "MinGW Makefiles"
mingw32-make
mingw32-make install
```

Запуск:
```cmd
cd ../../Bin/{Debug|Release}/
Alexandra_Pilipyuk_SIPP.exe ../../Experiments/RealMap/real_map.xml
```

### Пример результата запуска

Для карты `Experiments/RealMap/real_map.xml`. Результат будет сохранён в `Experiments/RealMap/real_map_log.xml`

```
../../Experiments/RealMap/real_map.xml
Parsing the map from XML:
Map OK!
Parsing configurations (algorithm, log) from XML:
Value of 'searchtype' tag (algorithm name) is sipp
short
Warning! Value of 'logpath' tag is missing!
Value of 'logpath' tag was defined to 'current directory'.
Warning! Value of 'logfilename' tag is missing.
Value of 'logfilename' tag was defined to default (original filename +'_log' + original file extension.
Configurations OK!
Creating log channel:
Log OK!
Start searching the path:
Search is finished!
Path found!
numberofsteps=3665
nodescreated=7015
pathlength=141
pathlength_scaled=141
time=0.013489
Results are saved (if chosen) via created log channel.
```

## Контакты

Работу выполнила: **Пилипюк Александра Сергеевна**
- aspilipyuk@edu.hse.ru

Руководитель КР: **Яковлев Константин Сергеевич**
- kyakovlev@hse.ru
- [Сайт НИУ ВШЭ](https://www.hse.ru/staff/yakovlev-ks)
- Telegram: @KonstantinYakovlev
