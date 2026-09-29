# transport_detection

Обнаружение посторонних объектов в габарите беспилотного поезда метро по данным 3D-лидара
Hesai Pandar128E3X. Решение читает поток облаков точек из ROS 2, строит путь и габарит поезда
впереди и сообщает, есть ли на пути препятствие и на каком расстоянии.

```
тоннель → облако точек → ось пути и габарит → объекты в габарите → подтверждение по кадрам → расстояние
```

## Быстрый старт

Нужен Docker. Данные подключаются как том.

```bash
docker build -t transport_detection .
```

Запуск детектора и проигрывание записи в одном контейнере:

```bash
docker run --rm -it -v /path/to/bags:/data transport_detection play /data/doubleT_obstacle
```

В консоли появятся сообщения о найденных препятствиях, результат каждого кадра публикуется в топики.

Только детектор, запись проигрывается отдельно (в этом же контейнере или на хосте в той же сети ROS):

```bash
docker run --rm -it --net=host -v /path/to/bags:/data --name det transport_detection detector
docker exec -it det bash -c "source /opt/ros/humble/setup.bash && ros2 bag play /data/doubleT_obstacle"
```

Прогон записи без ROS, результат по кадрам в JSON Lines:

```bash
docker run --rm -v /path/to/bags:/data -v $(pwd)/results:/out transport_detection \
  offline /data/doubleT_obstacle --out /out/doubleT_obstacle.jsonl
```

Визуализация в RViz (нужен доступ к X-серверу):

```bash
xhost +local:docker
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/bags:/data transport_detection play /data/doubleT_obstacle rviz:=true
```

В конфиге RViz выбраны топик `/lidar_points` и система координат `hesai_lidar`, как в большинстве записей.
В записи `doubleT_obstacle` облако приходит в топик `/sensing/lidar/hesai128/pointcloud` с системой
координат `lidar_livox`: их можно выбрать в RViz в полях Topic и Fixed Frame.

## Выход

| Топик | Тип | Что внутри |
|---|---|---|
| `/obstacle/status` | `std_msgs/String` | JSON: есть ли препятствие, расстояние, список объектов, дальность прослеженного пути, время обработки |
| `/obstacle/detected` | `std_msgs/Bool` | есть ли препятствие |
| `/obstacle/distance` | `std_msgs/Float32` | расстояние до ближайшего препятствия, м, или -1 |
| `/obstacle/markers` | `visualization_msgs/MarkerArray` | границы габарита и рамки объектов с подписью расстояния |
| `/obstacle/points` | `sensor_msgs/PointCloud2` | точки внутри габарита |

Пример `/obstacle/status`:

```json
{"obstacle": true, "nearest_m": 55.6, "visibility_m": 169.5, "latency_ms": 71.2,
 "objects": [{"id": 1, "distance_m": 55.6, "lateral_m": 0.01, "height_m": 1.52,
              "size_m": [0.6, 0.9, 1.3], "points": 128, "confidence": 1.0}]}
```

## Параметры

Все параметры алгоритма лежат в [config/params.yaml](config/params.yaml). Свой файл можно передать при запуске:

```bash
docker run --rm -it -v /path/to/bags:/data -v $(pwd)/my.yaml:/cfg.yaml transport_detection \
  play /data/doubleT_obstacle config:=/cfg.yaml
```

Аргументы запуска ноды:

| Аргумент | По умолчанию | Смысл |
|---|---|---|
| `input_topic` | пусто | топик с облаком точек; по умолчанию нода сама находит первый топик типа `PointCloud2` |
| `config` | `config/params.yaml` | параметры алгоритма |
| `log_path` | пусто | файл, куда писать результат каждого кадра |
| `rate` | `1.0` | скорость проигрывания записи в режиме `play` |
| `read_ahead` | `20` | сколько сообщений плеер читает заранее в режиме `play` |
| `play_delay` | `3.0` | через сколько секунд после старта ноды запускать плеер в режиме `play` |
| `rviz` | `false` | запустить RViz |

Основные параметры алгоритма:

| Параметр | Смысл |
|---|---|
| `forward_axis` | ось лидара, смотрящая вперёд (`-y` в выданных записях) |
| `gauge_half_width`, `gauge_height` | половина ширины и высота габарита над головкой рельса |
| `gauge_low_half_width`, `gauge_low_height` | узкая нижняя часть габарита |
| `max_range` | до какой дальности искать |
| `track_hits_to_confirm`, `track_window` | подтверждение объекта по кадрам |

Положение лидара определяется автоматически по первым кадрам.

## Замечания по запуску

- В образе используется CycloneDDS с настройками для больших облаков точек (`docker/cyclonedds.xml`).
  Если запись проигрывается на хосте с другой реализацией DDS, её можно выбрать при запуске контейнера,
  например `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- Плеер `ros2 bag play` по умолчанию заранее читает в память много сообщений. Кадр лидара весит
  от 8 до 24 МБ, поэтому на машине с небольшим объёмом памяти стоит добавить
  `--read-ahead-queue-size 20`. В режиме `play` это уже сделано.
- В Docker Desktop на Windows и macOS записи, подключённые из папки хоста, читаются медленно,
  и плеер не успевает за реальным временем. Для демонстрации лучше скопировать запись внутрь
  контейнера или в том Docker. На Linux такой проблемы нет.

## Без Docker

Нужен Python 3.8+.

```bash
pip install -e .[offline]
python tools/run_bag.py /path/to/bags/doubleT_obstacle --out results/doubleT_obstacle.jsonl
python tools/evaluate.py results/*.jsonl
python tools/render_video.py /path/to/bags/doubleT_obstacle --out results/doubleT_obstacle.mp4
python -m pytest tests
```

## Видео

- [Реальное препятствие, поезд стоит](media/real_obstacle.mp4)
- [Добавленные препятствия, поезд едет](media/synthetic_obstacles.mp4)
- [Пустой тоннель с кривой и гермоворотами](media/empty_curve.mp4)

Видео собираются командой `python tools/render_video.py <запись> --out <файл>.mp4`.

## Документация

- [Алгоритм](docs/algorithm.md)
- [Архитектура](docs/architecture.md)
- [Эксперименты](docs/experiments.md)
- [Анализ данных](docs/data.md)

## Структура

```
detector/     ядро: разбор облака, калибровка, путь, габарит, кластеры, трекинг
ros2_ws/      ROS 2 пакет lidar_obstacle_detector: нода, launch, RViz
config/       параметры алгоритма
tools/        офлайн-прогон, метрики, видео
tests/        тесты на синтетической сцене
analysis/     скрипты анализа данных
docs/         документация
reports/      графики
media/        видео работы
```
