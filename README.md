# transport_detection

Обнаружение посторонних объектов в габарите поезда метро по данным 3D-лидара
Hesai Pandar128E3X из записей ROS 2.

## Документы

- [Анализ данных](docs/02_data_analysis.md)

## Структура

```
analysis/
  bag_io.py              чтение облаков точек из записей ROS 2 без установленного ROS
  overview.py            сводка по записи и графики
  corridor_waterfall.py  заполненность коридора перед поездом по кадрам
  unzst.py               распаковка архивов .zst
reports/                 графики
docs/                    документация
```

## Данные

Данные в репозиторий не входят. По умолчанию скрипты ждут их рядом с репозиторием:

```
../data/for_hackathon/<запись>/
../data/synthetic/cloud_with_fake_obj/
```

Распаковка архива:

```bash
python analysis/unzst.py for_hackathon.zst ../data
```

## Запуск анализа

```bash
pip install -r analysis/requirements.txt
cd analysis
python overview.py ../../data/for_hackathon/* --out ../reports/overview
python corridor_waterfall.py ../../data/synthetic/cloud_with_fake_obj --out ../reports/waterfall
```
