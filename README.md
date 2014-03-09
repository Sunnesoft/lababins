Лабораторная работа по предмету "Комплексирование бортовых систем".

На данный момент реализован идеальный вариант работы БИНС при известных ошибках начальной ориентации. 

Убедительная просьба в коммит не включать временные файлы, создаваемые редактором.

Использование комплекса:

Сборка: 
-Qt libs 5.1 и выше
-MinGW 4.7 и выше, либо MSVC с поддержкой c++11

Визуализация:
-GnuPlot 4.6 и выше ( на других v. не проверялся )

Алгоритм:

1. Собираешь 
2. Запускаешь бинарник
3. Осуществляешь выставку комплекса
4. Запускаешь обработку
5. Результаты обработки (файлы лога и данных) скармливаешь скрипту visual/plot. *
6. Смотришь графики
7. Думаешь **


|* Прописываешь path - абсолютный путь до файлов лога и данных.

|** Желательно делать это на каждом шаге. Но можно и в конце.
