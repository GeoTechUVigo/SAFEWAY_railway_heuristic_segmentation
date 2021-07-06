# SAFEWAY_railway_heuristic_segmentation
porjecto SAFEWAY2020

Autor: Daniel Lamas Novoa https://github.com/Daniel22L

Publicación: https://www.mdpi.com/2072-4292/13/12/2332

Más información en la caprpeta Documents

# RESUMEN

![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/workflow.png)
            
 El código descrito es este documento tiene como objetivo el de segmentar los elementos presentes en entornos de ferrocarril a partir de nubes de puntos en formato .las y de la trayectoria del sensor en formato ASCII. Como resultado, se obtienen los índices de los puntos pertenecientes a cada elemento, indicando de qué tipo de elemento se trata. 
 
El proceso consta de un seccionado, un voxelizado y una segmentación heurística. En el proceso de seccionado, las nubes se organizan en fragmentos de una longitud determinada y limitando la curvatura máxima de la vía del tren en ella. Además, los puntos lejanos a la trayectoria no se incluyen en ninguna sección, ya que no aportan información de la infraestructura.

Tras voxelizar las secciones, se lleva a cabo el proceso de segmentación heurística. En este proceso se extrae secuencialmente cada tipo de elementos de cada sección.

El resultado se guarda en un archivo .mat. Este formato consta de celdas en las que en cada una se encuentra cada tipo de elemento. Dentro de estas se encuentran agrupados los índices en la nube de puntos originales de los puntos que conforman cada elemento.

Este archivo también tiene otra variable sobre el status del proceso de segmentación.


# EJEMPLOS
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/tunnel_entrance.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/two_tracks.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/under_overpass.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/several_tracks.png)

# EJECUCIÓN
Para su ejecución se debe de tener el código principal RailwayHeuristicSegmentation y las carpetas Classes, Preprocessing y Railway con sus respectivas subcarpetas y códigos.
En el código principal RailwayHeuristicSegmentation se deben indicar las rutas de los archivos de trayectoria y de las nubes, y la ruta para guardar las salidas. También se debe indicar la ruta de las nubes modelo de los archivos. Las nubes modelo empleadas están en la carpeta Railway\Models.

El código se ejecuta en un bucle paralelo. Puede ser necesario configurar cómo se desea hacer el bucle paralelo.

La salida es la nube .las de entrada en la que se modifican los siguientes campos.

Record.classification: asigna un número a cada punto en función del tipo de elemento al que pertenezca
	
	Rail = 1
	Catenaria = 2
	Contacto = 3
	Dropper = 4
	Otros cables = 5
	Mástiles = 6
	Señales = 7
	Semáforors = 8
	Marcadores = 9
	Señales en mástiles = 10
	Luces = 11
	
	
Record.user_data: asigna un número a los elementos que pertenecen a una misma vía. Estos elementos son raíles, cables de contacto, catenaria y droppers. Todos los elementos de una misma vía tienen un número comprendido entre una potencia de 10. A los raíles se le asigna una potencia de 10 (i ∙ 10^x). Al grupo de cables j de contacto y catenaria con sus droppers pertenecientes a la vía i se les asigna el valor i ∙ 10^x+j. El número máximo de cables de contacto-catenaria de cada vía no puede ser superior a 9.

Record.point_source_id: asigna un número a cada punto en función del grupo al que pertenezcan. Cada grupo tiene un número único.

Si no es capaz de segmentar una nube, guarda una variable con un mensaje de error en la ruta especificada para ello. No se analiza el tipo de error.

Las nubes deben tener georreferencia 3D, intensidad y sellos de tiempo.

La trayectoria debe tener georreferencia 3D y sellos de tiempo sincronizados con los de las nubes.
