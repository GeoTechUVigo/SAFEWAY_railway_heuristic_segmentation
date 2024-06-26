# [Automatic Point Cloud Semantic Segmentation of Complex Railway Environments](https://doi.org/10.3390/rs13122332)

* Parte del proyecto [SAFEWAY2020](https://github.com/orgs/GeoTechUVigo/projects/1).
* Autor: [Daniel Lamas Novoa](https://orcid.org/0000-0001-7275-183X)

## Overview
Este repositorio contiene el código del artículo titulado [Automatic Point Cloud Semantic Segmentation of Complex Railway Environments](https://doi.org/10.3390/rs13122332)

![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/workflow.png)
            

Tiene como objetivo el de segmentar los elementos presentes en entornos de ferrocarril a partir de nubes de puntos en formato .las y de la trayectoria del sensor en formato ASCII. Como resultado, se obtienen los índices de los puntos pertenecientes a cada elemento, indicando de qué tipo de elemento se trata. 
 
El proceso consta de un seccionado, un voxelizado y una segmentación heurística. En el proceso de seccionado, las nubes se organizan en fragmentos de una longitud determinada y limitando la curvatura máxima de la vía del tren en ella. Además, los puntos lejanos a la trayectoria no se incluyen en ninguna sección, ya que no aportan información de la infraestructura.

Tras voxelizar las secciones, se lleva a cabo el proceso de segmentación heurística. En este proceso se extrae secuencialmente cada tipo de elementos de cada sección.

El resultado es una nueva nube segmentada o la modificación de la nube de entrada, escrbiendo en:
-Record.classification indicando el tipo de elemento.
-Record.point_source_id insicando el número de elemento.
-Record.user_data indicando la vía a la que pertenece cada elemento.


A mayores se guarda en formato json los tiempos de ejecución y los errores si los hubiere.


# TESELAS
Su funcionamiento es el mismo pero partiendo de Teselas .laz como entrada y una matriz en la que se especifique si la tesela está suficientemente cerca de cada punto de la trayectoria. Es neceario tener las LAStools instaladas.

Primero, se secciona la trayectoria y se analiza cada sección en un bucle paralelo. Con las LAStools se unen las teselas que están cerca de la sección de la trayectoria, guardando la nube en un archivo temporal. Esta nube se cargar y se
aplica una reducción de puntos por si el archivo es demasiado pesado, y se guarda en disco. Esta nube será la que se guarde segmentada.

Después, se descartan los puntos remotos, y se procede como en SAFEWAY_railway_heuristic_segmentation, voxlizando, segmentando y modificando la nube. Por último, se comprime en .laz.  

## Citation
If you find our work useful in your research, please consider citing:
```
@article{LAMAS20210614,
title = {Automatic Point Cloud Semantic Segmentation of Complex Railway Environments},
journal = {Remote Sensing},
volume = {13},
pages = {2332},
year = {2021},
issn = {2072-4292},
doi = {https://doi.org/10.3390/RS13122332},
url = {https://www.sciencedirect.com/science/article/pii/S0926580523001255},
author = {Daniel Lamas and Mario Soilán and Javier Grandío and Belén Riveiro},
keywords = {LiDAR, point clouds, railway inventory, semantic segmentation},
abstract = {The growing development of data digitalisation methods has increased their demand and applications in the transportation infrastructure field. Currently, mobile mapping systems (MMSs) are one of the most popular technologies for the acquisition of infrastructure data, with three-dimensional (3D) point clouds as their main product. In this work, a heuristic-based workflow for semantic segmentation of complex railway environments is presented, in which their most relevant elements are classified, namely, rails, masts, wiring, droppers, traffic lights, and signals. This method takes advantage of existing methodologies in the field for point cloud processing and segmentation, taking into account the geometry and spatial context of each classified element in the railway environment. This method is applied to a 90-kilometre-long railway lane and validated against a manual reference on random sections of the case study data. The results are presented and discussed at the object level, differentiating the type of the element. The indicators F1 scores obtained for each element are superior to 85%, being higher than 99% in rails, the most significant element of the infrastructure. These metrics showcase the quality of the algorithm, which proves that this method is efficient for the classification of long and variable railway sections, and for the assisted labelling of point cloud data for future applications based on training supervised learning models.}
}
```

# EJEMPLOS
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/tunnel_entrance.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/two_tracks.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/under_overpass.png)
![alt_text](https://github.com/GeoTechUVigo/SAFEWAY_railway_heuristic_segmentation/blob/main/Images_readme/several_tracks.png)

# EJECUCIÓN
Para su ejecución se debe de tener el código principal, RailwayHeuristicSegmentation o RailwayHeuristicSegmentation, y las carpetas Classes, Preprocessing, Processing y Postprocessing con sus respectivas subcarpetas y códigos.
En el código principal se deben indicar las rutas de los archivos de trayectoria y de las nubes, y la ruta para guardar las salidas. También se debe indicar la ruta de las nubes modelo de los archivos. Las nubes modelo empleadas están en la carpeta Models.

El código se ejecuta en un bucle paralelo. Puede ser necesario configurar cómo se desea hacer el bucle paralelo.

La salida es la nube en la que se modificaron los siguientes campos:

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

Si no es capaz de segmentar una nube, guarda una variable con un mensaje de error en la ruta especificada para ello.

Las nubes deben tener georreferencia 3D, intensidad. En el caso de SAFEWAY_railway_heuristic_segmentation también sellos de tiempo.

La trayectoria debe tener georreferencia 3D. En el caso de SAFEWAY_railway_heuristic_segmentation también sellos de tiempo sincronizados con los de las nubes.
