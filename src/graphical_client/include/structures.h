#ifndef STRUCTURES_H
#define STRUCTURES_H

#include "geometry.h"

struct Robot{
	bool hasAngle;					/** Indica si el robot tiene o no el valor actual en 'angle' **/
	vector2d loc;					/** Indica la posición actual del robot en el campo (x, y) (en mm.) **/
//	vector2d posDestino;			/** Indica la posición destino del robot (x, y) (en mm.) **/
//	vector3d velocidades;		/** Indica las velocidades (en mm./s para las componentes x, y; en PI radianes/s para theta)**/
	vector3d vel;
	double angle;					/** Indica la posición del robot en el campo en PI radianes **/
	int id;							/** Inica el id del robot **/
//	bool kick;						/** Indica si ek robot debe de patear **/
	double conf;					/** Variable con valor en el rango [0,1]. Se puede usar para carga de capacitor del kicker, carga de baterias, etc (No se encuentra en uso actualmente) **/
	int team;						/** [0|1|2] Ver descripción abajo **/
	int cameraID;					/** [0|1] En caso de usar visión en esta variable se guarda el registro de que cámara procede el robot **/

	//Overloading for QT find and 
	inline bool operator==(const Robot& r){ return id == r.id && team == r.team; }
};

/** Descripción de equipos **/
typedef enum{
	teamUnknown = 0, 				/** 0 -> Desconocido **/
	teamBlue,						/** 1 -> Azul **/
	teamYellow						/** 2 -> Amarillo **/
}TeamTypes;

#endif //STRUCTURES_H
