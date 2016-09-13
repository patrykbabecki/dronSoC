#include <stm32f4xx.h>
#include"defines.h"

void Skalowanie_Danych_Mems(MEMS *m){
	
(*m).xk=(*m).konwersja*(*m).x;
(*m).yk=(*m).konwersja*(*m).y;
(*m).zk=(*m).konwersja*(*m).z;	

}

void OdejmowanieDryftu(MEMS *m){
	
	(*m).xks10=(*m).xks10-(*m).dryft;
	(*m).yks10=(*m).yks10-(*m).dryft;
	(*m).zks10=(*m).zks10-(*m).dryft;
	
}

