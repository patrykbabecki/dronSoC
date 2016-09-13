#ifndef KALMAN
#define KALMAN

#define freq 250
#define dt 0.004
//  [a1 a2;
//		b1 b2];



typedef struct matrix2x2{
	double a1,a2,b1,b2;
}MATRIX22;

typedef struct kalman{
	MATRIX22 X0,P0,XPRI,PPRI,XPOST,PPOST,U,eps,S,K;
}KALMAN_MATRIX;

typedef struct stan{
	MATRIX22 A,B,C,V,At,Ct,W;
	
}MODEL;

void UPDATE_SZUMY(MODEL *m,double szum_pomiarowy,double szum_procesowy);
void MATRIX_ZERO(MATRIX22 *x);
void kalman_model_init(MODEL *m,KALMAN_MATRIX *k,double szum_pomiarowy,double szum_procesowy);

void Ustaw_Wymuszenie(KALMAN_MATRIX *k,double u);
void Filtr_Kalmana(KALMAN_MATRIX *kalman,MODEL *model,MATRIX22 *X,ORIENTACJA_ACC *orient_acc,double *pomiar_kata);




#endif
