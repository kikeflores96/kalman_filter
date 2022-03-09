#include "attitude.h"
#include <math.h>


void eye(double *M, int dim, double diagonal){
    for(int i= 0; i<dim; i++){
      *M = diagonal;
      M+=dim;
  }
};




attitude :: attitude{

  eye(P, sizeof(P)/sizeof(P[0]), 0.1);


}

