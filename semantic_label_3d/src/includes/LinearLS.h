#include <vector>
#include "matrix.h"
#include <iostream>

inline void fit(matrx <double> &A, std::vector<double> &b, double numPoints)
{
  bool success;
  double temp, temp1, temp2;
  matrx <double> mat( 4, 4);
  std::vector<double> result(6);
  //Obtain mat = A_transpose * A
  for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
	{
	  for (int k=0; k<numPoints; k++)
	    {
	      A.getvalue(k,i,temp1,success);
	      A.getvalue(k,j,temp2,success);
	      mat.getvalue(i,j,temp,success);
	      mat.setvalue(i,j,temp+temp1 * temp2);
	    }
	}
    }
  //Invert mat
  mat.invert();
  //Form x = mat * A_transpose * b
  for (int i=0; i<4; i++)
    {
      result[i] = 0;
      for (int j=0; j<4; j++)
        {
          for (int k=0; k<numPoints; k++)
            {
	      mat.getvalue(i,j,temp1,success);
	      A.getvalue(k,j,temp2,success);
	      result[i] += temp1*temp2*b[k];
            }
        }
    }
  for (int i=0; i<4; i++)
    {
      b[i] = result[i];
    }
  return;
}
