#include "matrice.h"
#include "odometry_gr14.h"
#include "triangulation_gr14.h"
#include "useful_gr14.h"


//functions used in matrice operations for the Kalman filter

NAMESPACE_INIT(ctrlGr14);


/*! \multiplication of matrice 3x3
 * 
 * \param[in] a matrix
 * \param[in] b matrix
 * \param[out] result matrix in which the result of a*b is stored
 */
void mult_matrices_3x3(double a[][3], double b[][3], double result[][3])
{
    int i, j, k;
    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            for(k = 0; k < 3; k++)
            {
                result[i][j] +=  a[i][k] *  b[k][j];
            }
        }
    }
}

/*! \ multiplication of a matrix and vector size R_2 or R_3
 * 
 * \param[in] a matrix
 * \param[in] b vector
 * \param[in] size define the size of vector R_2 or R_3
 * \param[out] result matrix in which the result of a*b is stored
 */
void mult_matrices_vect_3x3(double a[3][3], double b[3], double result[3])
{
    int i, k;
    for(i = 0; i < 3; i++)
    {
    	for(k = 0; k < 3; k++)
        {
                result[i] +=  a[i][k] *  b[k];
        }        
    }
}

/*! \inverse of a 3x3 matrix
 * 
 * \param[in] mat matrix
 * \param[out] inv matrix in which is computed the inverse of mat
 * code found http://www.cquestions.com/2011/09/c-program-to-find-inverse-of-matrix.html
 */
int inv_mat_3x3(double mat[3][3],double inv[3][3])
{
	double determinant = 0;
    int i, j;
	for(i=0;i<3;i++)
	{
      determinant = determinant + (mat[0][i]*(mat[1][(i+1)%3]*mat[2][(i+2)%3] - mat[1][(i+2)%3]*mat[2][(i+1)%3]));
      if (determinant == 0) // check if matrix is singular
      {
      	return 0;
      }
	}
	for(i=0;i<3;i++)
	{
	    for(j=0;j<3;j++)
	   	{
	    	inv[i][j]=((mat[(i+1)%3][(j+1)%3] * mat[(i+2)%3][(j+2)%3]) - 
	        (mat[(i+1)%3][(j+2)%3]*mat[(i+2)%3][(j+1)%3]))/ determinant;
	    }
    } 
    return 1;
}

/*! \ transpoze of a matrix 3x3
 * 
 * \param[in] mat matrix
 * \param[out] trans_mat transpose of the matrix mat
 */
void mat_trans(double mat[3][3],double  trans_mat[3][3])
{
    int i, j;
	for(i=0;i<3;i++)
	{
	    for(j=0;j<3;j++)
	   	{
	   		trans_mat[j][i]=mat[i][j];
	   	}
	}
}

/*! \ addition or substraction of a 3x3 matrix
 * 
 * \param[in] mat_1 matrix 1
 * \param[in] mat_2 matrix 2
 * \param[in] add_or_sub define if we are adding or subbing the elements 0 subing(mat_1 - mat_2) 1 adding 
 * \param[out] result matrix in which the result of mat_1+-mat_2 is stored
 */
void mat_add(double mat_1[3][3],double mat_2[3][3],double result[3][3],bool add_or_sub)
{
    int i, j;
    if (add_or_sub) // add
    {
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                result[i][j]=mat_1[i][j]+mat_2[i][j];
            }
        }
    }
    else //  sub
    {
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                result[i][j]=mat_1[i][j]-mat_2[i][j];
            }
        }
    }
}

/*! \ addition or substraction of a 3x3 matrix
 * 
 * \param[in] vect_1 matrix 1
 * \param[in] vect_2 matrix 2
 * \param[in] add_or_sub define if we are adding or subbing the elements 0 subing(vect_1 - vect_2) 1 adding 
 * \param[out] result vect in which the result of mat_1+-mat_2 is stored
 */
void vect_add(double vect_1[3],double vect_2[3],double result[3],bool add_or_sub)
{
    int i;
    if (add_or_sub) // add
    {
        for(i=0;i<3;i++)
        {
            result[i] = vect_1[i] + vect_2[i];
        }
    }
    else //  sub
    {
        for(i=0;i<3;i++)
        {
            result[i] = vect_1[i] - vect_2[i];
        }
    }
}

/*! \copy of a 3x3 matrix
 * 
 * \param[in] mat matrix to be copied
 * \param[out] copy matrix in which we copy the first one
 */
void copy_mat(double mat[3][3],double copy[3][3])
{
    int i,j;
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            copy[i][j]=mat[i][j];
        }
    }
}

/*! copy of R_3 vector
 * 
 * \param[in] vect vector to be copied
 * \param[out] copy vector in which we copy the first one
 */
void copy_vect(double vect[3],double copy[3])
{
    int i;
    for(i=0;i<3;i++)
    {
        copy[i]=vect[i];
    }
}

NAMESPACE_CLOSE();
