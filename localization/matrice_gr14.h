/*! 
 * \author Group 14
 * \file matrice.h
 * \brief localization sensors fusion with Kalman
 */



#ifndef _MATRICE_GR14_H_
#define _MATRICE_GR14_H_

#include "CtrlStruct_gr14.h"
#include "init_pos_gr14.h"



NAMESPACE_INIT(ctrlGr14);


void mult_matrices_3x3(double a[3][3], double b[3][3], double result[3][3]); 
void mult_matrices_vect_3x3(double a[3][3], double b[3], double result[3]); 
int inv_mat_3x3(double mat[3][3],double inv[3][3]); 
void mat_trans(double mat[3][3],double trans_mat[3][3]); 
void mat_add(double mat_1[3][3],double mat_2[3][3],double result[3][3],bool add_or_sub);
void vect_add(double vect_1[3],double vect_2[3],double result[3],bool add_or_sub);
void copy_mat(double mat[3][3],double copy[3][3]);
void copy_vect(double vect[3],double copy[3]);
void print_mat(double mat[3][3]);
void print_vect(double vect[3]);
void set_mat_0(double mat[3][3]);
void set_vect_0(double vect[3]);


NAMESPACE_CLOSE();

#endif