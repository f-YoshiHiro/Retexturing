// ---------------------------------------------------------------------------------//
// Copyright (c) 2013, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//

#ifndef _MATH_HEADERS_H_
#define _MATH_HEADERS_H_

// eigen
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Sparse"

// global header
#include "global_headers.h"

// omp
#include <omp.h>

// eigen vectors and matrices
typedef int IndexType;
typedef unsigned int uint;
typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2, 2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3, 3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 4, 4, 0, 4, 4> EigenMatrix4;
typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2, 1> EigenVector2;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3, 1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 4, 1, 0, 4, 1> EigenVector4;
typedef Eigen::Matrix<ScalarType, 9, 1, 0, 9, 1> EigenVector9;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 3> Matrix3X;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXs;
typedef Eigen::Matrix<IndexType, Eigen::Dynamic, Eigen::Dynamic>  EigenMatrixXi;
typedef Eigen::Quaternion<ScalarType> EigenQuaternion;
typedef Eigen::SparseMatrix<ScalarType>      SparseMatrix;
typedef Eigen::Triplet<ScalarType, IndexType> SparseMatrixTriplet;

typedef Eigen::Matrix<IndexType, 2, 1, 0, 2, 1> EigenVector2i;
typedef Eigen::Matrix<IndexType, 3, 1, 0, 3, 1> EigenVector3i;

// omp
#define OPENMP

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)
#define block_matrix(a) block<9,1>(9*(a), 0)
#define block_quaternion(a) block<4,1>(4*(a), 0)

// vec 2 mat, mat 2 vec
EigenMatrix3 MatVec2Mat(const EigenVector9& vector9);
EigenVector9 Mat2MatVec(const EigenMatrix3& matrix3);

EigenVector4 Mat2QuatVec(const EigenMatrix3& matrix);
EigenVector4 MatVec2QuatVec(const EigenVector9& matrix);


void Matrix3x3x3x3MultiplyMatrix3x3(const EigenMatrix3 lhs[3][3], const EigenMatrix3 &rhs, EigenMatrix3 output[3][3]);

// solver
void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = ""); // factorize matrix A using LLT decomposition
void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg = ""); // factorize matrix A using LDLT decomposition
																																	
static ScalarType ReciprocalSqrt(const ScalarType x) { return 1.0 / sqrt(x); }
#endif