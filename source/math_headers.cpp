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

#include "math_headers.h"

EigenMatrix3 MatVec2Mat(const EigenVector9& vector9)
{
	EigenMatrix3 mat3;
	mat3.col(0) = vector9.block_vector(0);
	mat3.col(1) = vector9.block_vector(1);
	mat3.col(2) = vector9.block_vector(2);

	return mat3;
}

EigenVector9 Mat2MatVec(const EigenMatrix3& matrix3)
{
	EigenVector9 vec9;
	vec9.block_vector(0) = matrix3.col(0);
	vec9.block_vector(1) = matrix3.col(1);
	vec9.block_vector(2) = matrix3.col(2);

	return vec9;
}

EigenVector4 Mat2QuatVec(const EigenMatrix3& matrix)
{
	ScalarType trace;
	ScalarType s;
	ScalarType t;
	int i, j, k;

	EigenVector4 quat_vec;
	static int next[3] = { 1, 2, 0 };

	trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);

	if (trace > 0.0f) {

		t = trace + 1.0f;
		s = ReciprocalSqrt(t) * 0.5f;

		quat_vec[3] = s * t;
		quat_vec[0] = (matrix(2, 1) - matrix(1, 2)) * s;
		quat_vec[1] = (matrix(0, 2) - matrix(2, 0)) * s;
		quat_vec[2] = (matrix(1, 0) - matrix(0, 1)) * s;

	}
	else {

		i = 0;
		if (matrix(1, 1) > matrix(0, 0)) {
			i = 1;
		}
		if (matrix(2, 2) > matrix(i, i)) {
			i = 2;
		}
		j = next[i];
		k = next[j];

		t = (matrix(i, i) - (matrix(j, j) + matrix(k, k))) + 1.0f;
		s = ReciprocalSqrt(t) * 0.5f;

		quat_vec[i] = s * t;
		quat_vec[3] = (matrix(k, j) - matrix(j, k)) * s;
		quat_vec[j] = (matrix(j, i) + matrix(i, j)) * s;
		quat_vec[k] = (matrix(k, i) + matrix(i, k)) * s;
	}

	return quat_vec;
}

EigenVector4 MatVec2QuatVec(const EigenVector9& matrix)
{
	ScalarType trace;
	ScalarType s;
	ScalarType t;
	int i, j, k;

	EigenVector4 quat_vec;

	static int next[3] = { 1, 2, 0 };

	trace = matrix[0 * 3 + 0] + matrix[1 * 3 + 1] + matrix[2 * 3 + 2];

	if (trace > 0.0f) {

		t = trace + 1.0f;
		s = ReciprocalSqrt(t) * 0.5f;

		quat_vec[3] = s * t;
		quat_vec[0] = (matrix[1 * 3 + 2] - matrix[2 * 3 + 1]) * s;
		quat_vec[1] = (matrix[2 * 3 + 0] - matrix[0 * 3 + 2]) * s;
		quat_vec[2] = (matrix[0 * 3 + 1] - matrix[1 * 3 + 0]) * s;

	}
	else {

		i = 0;
		if (matrix[1 * 3 + 1] > matrix[0 * 3 + 0]) {
			i = 1;
		}
		if (matrix[2 * 3 + 2] > matrix[i * 3 + i]) {
			i = 2;
		}
		j = next[i];
		k = next[j];

		t = (matrix[i * 3 + i] - (matrix[j * 3 + j] + matrix[k * 3 + k])) + 1.0f;
		s = ReciprocalSqrt(t) * 0.5f;

		quat_vec[i] = s * t;
		quat_vec[3] = (matrix[j * 3 + k] - matrix[k * 3 + j]) * s;
		quat_vec[j] = (matrix[i * 3 + j] + matrix[j * 3 + i]) * s;
		quat_vec[k] = (matrix[i * 3 + k] + matrix[k * 3 + i]) * s;
	}

	return quat_vec;
}


void Matrix3x3x3x3MultiplyMatrix3x3(const EigenMatrix3 lhs[3][3], const EigenMatrix3 &rhs, EigenMatrix3 output[3][3])
{
	output[0][0] = lhs[0][0] * rhs(0, 0) + lhs[0][1] * rhs(1, 0) + lhs[0][2] * rhs(2, 0);
	output[0][1] = lhs[0][0] * rhs(0, 1) + lhs[0][1] * rhs(1, 1) + lhs[0][2] * rhs(2, 1);
	output[0][2] = lhs[0][0] * rhs(0, 2) + lhs[0][1] * rhs(1, 2) + lhs[0][2] * rhs(2, 2);

	output[1][0] = lhs[1][0] * rhs(0, 0) + lhs[1][1] * rhs(1, 0) + lhs[1][2] * rhs(2, 0);
	output[1][1] = lhs[1][0] * rhs(0, 1) + lhs[1][1] * rhs(1, 1) + lhs[1][2] * rhs(2, 1);
	output[1][2] = lhs[1][0] * rhs(0, 2) + lhs[1][1] * rhs(1, 2) + lhs[1][2] * rhs(2, 2);

	output[2][0] = lhs[2][0] * rhs(0, 0) + lhs[2][1] * rhs(1, 0) + lhs[2][2] * rhs(2, 0);
	output[2][1] = lhs[2][0] * rhs(0, 1) + lhs[2][1] * rhs(1, 1) + lhs[2][2] * rhs(2, 1);
	output[2][2] = lhs[2][0] * rhs(0, 2) + lhs[2][1] * rhs(1, 2) + lhs[2][2] * rhs(2, 2);
}

#pragma region solver
#include <iostream>

void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	SparseMatrix I(A.cols(), A.rows());
	I.setIdentity();

	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	ScalarType Regularization = 0.00001;
	bool success = true;
	while (lltSolver.info() != Eigen::Success)
	{
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		lltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(llt solver)" << std::endl;
}

void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	SparseMatrix I(A.cols(), A.rows());
	I.setIdentity();

	ldltSolver.analyzePattern(A_prime);
	ldltSolver.factorize(A_prime);
	ScalarType Regularization = 0.00001;
	bool success = true;
	while (ldltSolver.info() != Eigen::Success)
	{
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		ldltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(ldlt solver)" << std::endl;
}
#pragma endregion

#pragma region utility
//void computeTetBarycenter(const VectorX& vertices, const std::vector<unsigned int>& tet_list, VectorX& barycenter)
//{
//	barycenter.setZero();
//	unsigned int tet_num = tet_list.size()/4;
//
//	for (unsigned int i=0; i<tet_num; ++i)
//	{
//		barycenter.block_vector(i) += vertices.block_vector(tet_list[4*i + 0]);
//		barycenter.block_vector(i) += vertices.block_vector(tet_list[4*i + 1]);
//		barycenter.block_vector(i) += vertices.block_vector(tet_list[4*i + 2]);
//		barycenter.block_vector(i) += vertices.block_vector(tet_list[4*i + 3]);
//		barycenter.block_vector(i) /= 4.0;
//	}
//}


#pragma endregion