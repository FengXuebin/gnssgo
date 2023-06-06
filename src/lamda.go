/*------------------------------------------------------------------------------
* lambda.c : integer ambiguity resolution
*
*          Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
*         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
*         1995
*     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
*         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/13 1.0 new
*           2015/05/31 1.1 add api lambda_reduction(), lambda_search()
*		    2022/05/31 1.0  rewrite lambda.c with golang by fxb
*-----------------------------------------------------------------------------*/
package gnssgo

import (
	"math"
)

/* constants/macros ----------------------------------------------------------*/

const LOOPMAX int = 10000 /* maximum count of search loop */

func SGN(x float64) float64 {
	if x <= 0.0 {
		return -1.0
	} else {
		return 1.0
	}
}

// func ROUND_F(x float64) float64 { return math.Floor(x + 0.5) }
func ROUND_F(x float64) float64 {
	t := math.Trunc(x)
	if math.Abs(x-t) >= 0.5 {
		return t + math.Copysign(1, x)
	}
	return t
}

// #define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
func LD(n int, Q, L, D []float64) int {
	var (
		i, j, k, info int
		a             float64
	)
	A := Mat(n, n)
	copy(A, Q)
	//  memcpy(A,Q,sizeof(double)*n*n);
	for i = n - 1; i >= 0; i-- {
		if D[i] = A[i+i*n]; D[i] <= 0.0 {
			info = -1
			break
		}
		a = math.Sqrt(D[i])
		for j = 0; j <= i; j++ {
			L[i+j*n] = A[i+j*n] / a
		}
		for j = 0; j <= i-1; j++ {
			for k = 0; k <= j; k++ {
				A[j+k*n] -= L[i+k*n] * L[i+j*n]
			}
		}
		for j = 0; j <= i; j++ {
			L[i+j*n] /= L[i+i*n]
		}
	}
	if info != 0 {
		Trace(2, "%s : LD factorization error\n", "lamda.go")
	}
	return info
}

/* integer Gauss transformation ----------------------------------------------*/
func Gauss(n int, L, Z []float64, i, j int) {
	if mu := int(ROUND_F(L[i+j*n])); mu != 0 {
		for k := i; k < n; k++ {
			L[k+n*j] -= float64(mu) * L[k+i*n]
		}
		for k := 0; k < n; k++ {
			Z[k+n*j] -= float64(mu) * Z[k+i*n]
		}
	}
}

/* permutations --------------------------------------------------------------*/
func Perm(n int, L, D []float64, j int, del float64, Z []float64) {
	eta := D[j] / del
	lam := D[j+1] * L[j+1+j*n] / del
	D[j] = eta * D[j+1]
	D[j+1] = del
	for k := 0; k <= j-1; k++ {
		a0 := L[j+k*n]
		a1 := L[j+1+k*n]
		L[j+k*n] = -L[j+1+j*n]*a0 + a1
		L[j+1+k*n] = eta*a0 + lam*a1
	}
	L[j+1+j*n] = lam
	for k := j + 2; k < n; k++ {
		L[k+j*n], L[k+(j+1)*n] = L[k+(j+1)*n], L[k+j*n]
	}
	for k := 0; k < n; k++ {
		Z[k+j*n], Z[k+(j+1)*n] = Z[k+(j+1)*n], Z[k+j*n]
	}
}

/* lambda Reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
func Reduction(n int, L, D, Z []float64) {
	var i, j, k int

	j = n - 2
	k = n - 2
	for j >= 0 {
		if j <= k {
			for i = j + 1; i < n; i++ {
				Gauss(n, L, Z, i, j)
			}
		}
		del := D[j] + L[j+1+j*n]*L[j+1+j*n]*D[j+1]
		if del+(1e-6) < D[j+1] { /* compared considering numerical error */
			Perm(n, L, D, j, del, Z)
			k = j
			j = n - 2
		} else {
			j--
		}
	}
}

/* modified lambda (mlambda) Search (ref. [2]) -------------------------------*/
func Search(n, m int, L, D, zs, zn, s []float64) int {
	var (
		i, j, k, c, nn, imax int
		newdist, maxdist, y  float64 = 0.0, 1e99, 0.0
	)
	S := Zeros(n, n)
	dist := Mat(n, 1)
	zb := Mat(n, 1)
	z := Mat(n, 1)
	step := Mat(n, 1)

	k = n - 1
	dist[k] = 0.0
	zb[k] = zs[k]
	z[k] = ROUND_F(zb[k])
	y = zb[k] - z[k]
	step[k] = SGN(y)
	for c = 0; c < LOOPMAX; c++ {
		newdist = dist[k] + y*y/D[k]
		if newdist < maxdist {
			if k != 0 {
				k--
				dist[k] = newdist
				for i = 0; i <= k; i++ {
					S[k+i*n] = S[k+1+i*n] + (z[k+1]-zb[k+1])*L[k+1+i*n]
				}
				zb[k] = zs[k] + S[k+k*n]
				z[k] = ROUND_F(zb[k])
				y = zb[k] - z[k]
				step[k] = SGN(y)
			} else {
				if nn < m {
					if nn == 0 || newdist > s[imax] {
						imax = nn
					}
					for i = 0; i < n; i++ {
						zn[i+nn*n] = z[i]
					}
					s[nn] = newdist
					nn++
				} else {
					if newdist < s[imax] {
						for i = 0; i < n; i++ {
							zn[i+imax*n] = z[i]
						}
						s[imax] = newdist
						for i, imax = 0, 0; i < m; i++ {
							if s[imax] < s[i] {
								imax = i
							}
						}
					}
					maxdist = s[imax]
				}
				z[0] += step[0]
				y = zb[0] - z[0]
				step[0] = -step[0] - SGN(step[0])
			}
		} else {
			if k == n-1 {
				break
			} else {
				k++
				z[k] += step[k]
				y = zb[k] - z[k]
				step[k] = -step[k] - SGN(step[k])
			}
		}
	}
	for i = 0; i < m-1; i++ { /* sort by s */
		for j = i + 1; j < m; j++ {
			if s[i] < s[j] {
				continue
			}
			s[i], s[j] = s[j], s[i]
			for k = 0; k < n; k++ {
				zn[k+i*n], zn[k+j*n] = zn[k+j*n], zn[k+i*n]
			}
		}
	}

	if c >= LOOPMAX {
		Trace(2, "%s : search loop count overflow\n", "lamda")
		return -1
	}
	return 0
}

/* Lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by Lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
func Lambda(n, m int, a, Q, F, s []float64) int {
	var (
		info          int
		L, D, Z, z, E []float64
	)

	if n <= 0 || m <= 0 {
		return -1
	}
	L = Zeros(n, n)
	D = Mat(n, 1)
	Z = Eye(n)
	z = Mat(n, 1)
	E = Mat(n, m)

	/* LD factorization */
	if info = LD(n, Q, L, D); info == 0 {

		/* lambda reduction */
		Reduction(n, L, D, Z)
		MatMul("TN", n, 1, n, 1.0, Z, a, 0.0, z) /* z=Z'*a */

		/* mlambda search */
		if info = Search(n, m, L, D, z, E, s); info == 0 {

			info = Solve("T", Z, E, n, m, F) /* F=Z'\E */
		}
	}

	return info
}

/* lambda reduction ------------------------------------------------------------
* reduction by lambda (ref [1]) for integer least square
* args   : int    n      I  number of float parameters
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *Z     O  lambda reduction matrix (n x n)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
func LambdaReduction(n int, Q, Z []float64) int {

	if n <= 0 {
		return -1
	}

	L := Zeros(n, n)
	D := Mat(n, 1)

	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			if i == j {
				Z[i+j*n] = 1.0
			} else {
				Z[i+j*n] = 0.0
			}
		}
	}
	/* LD factorization */
	if info := LD(n, Q, L, D); info > 0 {

		return info
	}
	/* lambda reduction */
	Reduction(n, L, D, Z)

	return 0
}

/* mlambda search --------------------------------------------------------------
* search by  mlambda (ref [2]) for integer least square
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
func LambdaSearch(n, m int, a, Q, F, s []float64) int {
	if n <= 0 || m <= 0 {
		return -1
	}

	L := Zeros(n, n)
	D := Mat(n, 1)

	/* LD factorization */
	if info := LD(n, Q, L, D); info > 0 {

		return info
	}
	/* mlambda search */
	return Search(n, m, L, D, a, F, s)

}
