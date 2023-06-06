/*------------------------------------------------------------------------------
* gnssgo unit test driver : matrix and vector functions
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

func dbout1(x, y, P, H, R []float64, n, m int) {
	fmt.Printf("x=[\n")
	gnssgo.MatPrint(x, n, 1, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("y=[\n")
	gnssgo.MatPrint(y, m, 1, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("P=[\n")
	gnssgo.MatPrint(P, n, n, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("H=[\n")
	gnssgo.MatPrint(H, m, n, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("R=[\n")
	gnssgo.MatPrint(R, m, m, 8, 4)
	fmt.Printf("];\n")
}

func dbout2(x, P []float64, n int) {
	fmt.Printf("xu=[\n")
	gnssgo.MatPrint(x, n, 1, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("Pu=[\n")
	gnssgo.MatPrint(P, n, n, 8, 4)
	fmt.Printf("];\n")
	fmt.Printf("K=P*H'/(H*P*H'+R);\n")

	fmt.Printf("xd=x+K*y;\n")
	fmt.Printf("Pd=P-K*H*P\n")
	fmt.Printf("xu-xd,Pu-Pd\n")
}

/* mat(),gnssgo.IMat(),gnssgo.Zeros(),gnssgo.Eye()  */
func Test_matrixutest1(t *testing.T) {
	var i, j int
	var n, m = 100, 200
	var b []int
	var a []float64
	assert := assert.New(t)
	a = gnssgo.Mat(0, 0)
	assert.True(a == nil)
	a = gnssgo.Mat(0, 1)
	assert.True(a == nil)
	a = gnssgo.Mat(1, 0)
	assert.True(a == nil)
	a = gnssgo.Mat(1, 1)
	assert.True(a != nil)
	a = nil
	/*  a=gnssgo.Mat(1000000,1000000); assert.True(a==nil); */
	a = gnssgo.Zeros(0, m)
	assert.True(a == nil)
	a = gnssgo.Zeros(n, 0)
	assert.True(a == nil)
	a = gnssgo.Zeros(n, m)
	assert.True(a != nil)
	for i = 0; i < n; i++ {
		for j = 0; j < m; j++ {
			assert.True(a[i+j*n] == 0.0)
		}
	}
	a = nil
	/*  a=gnssgo.Zeros(10000000,1000000); assert.True(a==nil); */
	a = gnssgo.Eye(0)
	assert.True(a == nil)
	a = gnssgo.Eye(n)
	assert.True(a != nil)
	for i = 0; i < n; i++ {
		for j = 0; j < n; j++ {
			if i == j {
				assert.True(a[i+j*n] == 1.0)
			} else {
				assert.True(a[i+j*n] == 0.0)
			}
		}
	}
	a = nil
	/*  a=gnssgo.Eye(1000000); assert.True(a==nil); */
	b = gnssgo.IMat(0, m)
	assert.True(b == nil)
	b = gnssgo.IMat(n, 0)
	assert.True(b == nil)
	b = gnssgo.IMat(n, m)
	assert.True(b != nil)
	b = nil
	/*
	   a=gnssgo.IMat(1000000,1000000); assert.True(a==nil);
	*/

}

/* gnssgo.Dot() */
func Test_matrixutest2(t *testing.T) {
	var i int
	var a []float64 = []float64{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}
	var b []float64 = []float64{7.0, 8.0, 9.0, 1.4, 1.6, 7.8}
	var c, d float64
	assert := assert.New(t)
	for i, d = 0, 0.0; i < 6; i++ {
		d += a[i] * b[i]
	}
	c = gnssgo.Dot(a, b, 0)
	assert.True(c == 0.0)
	c = gnssgo.Dot(a, b, 6)
	assert.True(math.Abs(c-d) < 1e-14)
	for i, d = 0, 0.0; i < 6; i++ {
		d += a[i] * a[i]
	}
	d = math.Sqrt(d)
	c = gnssgo.Norm(a, 6)
	assert.True(math.Abs(c-d) < 1e-14)
	for i, d = 0, 0.0; i < 6; i++ {
		d += b[i] * b[i]
	}
	d = math.Sqrt(d)
	c = gnssgo.Norm(b, 6)
	assert.True(math.Abs(c-d) < 1e-14)
}

var A []float64 = []float64{
	0.935469699107605, 0.893649530913534,
	0.916904439913408, 0.0578913047842686,
	0.410270206990945, 0.352868132217}
var B []float64 = []float64{
	0.813166497303758, 0.13889088195695,
	0.00986130066092356, 0.202765218560273}
var AB []float64 = []float64{
	0.769505165266963, 0.311129254005026,
	0.74616685532878, 0.139088009397138,
	0.337097725912365, 0.128532174841568}
var ABT []float64 = []float64{
	0.884812390066127, 0.190425990414052,
	0.753636546145776, 0.0207802134266434,
	0.382628153265035, 0.0755951818152924}
var invB []float64 = []float64{
	1.24006142464565, -0.849421938204008,
	-0.0603092514252338, 4.97312316323555}

/* gnssgo.MatMul() */
func Test_matrixutest3(t *testing.T) {
	var i, j int
	var C [6]float64
	assert := assert.New(t)
	gnssgo.MatMul("TT", 3, 2, 2, 1.0, A, B, 0.0, C[:])
	for i = 0; i < 3; i++ {
		for j = 0; j < 2; j++ {
			assert.True(math.Abs(C[i+j*3]-AB[j+i*2]) < 1e-9)
		}
	}

	gnssgo.MatMul("NN", 2, 3, 2, 1.0, B, A, 0.0, C[:])
	for i = 0; i < 2; i++ {
		for j = 0; j < 3; j++ {
			assert.True(math.Abs(C[i+j*2]-AB[i+j*2]) < 1e-9)
		}
	}

	gnssgo.MatMul("TN", 3, 2, 2, 1.0, A, B, 0.0, C[:])
	for i = 0; i < 3; i++ {
		for j = 0; j < 2; j++ {
			assert.True(math.Abs(C[i+j*3]-ABT[j+i*2]) < 1e-9)
		}
	}

	gnssgo.MatMul("TN", 2, 3, 2, 1.0, B, A, 0.0, C[:])
	for i = 0; i < 2; i++ {
		for j = 0; j < 3; j++ {
			assert.True(math.Abs(C[i+j*2]-ABT[i+j*2]) < 1e-9)
		}
	}

}

/* matinv() */
func Test_matrixutest4(t *testing.T) {
	var i, j int
	var C [4]float64
	assert := assert.New(t)
	copy(C[:], B[:4])
	gnssgo.MatInv(B, 2)
	for i = 0; i < 2; i++ {
		for j = 0; j < 2; j++ {
			assert.True(math.Abs(B[i+j*2]-invB[i+j*2]) < 1e-9)
		}
	}
	gnssgo.MatInv(B, 2)
	for i = 0; i < 2; i++ {
		for j = 0; j < 2; j++ {
			assert.True(math.Abs(B[i+j*2]-C[i+j*2]) < 1e-9)
		}
	}
}

var H []float64 = []float64{
	0.123, 0.345, 0.567,
	0.890, -0.135, 0.791,
	1.020, 2.489, 0.111,
	0.321, -1.002, 5.678}
var y []float64 = []float64{
	0.3456,
	1.5678,
	0.1047,
	0.1047}
var xs []float64 = []float64{
	1.77586016656388,
	-0.64484683008484,
	-0.18684144028875}
var Qs []float64 = []float64{
	1.41343666098904, -0.56122396983116, -0.20536436422033,
	-0.56122396983117, 0.37710272892140, 0.10628198948746,
	-0.20536436422033, 0.10628198948746, 0.06392702788446}

/* lsq() */
func Test_matrixutest5(t *testing.T) {
	var i int
	var x [3]float64
	var Q [9]float64
	assert := assert.New(t)
	gnssgo.LSQ(H, y, 3, 4, x[:], Q[:])
	for i = 0; i < 3; i++ {
		assert.True(math.Abs(x[i]-xs[i]) < 1e-9)
	}
	for i = 0; i < 9; i++ {
		assert.True(math.Abs(Q[i]-Qs[i]) < 1e-9)
	}
}

/* matcpy() */
func Test_matrixutest6(t *testing.T) {
	assert := assert.New(t)
	a := gnssgo.Mat(100, 200)
	b := gnssgo.Zeros(100, 200)
	c := 0.0
	var i, j int
	for i = 0; i < 100; i++ {
		for j = 0; j < 200; j++ {
			a[i+j*100] = c
			c += 1.0
		}
	}
	gnssgo.MatCpy(b, a, 100, 200)
	for i, c = 0, 0.0; i < 100; i++ {
		for j = 0; j < 200; j++ {
			assert.True(a[i+j*100] == b[i+j*100])
		}
	}

}
