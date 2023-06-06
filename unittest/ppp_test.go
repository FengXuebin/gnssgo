/*------------------------------------------------------------------------------
* rtklib unit test driver : ppp functions
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* eci2ecef() */
func Test_ppputest1(t *testing.T) {
	var ep1 []float64 = []float64{1999, 3, 4, 0, 0, 0}
	var gmst float64
	var U [9]float64
	D2R := gnssgo.D2R
	var U1 []float64 = []float64{
		-0.947378027425279, 0.320116956820115, -8.43090456427539e-005,
		-0.32011695222455, -0.947378030590727, -6.36592598714651e-005,
		-0.00010025094616549, -3.33206293083182e-005, 0.999999994419742}
	var erpv [5]float64 = [5]float64{0.06740 * D2R / 3600, 0.24713 * D2R / 3600, 0.649232}
	var i, j int
	assert := assert.New(t)

	gnssgo.Eci2Ecef(gnssgo.Epoch2Time(ep1), erpv[:], U[:], &gmst)

	for i = 0; i < 3; i++ {
		for j = 0; j < 3; j++ {
			fmt.Printf("U(%d,%d)=%15.12f %15.12f %15.12f\n", i, j,
				U[i+j*3], U1[j+i*3], U[i+j*3]-U1[j+i*3])
			assert.True(math.Abs(U[i+j*3]-U1[j+i*3]) < 1e-11)
		}
	}
}

/* sunmoonpos() */
func Test_ppputest2(t *testing.T) {
	var ep1 []float64 = []float64{2010, 12, 31, 8, 9, 10}                               /* utc */
	var rs []float64 = []float64{70842740307.0837, 115293403265.153, -57704700666.9715} /* de405 */
	var rm []float64 = []float64{350588081.147922, 29854134.6432052, -136870369.169738}
	var rsun, rmoon [3]float64
	var erpv [5]float64
	var i int
	assert := assert.New(t)

	gnssgo.SunMoonPos(gnssgo.Epoch2Time(ep1), erpv[:], rsun[:], rmoon[:], nil)
	fmt.Printf("X_sun =%15.0f %15.0f %7.4f\n", rsun[0], rs[0], (rsun[0]-rs[0])/rsun[0])
	fmt.Printf("Y_sun =%15.0f %15.0f %7.4f\n", rsun[1], rs[1], (rsun[1]-rs[1])/rsun[1])
	fmt.Printf("Z_sun =%15.0f %15.0f %7.4f\n", rsun[2], rs[2], (rsun[2]-rs[2])/rsun[2])
	fmt.Printf("X_moon=%15.0f %15.0f %7.4f\n", rmoon[0], rm[0], (rmoon[0]-rm[0])/rmoon[0])
	fmt.Printf("Y_moon=%15.0f %15.0f %7.4f\n", rmoon[1], rm[1], (rmoon[1]-rm[1])/rmoon[1])
	fmt.Printf("Z_moon=%15.0f %15.0f %7.4f\n", rmoon[2], rm[2], (rmoon[2]-rm[2])/rmoon[2])

	for i = 0; i < 3; i++ {
		assert.True(math.Abs((rsun[i]-rs[i])/rsun[i]) < 0.03)
		assert.True(math.Abs((rmoon[i]-rm[i])/rmoon[i]) < 0.03)
	}
}

/* tidedisp() */
func Test_ppputest3(t *testing.T) {
	var ep1 []float64 = []float64{2010, 6, 7, 1, 2, 3}
	var rr []float64 = []float64{-3957198.431, 3310198.621, 3737713.474} /* TSKB */
	var dp []float64 = []float64{-0.05294, 0.075607, 0.03644}
	var dr [3]float64 = [3]float64{0}
	var i int
	assert := assert.New(t)

	gnssgo.TideDisp(gnssgo.Epoch2Time(ep1), rr, 1, nil, nil, dr[:])

	fmt.Printf("X_disp=%8.5f %8.5f %8.5f\n", dr[0], dp[0], dr[0]-dp[0])
	fmt.Printf("Y_disp=%8.5f %8.5f %8.5f\n", dr[1], dp[1], dr[1]-dp[1])
	fmt.Printf("Z_disp=%8.5f %8.5f %8.5f\n", dr[2], dp[2], dr[2]-dp[2])

	for i = 0; i < 3; i++ {
		assert.True(math.Abs(dr[i]-dp[i]) < 0.001)
	}
}
