/*------------------------------------------------------------------------------
* rtklib unit test driver : atomospheric models
*-----------------------------------------------------------------------------*/

package gnss_test

import (
	"gnssgo"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* ionmodel() */
func Test_atmosutest1(t *testing.T) {
	var e1 []float64 = []float64{2007, 1, 16, 1, 0, 0}
	var e2 []float64 = []float64{2007, 1, 16, 13, 0, 0}
	var e3 []float64 = []float64{2007, 1, 16, 22, 0, 0}
	var ion1 []float64 = []float64{
		0.2e-7, -0.8e-8, -0.5e-7, 0.1e-6, 0.2e+6, 0.2e+6, -0.1e+6, -0.1e+7}
	var ion2 []float64 = []float64{
		0.2e-7, -0.8e-8, -0.5e-7, 0.1e-6, 0.2e+6, 0.2e+6, -0.1e+6, -0.1e+7}
	D2R := gnssgo.D2R
	var pos1 []float64 = []float64{35 * D2R, 140 * D2R, 100.0}
	var pos2 []float64 = []float64{-80 * D2R, -170 * D2R, 1000.0}
	var pos3 []float64 = []float64{10 * D2R, 30 * D2R, 0.0}
	var azel1 []float64 = []float64{60 * D2R, 75 * D2R}
	var azel2 []float64 = []float64{190 * D2R, 3 * D2R}
	var azel3 []float64 = []float64{350 * D2R, 60 * D2R}
	var azel4 []float64 = []float64{0 * D2R, 90 * D2R}
	t1 := gnssgo.Epoch2Time(e1)
	t2 := gnssgo.Epoch2Time(e2)
	t3 := gnssgo.Epoch2Time(e3)
	var dion float64
	assert := assert.New(t)

	dion = gnssgo.IonModel(t1, ion1, pos1, azel1)
	assert.True(math.Abs(dion-6.73590532099438) < 1e-8)

	dion = gnssgo.IonModel(t1, ion1, pos2, azel1)
	assert.True(math.Abs(dion-3.56895382197387) < 1e-8)

	dion = gnssgo.IonModel(t1, ion1, pos3, azel1)
	assert.True(math.Abs(dion-3.80716435655161) < 1e-8)

	dion = gnssgo.IonModel(t2, ion1, pos1, azel1)
	assert.True(math.Abs(dion-5.21796954585452) < 1e-8)

	dion = gnssgo.IonModel(t3, ion1, pos1, azel1)
	assert.True(math.Abs(dion-5.90190539264777) < 1e-8)

	dion = gnssgo.IonModel(t1, ion1, pos1, azel2)
	assert.True(math.Abs(dion-21.6345415123632) < 1e-8)

	dion = gnssgo.IonModel(t1, ion1, pos1, azel3)
	assert.True(math.Abs(dion-7.33844278822561) < 1e-8)

	dion = gnssgo.IonModel(t1, ion1, pos1, azel4)
	assert.True(math.Abs(dion-6.58339711400694) < 1e-8)

	dion = gnssgo.IonModel(t1, ion2, pos1, azel1)
	assert.True(math.Abs(dion-6.73590532099438) < 1e-8)

}

/* gnssgo.TropModel */
func Test_atmosutest3(t *testing.T) {
	var time gnssgo.Gtime
	D2R := gnssgo.D2R
	var pos1 []float64 = []float64{35 * D2R, 140 * D2R, 100.0}
	var pos2 []float64 = []float64{-80 * D2R, -170 * D2R, 1000.0}
	var pos3 []float64 = []float64{-80 * D2R, -170 * D2R, 100000.0}
	var pos4 []float64 = []float64{-80 * D2R, -170 * D2R, -200.0}
	var azel1 []float64 = []float64{60 * D2R, 75 * D2R}
	var azel2 []float64 = []float64{190 * D2R, 3 * D2R}
	var azel3 []float64 = []float64{190 * D2R, -10 * D2R}
	var dtrp float64
	assert := assert.New(t)

	dtrp = gnssgo.TropModel(time, pos1, azel1, 0.5)
	assert.True(math.Abs(dtrp-2.44799870144088) < 1e-8)

	dtrp = gnssgo.TropModel(time, pos1, azel2, 0.5)
	assert.True(math.Abs(dtrp-45.1808916506163) < 1e-8)

	dtrp = gnssgo.TropModel(time, pos2, azel1, 0.5)
	assert.True(math.Abs(dtrp-2.17295817298152) < 1e-8)

	dtrp = gnssgo.TropModel(time, pos1, azel3, 0.0)
	assert.True(math.Abs(dtrp-0.00000000000000) < 1e-8)

	dtrp = gnssgo.TropModel(time, pos3, azel1, 0.0)
	assert.True(math.Abs(dtrp-0.00000000000000) < 1e-8)

	dtrp = gnssgo.TropModel(time, pos4, azel1, 0.0)
	assert.True(math.Abs(dtrp-0.00000000000000) < 1e-8)
}

func Test_atmosutest4(t *testing.T) {
	var mapfd, mapfw float64
	var e1 []float64 = []float64{2007, 1, 16, 6, 0, 0}
	var e2 []float64 = []float64{2030, 12, 31, 23, 59, 59}
	D2R := gnssgo.D2R
	var pos1 []float64 = []float64{35 * D2R, 140 * D2R, 100.0}
	var pos2 []float64 = []float64{-80 * D2R, -170 * D2R, 1000.0}
	var pos3 []float64 = []float64{10 * D2R, 30 * D2R, 0.0}
	var azel1 []float64 = []float64{60 * D2R, 75 * D2R}
	var azel2 []float64 = []float64{190 * D2R, 3 * D2R}
	var azel3 []float64 = []float64{350 * D2R, 60 * D2R}
	var azel4 []float64 = []float64{0 * D2R, 90 * D2R}
	assert := assert.New(t)
	t1 := gnssgo.Epoch2Time(e1)
	t2 := gnssgo.Epoch2Time(e2)

	mapfd = gnssgo.TropMapFunc(t1, pos1, azel1, &mapfw)
	assert.True(math.Abs(mapfd-1.035184526466435) < 1e-8)
	assert.True(math.Abs(mapfw-1.035233787448654) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos1, azel2, &mapfw)
	assert.True(math.Abs(mapfd-14.643271711748200) < 1e-8)
	assert.True(math.Abs(mapfw-16.455045694559484) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos1, azel3, &mapfw)
	assert.True(math.Abs(mapfd-1.154226397147367) < 1e-8)
	assert.True(math.Abs(mapfw-1.154481126139610) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos1, azel4, &mapfw)
	assert.True(math.Abs(mapfd-1.000000000000000) < 1e-8)
	assert.True(math.Abs(mapfw-1.000000000000000) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t2, pos1, azel1, &mapfw)
	assert.True(math.Abs(mapfd-1.035184415128022) < 1e-8)
	assert.True(math.Abs(mapfw-1.035233787448654) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos2, azel1, &mapfw)
	assert.True(math.Abs(mapfd-1.035186155749051) < 1e-8)
	assert.True(math.Abs(mapfw-1.035230548304367) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos3, azel1, &mapfw)
	assert.True(math.Abs(mapfd-1.035181919429758) < 1e-8)
	assert.True(math.Abs(mapfw-1.035233200318210) < 1e-8)

	mapfd = gnssgo.TropMapFunc(t1, pos1, azel1, nil)
	assert.True(math.Abs(mapfd-1.035184526466435) < 1e-8)
}
