/*------------------------------------------------------------------------------
* rtklib unit test driver : coordinates functions
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"gnssgo"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* gnssgo.Ecef2Pos() */
func Test_coordutest1(t *testing.T) {
	var r1 []float64 = []float64{0.0, 0.0, 0.0}
	var r2 []float64 = []float64{10000000.0, 0.0, 0.0}
	var r3 []float64 = []float64{0.0, 10000000.0, 0.0}
	var r4 []float64 = []float64{0.0, 0.0, 10000000.0}
	var r5 []float64 = []float64{0.0, 0.0, -10000000.0}
	var r6 []float64 = []float64{-3.5173197701e+06, 4.1316679161e+06, 3.3412651227e+06}
	var r7 []float64 = []float64{-3.5173197701e+06, 4.1316679161e+06, -3.3412651227e+06}
	var pos [3]float64
	PI := gnssgo.PI
	R2D := gnssgo.R2D
	assert := assert.New(t)
	gnssgo.Ecef2Pos(r1, pos[:])
	assert.True(pos[2] < 0.0)
	gnssgo.Ecef2Pos(r2, pos[:])
	assert.True(pos[0] == 0 && pos[1] == 0 && pos[2] > 0.0)
	gnssgo.Ecef2Pos(r3, pos[:])
	assert.True(pos[0] == 0 && math.Abs(pos[1]-PI/2) < 1e-6 && pos[2] > 0.0)
	gnssgo.Ecef2Pos(r4, pos[:])
	assert.True(math.Abs(pos[0]-PI/2) < 1e-6 && pos[2] > 0.0)
	gnssgo.Ecef2Pos(r5, pos[:])
	assert.True(math.Abs(pos[0]+PI/2) < 1e-6 && pos[2] > 0.0)
	gnssgo.Ecef2Pos(r6, pos[:])
	assert.True(math.Abs(pos[0]*R2D-3.1796021375e+01) < 1e-7 &&
		math.Abs(pos[1]*R2D-1.3040799917e+02) < 1e-7 &&
		math.Abs(pos[2]-6.8863206206e+01) < 1e-4)
	gnssgo.Ecef2Pos(r7, pos[:])
	assert.True(math.Abs(pos[0]*R2D+3.1796021375e+01) < 1e-7 &&
		math.Abs(pos[1]*R2D-1.3040799917e+02) < 1e-7 &&
		math.Abs(pos[2]-6.8863206206e+01) < 1e-4)
}

/* pos2ecef() */
func Test_coordutest2(t *testing.T) {
	var lat, lon, h float64
	var pos, posi, r [3]float64
	assert := assert.New(t)
	D2R := gnssgo.D2R
	R2D := gnssgo.R2D
	for lat = -90.0; lat <= 90.0; lat += 5.0 {
		for lon = -180.0; lon < 180.0; lon += 5.0 {
			for h = -10.0; h < 10000.0; h += 100.0 {
				pos[0] = lat * D2R
				pos[1] = lon * D2R
				pos[2] = h
				gnssgo.Pos2Ecef(pos[:], r[:])
				gnssgo.Ecef2Pos(r[:], posi[:])
				b := math.Abs(lon-posi[1]*R2D) < 1e-7
				if lat == 90.0 {
					b = true
				}
				assert.True(math.Abs(lat-posi[0]*R2D) < 1e-7)
				assert.True(lat == -90.0 || b)
				assert.True(math.Abs(h-posi[2]) < 1e-4)
			}
		}
	}
}

/* ecef2enu(), enu2ecef() */
func Test_coordutest3(t *testing.T) {
	D2R := gnssgo.D2R
	var pos1 []float64 = []float64{0.000 * D2R, 0.000 * D2R, 0.0}
	var pos2 []float64 = []float64{35.000 * D2R, 140.000 * D2R, 0.0}
	var r1 []float64 = []float64{1.0, 0.0, 0.0}
	var r2 []float64 = []float64{0.0, 1.0, 0.0}
	var r3 []float64 = []float64{0.0, 0.0, 1.0}
	var r4 []float64 = []float64{0.3, 0.4, 0.5}
	var e, r [3]float64
	assert := assert.New(t)
	gnssgo.Ecef2Enu(pos1, r1, e[:])
	assert.True(e[0] == 0.0 && e[1] == 0.0 && e[2] == 1.0)
	gnssgo.Ecef2Enu(pos1, r2, e[:])
	assert.True(e[0] == 1.0 && e[1] == 0.0 && e[2] == 0.0)
	gnssgo.Ecef2Enu(pos1, r3, e[:])
	assert.True(e[0] == 0.0 && e[1] == 1.0 && e[2] == 0.0)
	gnssgo.Ecef2Enu(pos2, r4, e[:])
	assert.True(math.Abs(e[0]+0.499254) < 1e-6 &&
		math.Abs(e[1]-0.393916) < 1e-6 &&
		math.Abs(e[2]-0.309152) < 1e-6)
	gnssgo.Enu2Ecef(pos2, e[:], r[:])
	assert.True(math.Abs(r[0]-r4[0]) < 1e-6 &&
		math.Abs(r[1]-r4[1]) < 1e-6 &&
		math.Abs(r[2]-r4[2]) < 1e-6)
}
