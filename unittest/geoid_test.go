/*------------------------------------------------------------------------------
* rtklib unit test driver : geoid functions
*-----------------------------------------------------------------------------*/

package gnss_test

import (
	"gnssgo"
	"testing"

	"github.com/stretchr/testify/assert"
)

/* latitude, longitude, geoid height (m) */
/* reference : http://sps.unavco.org/geoid */
var D2R float64 = gnssgo.D2R
var poss [][3]float64 = [][3]float64{
	{90.001 * D2R, 80.000 * D2R, 0.000},
	{-90.001 * D2R, 80.000 * D2R, 0.000},
	{30.000 * D2R, 360.000 * D2R, 0.000},
	{-30.000 * D2R, -360.001 * D2R, 0.000},
	{-90.000 * D2R, 359.999 * D2R, -29.534},
	{90.000 * D2R, 80.000 * D2R, 13.606},
	{-90.000 * D2R, -60.000 * D2R, -29.534},
	{30.000 * D2R, -360.000 * D2R, 35.387},
	{-30.000 * D2R, 359.999 * D2R, 21.409},
	{10.000 * D2R, 45.000 * D2R, -20.486},
	{-60.123 * D2R, 135.123 * D2R, -33.152},
	{19.999 * D2R, 135.000 * D2R, 41.602},
	{50.001 * D2R, 135.000 * D2R, 20.555},
	{35.000 * D2R, 119.999 * D2R, 4.386},
	{35.000 * D2R, 150.001 * D2R, 14.779},
	{20.000 * D2R, 120.000 * D2R, 21.269},
	{50.000 * D2R, 150.000 * D2R, 20.277},
	{35.000 * D2R, 135.000 * D2R, 36.355},
	{45.402 * D2R, 141.750 * D2R, 27.229}, /* wakkanai */
	{24.454 * D2R, 122.942 * D2R, 21.652}, /* ishigaki */
	{33.120 * D2R, 139.797 * D2R, 43.170}, /* hachijo */
	{30.000 * D2R, 135.000 * D2R, 36.017}, /* taiheiyo */
	{0, 0, 0}}
var DATADIR string = "../../../../data/geoiddata/"
var file1 string = DATADIR + "WW15MGH.DAC"
var file2 string = DATADIR + "Und_min1x1_egm2008_isw=82_WGS84_TideFree_SE"
var file3 string = DATADIR + "Und_min2.5x2.5_egm2008_isw=82_WGS84_TideFree_SE"
var file4 string = DATADIR + "gsigeome.ver4"

/* opengeoid(), closegeoid() */
func Test_geoidutest1(t *testing.T) {
	var ret int
	assert := assert.New(t)

	ret = gnssgo.OpenGeoid(10, file1)
	assert.True(ret == 0) /* no model */
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_EGM96_M150, "../../../geoiddata/WW15MGH.DAA")
	assert.True(ret == 0) /* no file */
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_EMBEDDED, "")
	assert.True(ret == 1)
	gnssgo.CloseGeoid()
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_EGM96_M150, file1)
	assert.True(ret == 1)
	gnssgo.CloseGeoid()
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_EGM2008_M10, file2)
	assert.True(ret == 1)
	gnssgo.CloseGeoid()
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_EGM2008_M25, file3)
	assert.True(ret == 1)
	gnssgo.CloseGeoid()
	ret = gnssgo.OpenGeoid(gnssgo.GEOID_GSI2000_M15, file4)
	assert.True(ret == 1)
	gnssgo.CloseGeoid()
}
