/*------------------------------------------------------------------------------
* rtklib unit test driver : ionex function
*-----------------------------------------------------------------------------*/
package gnss_test

import (
	"fmt"
	"gnssgo"
	"math"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
)

func dumptec(tec []gnssgo.Tec, n, level int, t *testing.T) {
	var p *gnssgo.Tec
	var s string
	var i, j, k, m int

	for i = 0; i < n; i++ {
		p = &tec[i]
		gnssgo.Time2Str(p.Time, &s, 3)
		fmt.Printf("(%2d) time =%s ndata=%d %d %d\n", i+1, s, p.Ndata[0], p.Ndata[1], p.Ndata[2])
		if level < 1 {
			continue
		}
		fmt.Printf("lats =%6.1f %6.1f %6.1f\n", p.Lats[0], p.Lats[1], p.Lats[2])
		fmt.Printf("lons =%6.1f %6.1f %6.1f\n", p.Lons[0], p.Lons[1], p.Lons[2])
		fmt.Printf("hgts =%6.1f %6.1f %6.1f\n", p.Hgts[0], p.Hgts[1], p.Hgts[2])
		fmt.Printf("data =\n")
		for j = 0; j < p.Ndata[2]; j++ { /* hgt */
			for k = 0; k < p.Ndata[0]; k++ { /* lat */
				fmt.Printf("lat=%.1f lon=%.1f:%.1f hgt=%.1f\n", p.Lats[0]+float64(k)*p.Lats[2],
					p.Lons[0], p.Lons[1], p.Hgts[0]+float64(j)*p.Hgts[2])
				for m = 0; m < p.Ndata[1]; m++ { /* lon */
					if m > 0 && m%16 == 0 {
						fmt.Printf("\n")
					}
					fmt.Printf("%5.1f ", p.Data[k+p.Ndata[0]*(m+p.Ndata[1]*j)])
				}
				fmt.Printf("\n")
			}
			fmt.Printf("\n")
		}
		fmt.Printf("rms  =\n")
		for j = 0; j < p.Ndata[2]; j++ { /* hgt */
			for k = 0; k < p.Ndata[0]; k++ { /* lat */
				fmt.Printf("lat=%.1f lon=%.1f:%.1f hgt=%.1f\n", p.Lats[0]+float64(k)*p.Lats[2],
					p.Lons[0], p.Lons[1], p.Hgts[0]+float64(j)*p.Hgts[2])
				for m = 0; m < p.Ndata[1]; m++ { /* lon */
					if m > 0 && m%16 == 0 {
						fmt.Printf("\n")
					}
					fmt.Printf("%5.1f ", p.Rms[k+p.Ndata[0]*(m+p.Ndata[1]*j)])
				}
				fmt.Printf("\n")
			}
			fmt.Printf("\n")
		}
	}
}
func dumpdcb(nav *gnssgo.Nav, t *testing.T) {
	var i int
	fmt.Printf("dcbs=\n")
	for i = 0; i < len(nav.CBias); i++ {
		fmt.Printf("%3d: P1-P2=%6.3f\n", i+1, nav.CBias[i][0]/gnssgo.CLIGHT*1e9) /* ns */
	}
}

/* readtec() */
func Test_ionexutest1(t *testing.T) {
	var file1 string = "../data/sp3/igrg3380.10j"
	var file2 string = "../data/sp3/igrg3380.10i"
	var file3 string = "../data/sp3/igrg33*0.10i"
	var nav gnssgo.Nav
	assert := assert.New(t)

	fmt.Printf("file=%s\n", file1)
	nav.ReadTec(file1, 0)
	assert.True(nav.Nt() == 0)

	fmt.Printf("file=%s\n", file2)
	nav.ReadTec(file2, 0)
	assert.True(nav.Nt() == 13)
	dumptec(nav.Tec, nav.Nt(), 1, t)

	fmt.Printf("file=%s\n", file3)
	nav.ReadTec(file3, 0)
	assert.True(nav.Nt() == 25)
	dumptec(nav.Tec, nav.Nt(), 0, t)

	dumptec(nav.Tec, 1, 1, t)
	dumptec(nav.Tec[12:], 1, 1, t)
	dumpdcb(&nav, t)
}

/* nav.IonTec() 1 */
func Test_ionexutest2(t *testing.T) {
	var file3 string = "../data/sp3/igrg33*0.10i"
	var nav gnssgo.Nav
	var time1, time2, time3, time4 gnssgo.Gtime
	var ep1 []float64 = []float64{2010, 12, 4, 0, 0, 0}
	var ep2 []float64 = []float64{2010, 12, 5, 23, 59, 59}
	var ep3 []float64 = []float64{2010, 12, 3, 23, 59, 59} /* error */
	var ep4 []float64 = []float64{2010, 12, 6, 0, 0, 0}    /* error */
	D2R := gnssgo.D2R
	var pos1 []float64 = []float64{45.1 * D2R, 135.7 * D2R, 0.0}
	var pos2 []float64 = []float64{-45.1 * D2R, -170.7 * D2R, 0.0}
	var pos3 []float64 = []float64{-45.1 * D2R, 189.3 * D2R, 0.0}
	var pos4 []float64 = []float64{87.6 * D2R, 0.0 * D2R, 0.0}  /* out of grid */
	var pos5 []float64 = []float64{-87.6 * D2R, 0.0 * D2R, 0.0} /* out of grid */
	var azel1 []float64 = []float64{0.0, 90.0 * D2R}
	var azel2 []float64 = []float64{120.0, 30.0 * D2R}
	var azel3 []float64 = []float64{0.0, -0.1 * D2R} /* error */
	var delay1, var1, delay2, var2 float64
	var stat int
	assert := assert.New(t)

	time1 = gnssgo.Epoch2Time(ep1)
	time2 = gnssgo.Epoch2Time(ep2)
	time3 = gnssgo.Epoch2Time(ep3)
	time4 = gnssgo.Epoch2Time(ep4)

	nav.ReadTec(file3, 0)
	stat = nav.IonTec(time1, pos1, azel1, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time2, pos1, azel1, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time3, pos1, azel1, 1, &delay1, &var1)
	assert.True(stat == 0)
	stat = nav.IonTec(time4, pos1, azel1, 1, &delay1, &var1)
	assert.True(stat == 0)
	stat = nav.IonTec(time1, pos2, azel1, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time1, pos3, azel1, 1, &delay2, &var2)
	assert.True(stat == 1)
	assert.True(math.Abs(delay1-delay2) < 1e-4)
	assert.True(math.Abs(var1-var2) < 1e-8)
	stat = nav.IonTec(time1, pos4, azel1, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time1, pos5, azel1, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time1, pos1, azel2, 1, &delay1, &var1)
	assert.True(stat == 1)
	stat = nav.IonTec(time1, pos1, azel3, 1, &delay1, &var1)
	assert.True(stat == 1 && delay1 == 0.0)
}

/* iontec() 2 */
func Test_ionexutest3(t *testing.T) {
	var fp *os.File
	var file3 string = "../data/sp3/igrg33*0.10i"
	var nav gnssgo.Nav
	var time1 gnssgo.Gtime
	var ep1 []float64 = []float64{2010, 12, 5, 0, 0, 0}
	var delay, fvar float64
	var pos [3]float64
	var azel []float64 = []float64{0.0, gnssgo.PI / 2}
	var i, j int
	var err error
	assert := assert.New(t)
	D2R := gnssgo.D2R

	time1 = gnssgo.Epoch2Time(ep1)
	nav.ReadTec(file3, 0)

	fp, err = os.OpenFile("../data/testionex3.m", os.O_CREATE|os.O_WRONLY, os.ModeAppend|os.ModePerm)
	if err != nil {
		t.Log(err)
	}
	assert.NotEmpty(fp)

	fp.WriteString("tec=[\n")
	for i = 90; i >= -90; i -= 2 {
		for j = 0; j <= 360; j += 2 {
			pos[0] = float64(i) * D2R
			pos[1] = float64(j) * D2R
			if nav.IonTec(time1, pos[:], azel, 1, &delay, &fvar) != 0 {
				fp.WriteString(fmt.Sprintf("%4.2f ", delay))
			} else {
				fp.WriteString(fmt.Sprintf(" nan "))
			}
		}
		fp.WriteString(fmt.Sprintf("\n"))
	}
	fp.WriteString(fmt.Sprintf("];\n"))
	fp.Close()

	fp, _ = os.OpenFile("../data/testionex3.m", os.O_APPEND, 0666)
	assert.NotEmpty(fp)

	fp.WriteString(fmt.Sprintf("rms=[\n"))
	for i = 90; i >= -90; i -= 2 {
		for j = 0; j <= 360; j += 2 {
			pos[0] = float64(i) * D2R
			pos[1] = float64(j) * D2R
			if nav.IonTec(time1, pos[:], azel, 1, &delay, &fvar) != 0 {
				fp.WriteString(fmt.Sprintf("%4.2f ", math.Sqrt(fvar)))
			} else {
				fp.WriteString(fmt.Sprintf(" nan "))
			}
		}
		fp.WriteString(fmt.Sprintf("\n"))
	}
	fp.WriteString(fmt.Sprintf("];\n"))
	fp.Close()
}

/* iontec() 3 */
func Test_ionexutest4(t *testing.T) {
	var fp *os.File
	var file3 string = "../data/sp3/igrg33*0.10i"
	var nav gnssgo.Nav
	var time1 gnssgo.Gtime
	var ep1 []float64 = []float64{2010, 12, 3, 12, 0, 0}
	var delay, fvar float64
	D2R := gnssgo.D2R
	var pos [3]float64 = [3]float64{25 * D2R, 135 * D2R, 0}
	var azel []float64 = []float64{75 * D2R, 90 * D2R}
	var i int
	assert := assert.New(t)

	time1 = gnssgo.Epoch2Time(ep1)
	nav.ReadTec(file3, 0)

	fp, _ = os.OpenFile("../data/testionex4.m", os.O_CREATE|os.O_WRONLY, os.ModeAppend|os.ModePerm)
	assert.NotEmpty(fp)

	fp.WriteString(fmt.Sprintf("tec=[\n"))
	for i = 0; i <= 86400*3; i += 30 {
		if nav.IonTec(gnssgo.TimeAdd(time1, float64(i)), pos[:], azel, 1, &delay, &fvar) != 0 {
			fp.WriteString(fmt.Sprintf("%6d %5.3f %5.3f\n", i, delay, math.Sqrt(fvar)))
		} else {
			fp.WriteString(fmt.Sprintf("%6d  nan   nan\n", i))
		}
	}
	fp.WriteString(fmt.Sprintf("];\n"))
	fp.Close()
}
