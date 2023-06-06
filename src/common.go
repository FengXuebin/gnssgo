/*------------------------------------------------------------------------------
* commnon.go : realized rtklib common functions
*
*          Copyright (C) 2022-2025 by feng xuebin, All rights reserved.
*
 */
/*------------------------------------------------------------------------------
* rtkcmn.c : rtklib common functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* options : -DLAPACK   use LAPACK/BLAS
*           -DMKL      use Intel MKL
*           -DTRACE    enable debug trace
*           -DWIN32    use WIN32 API
*           -DNOCALLOC no use calloc for zero matrix
*           -DIERS_MODEL use GMF instead of NMF
*           -DDLL      built for shared library
*           -DCPUTIME_IN_GPST cputime operated in gpst
*
* references :
*     [1] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
*         7 March, 2006
*     [2] RTCA/DO-229C, Minimum operational performance standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         November 28, 2001
*     [3] M.Rothacher, R.Schmid, ANTEX: The Antenna Exchange Format Version 1.4,
*         15 September, 2010
*     [4] A.Gelb ed., Applied Optimal Estimation, The M.I.T Press, 1974
*     [5] A.E.Niell, Global mapping functions for the atmosphere delay at radio
*         wavelengths, Journal of geophysical research, 1996
*     [6] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 3.00, November 28, 2007
*     [7] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [8] China Satellite Navigation Office, BeiDou navigation satellite system
*         signal in space interface control document, open service signal B1I
*         (version 1.0), Dec 2012
*     [9] J.Boehm, A.Niell, P.Tregoning and H.Shuh, Global Mapping Function
*         (GMF): A new empirical mapping function base on numerical weather
*         model data, Geophysical Research Letters, 33, L07304, 2006
*     [10] GLONASS/GPS/Galileo/Compass/SBAS NV08C receiver series BINR interface
*         protocol specification ver.1.3, August, 2012
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/12 1.0 new
*           2007/03/06 1.1 input initial rover pos of pntpos()
*                          update only effective states of filter()
*                          fix bug of atan2() domain error
*           2007/04/11 1.2 add function antmodel()
*                          add gdop mask for pntpos()
*                          change constant MAXDTOE value
*           2007/05/25 1.3 add function execcmd(),expandpath()
*           2008/06/21 1.4 add funciton sortobs(),uniqeph(),screent()
*                          replace geodist() by sagnac correction way
*           2008/10/29 1.5 fix bug of ionospheric mapping function
*                          fix bug of seasonal variation term of tropmapf
*           2008/12/27 1.6 add function tickget(), sleepms(), tracenav(),
*                          xyz2enu(), satposv(), pntvel(), covecef()
*           2009/03/12 1.7 fix bug on error-stop when localtime() returns NULL
*           2009/03/13 1.8 fix bug on time adjustment for summer time
*           2009/04/10 1.9 add function adjgpsweek(),getbits(),getbitu()
*                          add function geph2pos()
*           2009/06/08 1.10 add function seph2pos()
*           2009/11/28 1.11 change function pntpos()
*                           add function tracegnav(),tracepeph()
*           2009/12/22 1.12 change default parameter of ionos std
*                           valid under second for timeget()
*           2010/07/28 1.13 fix bug in tropmapf()
*                           added api:
*                               obs2code(),code2obs(),cross3(),normv3(),
*                               gst2time(),time2gst(),time_str(),timeset(),
*                               deg2dms(),dms2deg(),searchpcv(),antmodel_s(),
*                               tracehnav(),tracepclk(),reppath(),reppaths(),
*                               createdir()
*                           changed api:
*                               readpcv(),
*                           deleted api:
*                               uniqeph()
*           2010/08/20 1.14 omit to include mkl header files
*                           fix bug on chi-sqr(n) table
*           2010/12/11 1.15 added api:
*                               freeobs(),freenav(),ionppp()
*           2011/05/28 1.16 fix bug on half-hour offset by time2epoch()
*                           added api:
*                               uniqnav()
*           2012/06/09 1.17 add a leap second after 2012-6-30
*           2012/07/15 1.18 add api setbits(),setbitu(),utc2gmst()
*                           fix bug on interpolation of antenna pcv
*                           fix bug on str2num() for string with over 256 char
*                           add api readblq(),satexclude(),setcodepri(),
*                           getcodepri()
*                           change api obs2code(),code2obs(),antmodel()
*           2012/12/25 1.19 fix bug on satwavelen(),code2obs(),obs2code()
*                           add api testsnr()
*           2013/01/04 1.20 add api gpst2bdt(),bdt2gpst(),bdt2time(),time2bdt()
*                           readblq(),readerp(),geterp(),crc16()
*                           change api eci2ecef(),sunmoonpos()
*           2013/03/26 1.21 tickget() uses clock_gettime() for linux
*           2013/05/08 1.22 fix bug on nutation coefficients for ast_args()
*           2013/06/02 1.23 add #ifdef for undefined CLOCK_MONOTONIC_RAW
*           2013/09/01 1.24 fix bug on interpolation of satellite antenna pcv
*           2013/09/06 1.25 fix bug on extrapolation of erp
*           2014/04/27 1.26 add SYS_LEO for satellite system
*                           add BDS L1 code for RINEX 3.02 and RTCM 3.2
*                           support BDS L1 in satwavelen()
*           2014/05/29 1.27 fix bug on obs2code() to search obs code table
*           2014/08/26 1.28 fix problem on output of uncompress() for tar file
*                           add function to swap trace file with keywords
*           2014/10/21 1.29 strtok() -> strtok_r() in expath() for thread-safe
*                           add bdsmodear in procopt_default
*           2015/03/19 1.30 fix bug on interpolation of erp values in geterp()
*                           add leap second insertion before 2015/07/01 00:00
*                           add api read_leaps()
*           2015/05/31 1.31 delete api windupcorr()
*           2015/08/08 1.32 add compile option CPUTIME_IN_GPST
*                           add api add_fatal()
*                           support usno leapsec.dat for api read_leaps()
*           2016/01/23 1.33 enable septentrio
*           2016/02/05 1.34 support GLONASS for savenav(), loadnav()
*           2016/06/11 1.35 delete trace() in reppath() to avoid deadlock
*           2016/07/01 1.36 support IRNSS
*                           add leap second before 2017/1/1 00:00:00
*           2016/07/29 1.37 rename api compress() -> rtk_uncompress()
*                           rename api crc16()    -> rtk_crc16()
*                           rename api crc24q()   -> rtk_crc24q()
*                           rename api crc32()    -> rtk_crc32()
*           2016/08/20 1.38 fix type incompatibility in win64 environment
*                           change constant _POSIX_C_SOURCE 199309 -> 199506
*           2016/08/21 1.39 fix bug on week overflow in time2gpst()/gpst2time()
*           2016/09/05 1.40 fix bug on invalid nav data read in readnav()
*           2016/09/17 1.41 suppress warnings
*           2016/09/19 1.42 modify api deg2dms() to consider numerical error
*           2017/04/11 1.43 delete EXPORT for global variables
*           2018/10/10 1.44 modify api satexclude()
*           2020/11/30 1.45 add API code2idx() to get freq-index
*                           add API code2freq() to get carrier frequency
*                           add API timereset() to reset current time
*                           modify API obs2code(), code2obs() and setcodepri()
*                           delete API satwavelen()
*                           delete API csmooth()
*                           delete global variable lam_carr[]
*                           compensate L3,L4,... PCVs by L2 PCV if no PCV data
*                            in input file by API readpcv()
*                           add support hatanaka-compressed RINEX files with
*                            extension ".crx" or ".CRX"
*                           update stream format strings table
*                           update obs code strings and priority table
*                           use integer types in stdint.h
*                           surppress warnings
*		    2022/05/31 1.0  rewrite rtkcmn.c with golang by fxb
*-----------------------------------------------------------------------------*/
// /* satellites, systems, codes functions --------------------------------------*/
// EXPORT int  satno   (int sys, int prn);
// EXPORT int  satsys  (int sat, int *prn);
// EXPORT int  satid2no(const char *id);
// EXPORT void satno2id(int sat, char *id);
// EXPORT uint8_t obs2code(const char *obs);
// EXPORT char *code2obs(uint8_t code);
// EXPORT double code2freq(int sys, uint8_t code, int fcn);
// EXPORT double sat2freq(int sat, uint8_t code, const nav_t *nav);
// EXPORT int  code2idx(int sys, uint8_t code);
// EXPORT int  satexclude(int sat, double var, int svh, const prcopt_t *opt);
// EXPORT int  testsnr(int base, int freq, double el, double snr,
//                     const snrmask_t *mask);
// EXPORT void setcodepri(int sys, int idx, const char *pri);
// EXPORT int  getcodepri(int sys, uint8_t code, const char *opt);

// /* matrix and vector functions -----------------------------------------------*/
// EXPORT double *mat  (int n, int m);
// EXPORT int    *imat (int n, int m);
// EXPORT double *zeros(int n, int m);
// EXPORT double *eye  (int n);
// EXPORT double dot (const double *a, const double *b, int n);
// EXPORT double norm(const double *a, int n);
// EXPORT void cross3(const double *a, const double *b, double *c);
// EXPORT int  normv3(const double *a, double *b);
// EXPORT void matcpy(double *A, const double *B, int n, int m);
// EXPORT void matmul(const char *tr, int n, int k, int m, double alpha,
//                    const double *A, const double *B, double beta, double *C);
// EXPORT int  matinv(double *A, int n);
// EXPORT int  solve (const char *tr, const double *A, const double *Y, int n,
//                    int m, double *X);
// EXPORT int  lsq   (const double *A, const double *y, int n, int m, double *x,
//                    double *Q);
// EXPORT int  filter(double *x, double *P, const double *H, const double *v,
//                    const double *R, int n, int m);
// EXPORT int  smoother(const double *xf, const double *Qf, const double *xb,
//                      const double *Qb, int n, double *xs, double *Qs);
// EXPORT void matprint (const double *A, int n, int m, int p, int q);
// EXPORT void matfprint(const double *A, int n, int m, int p, int q, FILE *fp);

// EXPORT void add_fatal(fatalfunc_t *func);

// /* time and string functions -------------------------------------------------*/
// EXPORT double  str2num(const char *s, int i, int n);
// EXPORT int     str2time(const char *s, int i, int n, gtime_t *t);
// EXPORT void    time2str(gtime_t t, char *str, int n);
// EXPORT gtime_t epoch2time(const double *ep);
// EXPORT void    time2epoch(gtime_t t, double *ep);
// EXPORT gtime_t gpst2time(int week, double sec);
// EXPORT double  time2gpst(gtime_t t, int *week);
// EXPORT gtime_t gst2time(int week, double sec);
// EXPORT double  time2gst(gtime_t t, int *week);
// EXPORT gtime_t bdt2time(int week, double sec);
// EXPORT double  time2bdt(gtime_t t, int *week);
// EXPORT char    *time_str(gtime_t t, int n);

// EXPORT gtime_t timeadd  (gtime_t t, double sec);
// EXPORT double  timediff (gtime_t t1, gtime_t t2);
// EXPORT gtime_t gpst2utc (gtime_t t);
// EXPORT gtime_t utc2gpst (gtime_t t);
// EXPORT gtime_t gpst2bdt (gtime_t t);
// EXPORT gtime_t bdt2gpst (gtime_t t);
// EXPORT gtime_t timeget  (void);
// EXPORT void    timeset  (gtime_t t);
// EXPORT void    timereset(void);
// EXPORT double  time2doy (gtime_t t);
// EXPORT double  utc2gmst (gtime_t t, double ut1_utc);
// EXPORT int read_leaps(const char *file);

// EXPORT int adjgpsweek(int week);
// EXPORT uint32_t tickget(void);
// EXPORT void sleepms(int ms);

// EXPORT int reppath(const char *path, char *rpath, gtime_t time, const char *rov,
//                    const char *base);
// EXPORT int reppaths(const char *path, char *rpaths[], int nmax, gtime_t ts,
//                     gtime_t te, const char *rov, const char *base);

// /* coordinates transformation ------------------------------------------------*/
// EXPORT void ecef2pos(const double *r, double *pos);
// EXPORT void pos2ecef(const double *pos, double *r);
// EXPORT void ecef2enu(const double *pos, const double *r, double *e);
// EXPORT void enu2ecef(const double *pos, const double *e, double *r);
// EXPORT void covenu  (const double *pos, const double *P, double *Q);
// EXPORT void covecef (const double *pos, const double *Q, double *P);
// EXPORT void xyz2enu (const double *pos, double *E);
// EXPORT void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst);
// EXPORT void deg2dms (double deg, double *dms, int ndec);
// EXPORT double dms2deg(const double *dms);

package gnssgo

import (
	"bufio"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

// return sign of x
func SQRS(x float64) float64 {
	if x < 0.0 {
		return -x * x
	} else {
		return x * x
	}
}
func SQR(x float64) float64 {
	return x * x
}

func SQR32(x float32) float32 {
	if x < 0.0 {
		return -x * x
	} else {
		return x * x
	}
}

func SQRT(x float64) float64 {
	if x < 0.0 {
		return 0.0
	} else {
		return math.Sqrt(x)
	}
}
func SQRT32(x float32) float32 {
	if x < 0.0 {
		return 0.0
	} else {
		return float32(math.Sqrt(float64(x)))
	}
}

const POLYCRC32 = 0xEDB88320 /* CRC32 polynomial */
const POLYCRC24Q = 0x1864CFB /* CRC24Q polynomial */

var MAX_VAR_EPH float64 = SQR(300.0) /* max variance eph to reject satellite (m^2) */

var chisqr [100]float64 = [100]float64{ /* chi-sqr(n) (alpha=0.001) */
	10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
	31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
	46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
	61.1, 62.5, 63.9, 65.2, 66.6, 68.0, 69.3, 70.7, 72.1, 73.4,
	74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0, 85.4, 86.7,
	88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100,
	101, 102, 103, 104, 105, 107, 108, 109, 110, 112,
	113, 114, 115, 116, 118, 119, 120, 122, 123, 125,
	126, 127, 128, 129, 131, 132, 133, 134, 135, 137,
	138, 139, 140, 142, 143, 144, 145, 147, 148, 149}

func DefaultProcOpt() PrcOpt {
	return PrcOpt{ /* defaults processing options */
		Mode: PMODE_SINGLE, SolType: 0, Nf: 2, NavSys: SYS_GPS, /* mode,soltype,nf,navsys */
		Elmin: 15.0 * D2R, SnrMask: SnrMask{}, /* elmin,snrmask */
		SatEph: 0, ModeAr: 1, GloModeAr: 1, BDSModeAr: 1, /* sateph,modear,glomodear,bdsmodear */
		MaxOut: 5, MinLock: 0, MinFix: 10, ArMaxIter: 1, /* maxout,minlock,minfix,armaxiter */
		IonoOpt: 0, TropOpt: 0, Dynamics: 0, TideCorr: 0, /* estion,esttrop,dynamics,tidecorr */
		NoIter: 1, CodeSmooth: 0, IntPref: 0, SbasCorr: 0, SbasSatSel: 0, /* niter,codesmooth,intpref,sbascorr,sbassatsel */
		RovPos: 0, RefPos: 0, /*  */
		eratio:     [NFREQ]float64{100.0, 100.0, 0.0},             /* eratio[] */
		Err:        [5]float64{100.0, 0.003, 0.003, 0.0, 1.0},     /* err[] */
		Std:        [3]float64{30.0, 0.03, 0.3},                   /* std[] */
		Prn:        [6]float64{1e-4, 1e-3, 1e-4, 1e-1, 1e-2, 0.0}, /* prn[] */
		SatClkStab: 5e-12,                                         /* sclkstab */
		ThresAr:    [8]float64{3.0, 0.9999, 0.25, 0.1, 0.05},      /* thresar */
		ElMaskAr:   0.0, ElMaskHold: 0.0, ThresSlip: 0.05,         /* elmaskar,elmaskhold,thresslip */
		MaxTmDiff: 30.0, MaxInno: 30.0, MaxGdop: 30.0, /* maxtdiff,maxinno,maxgdop */
		Baseline: [2]float64{0}, Ru: [3]float64{0}, Rb: [3]float64{0} /* baseline,ru,rb */}
}

func DefaultSolOpt() SolOpt {
	return SolOpt{ /* defaults solution output options */
		Posf: SOLF_LLH, TimeS: TIMES_GPST, TimeF: 1, TimeU: 3, /* posf,times,timef,timeu */
		DegF: 0, OutHead: 1, OutOpt: 0, OutVel: 0, Datum: 0, Height: 0, Geoid: 0, /* degf,outhead,outopt,outvel,datum,height,geoid */
		SolStatic: 0, SStat: 0, Trace: 0, /* solstatic,sstat,trace */
		NmeaIntv: [2]float64{0.0, 0.0}, /* nmeaintv */
		Sep:      "", Prog: ""}         /* separator/program name */
}

var FormatStrs [32]string = [32]string{ /* stream format strings */
	"RTCM 2",         /*  0 */
	"RTCM 3",         /*  1 */
	"NovAtel OEM7",   /*  2 */
	"NovAtel OEM3",   /*  3 */
	"u-blox UBX",     /*  4 */
	"Superstar II",   /*  5 */
	"Hemisphere",     /*  6 */
	"SkyTraq",        /*  7 */
	"Javad GREIS",    /*  8 */
	"NVS BINR",       /*  9 */
	"BINEX",          /* 10 */
	"Trimble RT17",   /* 11 */
	"Septentrio SBF", /* 12 */
	"RINEX",          /* 13 */
	"SP3",            /* 14 */
	"RINEX CLK",      /* 15 */
	"SBAS",           /* 16 */
	"NMEA 0183",      /* 17 */
	""}

var obscodes []string = []string{ /* observation code strings */

	"", "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E", /*  0- 9 */
	"1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P", /* 10-19 */
	"2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X", /* 20-29 */
	"6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8L", "8Q", "8X", /* 30-39 */
	"2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A", /* 40-49 */
	"5B", "5C", "9A", "9B", "9C", "9X", "1D", "5D", "5P", "5Z", /* 50-59 */
	"6E", "7D", "7P", "7Z", "8D", "8P", "4A", "4B", "4X", ""} /* 60-69 */
/* global variables ----------------------------------------------------------*/
var navsys []int = []int{ /* system codes */
	SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP, SYS_IRN, 0}

var codepris [7][MAXFREQ]string = [7][MAXFREQ]string{ /* code priority for each freq-index */
	/*    0         1          2          3         4         5     */
	{"CPYWMNSL", "PYWCMNDLSX", "IQX", "", "", "", ""},   /* GPS */
	{"CPABX", "PCABX", "IQX", "", "", "", ""},           /* GLO */
	{"CABXZ", "IQX", "IQX", "ABCXZ", "IQX", "", ""},     /* GAL */
	{"CLSXZ", "LSX", "IQXDPZ", "LSXEZ", "", "", ""},     /* QZS */
	{"C", "IQX", "", "", "", "", ""},                    /* SBS */
	{"IQXDPAN", "IQXDPZ", "DPX", "IQXA", "DPX", "", ""}, /* BDS */
	{"ABCX", "ABCX", "", "", "", "", ""}}                /* IRN */

//var fatalfunc *fatalfunc = nil /* fatal callback function */

/* crc tables generated by util/gencrc ---------------------------------------*/
var tbl_CRC16 []uint16 = []uint16{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0}

var tbl_CRC24Q []uint32 = []uint32{
	0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
	0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
	0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
	0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
	0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
	0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
	0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
	0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
	0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
	0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
	0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
	0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
	0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
	0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
	0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
	0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
	0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
	0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
	0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
	0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
	0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
	0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
	0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
	0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
	0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
	0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
	0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
	0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
	0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
	0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
	0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
	0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538}

/* function prototypes -------------------------------------------------------*/
/* #ifdef MKL
#define LAPACK
#define dgemm_      dgemm
#define dgetrf_     dgetrf
#define dgetri_     dgetri
#define dgetrs_     dgetrs
#endif
#ifdef LAPACK
extern void dgemm_(char *, char *, int *, int *, int *, double *, double *,
                   int *, double *, int *, double *, double *, int *);
extern void dgetrf_(int *, int *, double *, int *, int *, int *);
extern void dgetri_(int *, double *, int *, int *, double *, int *, int *);
extern void dgetrs_(char *, int *, int *, double *, int *, int *, double *,
                    int *, int *);
#endif

#ifdef IERS_MODEL
extern  int gmf_(double *mjd, double *lat, double *lon, double *hgt, double *zd,
                double *gmfh, double *gmfw);
#endif */

var (
	gpst0 = [6]float64{1980, 1, 6, 0, 0, 0}  /* gps time reference */
	gst0  = [6]float64{1999, 8, 22, 0, 0, 0} /* galileo system time reference */
	bdt0  = [6]float64{2006, 1, 1, 0, 0, 0} /* beidou time reference */)

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
func SatNo(sys int, prn int) int {
	if prn <= 0 {
		return 0
	}
	switch sys {
	case SYS_GPS:
		if prn < MINPRNGPS || MAXPRNGPS < prn {
			return 0
		}
		return prn - MINPRNGPS + 1
	case SYS_GLO:
		if prn < MINPRNGLO || MAXPRNGLO < prn {
			return 0
		}
		return NSATGPS + prn - MINPRNGLO + 1
	case SYS_GAL:
		if prn < MINPRNGAL || MAXPRNGAL < prn {
			return 0
		}
		return NSATGPS + NSATGLO + prn - MINPRNGAL + 1
	case SYS_QZS:
		if prn < MINPRNQZS || MAXPRNQZS < prn {
			return 0
		}
		return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1
	case SYS_CMP:
		if prn < MINPRNCMP || MAXPRNCMP < prn {
			return 0
		}
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1
	case SYS_IRN:
		if prn < MINPRNIRN || MAXPRNIRN < prn {
			return 0
		}
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNIRN + 1
	case SYS_LEO:
		if prn < MINPRNLEO || MAXPRNLEO < prn {
			return 0
		}
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + prn - MINPRNLEO + 1
	case SYS_SBS:
		if prn < MINPRNSBS || MAXPRNSBS < prn {
			return 0
		}
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATLEO + prn - MINPRNSBS + 1
	}
	return 0
}

/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
func SatSys(sat int, prn *int) int {
	var sys = SYS_NONE
	if sat <= 0 || MAXSAT < sat {
		sat = 0
	} else if sat <= NSATGPS {
		sys = SYS_GPS
		sat += MINPRNGPS - 1
	} else if sat -= NSATGPS; sat <= NSATGLO {
		sys = SYS_GLO
		sat += MINPRNGLO - 1
	} else if sat -= NSATGLO; sat <= NSATGAL {
		sys = SYS_GAL
		sat += MINPRNGAL - 1
	} else if sat -= NSATGAL; sat <= NSATQZS {
		sys = SYS_QZS
		sat += MINPRNQZS - 1
	} else if sat -= NSATQZS; sat <= NSATCMP {
		sys = SYS_CMP
		sat += MINPRNCMP - 1
	} else if sat -= NSATCMP; sat <= NSATIRN {
		sys = SYS_IRN
		sat += MINPRNIRN - 1
	} else if sat -= NSATIRN; sat <= NSATLEO {
		sys = SYS_LEO
		sat += MINPRNLEO - 1
	} else if sat -= NSATLEO; sat <= NSATSBS {
		sys = SYS_SBS
		sat += MINPRNSBS - 1
	} else {
		sat = 0
	}

	if prn != nil {
		*prn = sat
	}
	return sys
}

/* satellite id to satellite number --------------------------------------------
* convert satellite id to satellite number
* args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
* return : satellite number (0: error)
* notes  : 120-142 and 193-199 are also recognized as sbas and qzss
*-----------------------------------------------------------------------------*/
func SatId2No(id string) int {
	var (
		sys, prn int
		code     rune
	)

	if ret, _ := fmt.Sscanf(id, "%d", &prn); ret == 1 {
		if MINPRNGPS <= prn && prn <= MAXPRNGPS {
			sys = SYS_GPS
		} else if MINPRNSBS <= prn && prn <= MAXPRNSBS {
			sys = SYS_SBS
		} else if MINPRNQZS <= prn && prn <= MAXPRNQZS {
			sys = SYS_QZS
		} else {
			return 0
		}
		return SatNo(sys, prn)
	}
	if ret, _ := fmt.Sscanf(id, "%c%d", &code, &prn); ret < 2 {
		return 0
	}

	switch code {
	case 'G':
		sys = SYS_GPS
		prn += MINPRNGPS - 1
	case 'R':
		sys = SYS_GLO
		prn += MINPRNGLO - 1
	case 'E':
		sys = SYS_GAL
		prn += MINPRNGAL - 1
	case 'J':
		sys = SYS_QZS
		prn += MINPRNQZS - 1
	case 'C':
		sys = SYS_CMP
		prn += MINPRNCMP - 1
	case 'I':
		sys = SYS_IRN
		prn += MINPRNIRN - 1
	case 'L':
		sys = SYS_LEO
		prn += MINPRNLEO - 1
	case 'S':
		sys = SYS_SBS
		prn += 100
	default:
		return 0
	}
	return SatNo(sys, prn)
}

/* satellite number to satellite id --------------------------------------------
* convert satellite number to satellite id
* args   : int    sat       I   satellite number
*          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
* return : none
*-----------------------------------------------------------------------------*/
func SatNo2Id(sat int, id *string) {
	var prn int
	if id == nil {
		return
	}

	switch SatSys(sat, &prn) {
	case SYS_GPS:
		*id = fmt.Sprintf("G%02d", prn-MINPRNGPS+1)
		return
	case SYS_GLO:
		*id = fmt.Sprintf("R%02d", prn-MINPRNGLO+1)
		return
	case SYS_GAL:
		*id = fmt.Sprintf("E%02d", prn-MINPRNGAL+1)
		return
	case SYS_QZS:
		*id = fmt.Sprintf("J%02d", prn-MINPRNQZS+1)
		return
	case SYS_CMP:
		*id = fmt.Sprintf("C%02d", prn-MINPRNCMP+1)
		return
	case SYS_IRN:
		*id = fmt.Sprintf("I%02d", prn-MINPRNIRN+1)
		return
	case SYS_LEO:
		*id = fmt.Sprintf("L%02d", prn-MINPRNLEO+1)
		return
	case SYS_SBS:
		*id = fmt.Sprintf("%03d", prn)
		return
	}
	*id = ""
}

/* test excluded satellite -----------------------------------------------------
* test excluded satellite
* args   : int    sat       I   satellite number
*          double var       I   variance of ephemeris (m^2)
*          int    svh       I   sv health flag
*          prcopt_t *opt    I   processing options (NULL: not used)
* return : status (1:excluded,0:not excluded)
*-----------------------------------------------------------------------------*/
func SatExclude(sat int, variant float64, svh int, opt *PrcOpt) int {
	var sys = SatSys(sat, nil)

	if svh < 0 {
		return 1 /* ephemeris unavailable */
	}

	if opt != nil {
		if opt.ExSats[sat-1] == 1 {
			return 1 /* excluded satellite */
		}
		if opt.ExSats[sat-1] == 2 {
			return 0 /* included satellite */
		}
		if sys&opt.NavSys == 0 {
			return 1 /* unselected sat sys */
		}
	}
	if sys == SYS_QZS {
		svh &= 0xFE /* mask QZSS LEX health */
	}
	if svh != 0 {
		Trace(2, "unhealthy satellite: sat=%3d svh=%02X\n", sat, svh)
		return 1
	}
	if variant > MAX_VAR_EPH {
		Trace(2, "invalid ura satellite: sat=%3d ura=%.2f\n", sat, math.Sqrt(variant))
		return 1
	}
	return 0
}

/* test SNR mask ---------------------------------------------------------------
* test SNR mask
* args   : int    base      I   rover or base-station (0:rover,1:base station)
*          int    idx       I   frequency index (0:L1,1:L2,2:L3,...)
*          double el        I   elevation angle (rad)
*          double snr       I   C/N0 (dBHz)
*          snrmask_t *mask  I   SNR mask
* return : status (1:masked,0:unmasked)
*-----------------------------------------------------------------------------*/
func TestSnr(base int, idx int, el float64, snr float64, mask *SnrMask) int {
	var (
		minsnr, a float64
		i         int
	)

	if mask.ena[base] == 0 || idx < 0 || idx >= NFREQ {
		return 0
	}

	a = (el*R2D + 5.0) / 10.0
	i = int(math.Floor(a))
	a -= float64(i)
	if i < 1 {
		minsnr = mask.mask[idx][0]
	} else if i > 8 {
		minsnr = mask.mask[idx][8]
	} else {
		minsnr = (1.0-a)*mask.mask[idx][i-1] + a*mask.mask[idx][i]
	}

	if snr < minsnr {
		return 1
	}

	return 0
}

/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str      I   obs code string ("1C","1P","1Y",...)
* return : obs code (CODE_???)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
func Obs2Code(obs string) uint8 {
	var i int

	for i = 1; len(obscodes[i]) > 0; i++ {
		if strings.Compare(obscodes[i], obs) != 0 {
			continue
		}
		return uint8(i)
	}
	return CODE_NONE
}

/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : uint8_t code     I   obs code (CODE_???)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
func Code2Obs(code uint8) string {
	if code <= CODE_NONE || MAXCODE < code {
		return ""
	}
	return obscodes[code]
}

/* GPS obs code to frequency -------------------------------------------------*/
func Code2Freq_GPS(code uint8, freq *float64) int {
	obs := Code2Obs(code)

	if len(obs) > 0 {
		switch obs[0] {
		case '1':
			*freq = FREQ1
			return 0 /* L1 */
		case '2':
			*freq = FREQ2
			return 1 /* L2 */
		case '5':
			*freq = FREQ5
			return 2 /* L5 */
		}
	}
	return -1
}

/* GLONASS obs code to frequency ---------------------------------------------*/
func Code2Freq_GLO(code uint8, fcn int, freq *float64) int {
	obs := Code2Obs(code)

	if fcn < (-7) || fcn > 6 {
		return -1
	}

	if len(obs) > 0 {
		switch obs[0] {
		case '1':
			*freq = FREQ1_GLO + DFRQ1_GLO*float64(fcn)
			return 0 /* G1 */
		case '2':
			*freq = FREQ2_GLO + DFRQ2_GLO*float64(fcn)
			return 1 /* G2 */
		case '3':
			*freq = FREQ3_GLO
			return 2 /* G3 */
		case '4':
			*freq = FREQ1a_GLO
			return 0 /* G1a */
		case '6':
			*freq = FREQ2a_GLO
			return 1 /* G2a */
		}
	}
	return -1
}

/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
func Str2Time(s string, i, n int, t *Gtime) int {
	var ep []float64 = []float64{0, 0, 0, 0, 0, 0}

	if i < 0 || len(s) < i || i+n > len(s) {
		return -1
	}
	s = s[i : i+n]
	n, _ = fmt.Sscanf(s, "%f %f %f %f %f %f", &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5])
	if n < 6 {
		return -1
	}
	if ep[0] < 100.0 {
		if ep[0] < 80.0 {
			ep[0] += 2000.0
		} else {
			ep[0] += 1900.0
		}
	}
	*t = Epoch2Time(ep)
	return 0
}

/* Galileo obs code to frequency ---------------------------------------------*/
func Code2Freq_GAL(code uint8, freq *float64) int {

	obs := Code2Obs(code)
	if len(obs) < 1 {
		return -1
	}
	switch obs[0] {
	case '1':
		*freq = FREQ1
		return 0 /* E1 */
	case '7':
		*freq = FREQ7
		return 1 /* E5b */
	case '5':
		*freq = FREQ5
		return 2 /* E5a */
	case '6':
		*freq = FREQ6
		return 3 /* E6 */
	case '8':
		*freq = FREQ8
		return 4 /* E5ab */
	}
	return -1
}

/* QZSS obs code to frequency ------------------------------------------------*/
func Code2Freq_QZS(code uint8, freq *float64) int {
	obs := Code2Obs(code)
	if len(obs) < 1 {
		return -1
	}

	switch obs[0] {
	case '1':
		*freq = FREQ1
		return 0 /* L1 */
	case '2':
		*freq = FREQ2
		return 1 /* L2 */
	case '5':
		*freq = FREQ5
		return 2 /* L5 */
	case '6':
		*freq = FREQ6
		return 3 /* L6 */
	}
	return -1
}

/* SBAS obs code to frequency ------------------------------------------------*/
func Code2Freq_SBS(code uint8, freq *float64) int {
	obs := Code2Obs(code)
	if len(obs) < 1 {
		return -1
	}

	switch obs[0] {
	case '1':
		*freq = FREQ1
		return 0 /* L1 */
	case '5':
		*freq = FREQ5
		return 1 /* L5 */
	}
	return -1
}

/* BDS obs code to frequency -------------------------------------------------*/
func Code2Freq_BDS(code uint8, freq *float64) int {
	obs := Code2Obs(code)
	if len(obs) < 1 {
		return -1
	}

	switch obs[0] {
	case '1':
		*freq = FREQ1
		return 0 /* B1C */
	case '2':
		*freq = FREQ1_CMP
		return 0 /* B1I */
	case '7':
		*freq = FREQ2_CMP
		return 1 /* B2I/B2b */
	case '5':
		*freq = FREQ5
		return 2 /* B2a */
	case '6':
		*freq = FREQ3_CMP
		return 3 /* B3 */
	case '8':
		*freq = FREQ8
		return 4 /* B2ab */
	}
	return -1
}

/* NavIC obs code to frequency -----------------------------------------------*/
func Code2Freq_IRN(code uint8, freq *float64) int {
	obs := Code2Obs(code)
	if len(obs) < 1 {
		return -1
	}

	switch obs[0] {
	case '5':
		*freq = FREQ5
		return 0 /* L5 */
	case '9':
		*freq = FREQ9
		return 1 /* S */
	}
	return -1
}

/* system and obs code to frequency index --------------------------------------
* convert system and obs code to frequency index
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
* return : frequency index (-1: error)
*                       0     1     2     3     4
*           --------------------------------------
*            GPS       L1    L2    L5     -     -
*            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
*            Galileo   E1    E5b   E5a   E6   E5ab
*            QZSS      L1    L2    L5    L6     -
*            SBAS      L1     -    L5     -     -
*            BDS       B1    B2    B2a   B3   B2ab (B1=B1I,B1C,B2=B2I,B2b)
*            NavIC     L5     S     -     -     -
*-----------------------------------------------------------------------------*/
func Code2Idx(sys int, code uint8) int {
	var freq float64

	switch sys {
	case SYS_GPS:
		return Code2Freq_GPS(code, &freq)
	case SYS_GLO:
		return Code2Freq_GLO(code, 0, &freq)
	case SYS_GAL:
		return Code2Freq_GAL(code, &freq)
	case SYS_QZS:
		return Code2Freq_QZS(code, &freq)
	case SYS_SBS:
		return Code2Freq_SBS(code, &freq)
	case SYS_CMP:
		return Code2Freq_BDS(code, &freq)
	case SYS_IRN:
		return Code2Freq_IRN(code, &freq)
	}
	return -1
}

/* system and obs code to frequency --------------------------------------------
* convert system and obs code to carrier frequency
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
*          int    fcn       I   frequency channel number for GLONASS
* return : carrier frequency (Hz) (0.0: error)
*-----------------------------------------------------------------------------*/
func Code2Freq(sys int, code uint8, fcn int) float64 {
	var freq = 0.0

	switch sys {
	case SYS_GPS:
		Code2Freq_GPS(code, &freq)
	case SYS_GLO:
		Code2Freq_GLO(code, fcn, &freq)
	case SYS_GAL:
		Code2Freq_GAL(code, &freq)
	case SYS_QZS:
		Code2Freq_QZS(code, &freq)
	case SYS_SBS:
		Code2Freq_SBS(code, &freq)
	case SYS_CMP:
		Code2Freq_BDS(code, &freq)
	case SYS_IRN:
		Code2Freq_IRN(code, &freq)
	}
	return freq
}

/* satellite and obs code to frequency -----------------------------------------
* convert satellite and obs code to carrier frequency
* args   : int    sat       I   satellite number
*          uint8_t code     I   obs code (CODE_???)
*          nav_t  *nav_t    I   navigation data for GLONASS (NULL: not used)
* return : carrier frequency (Hz) (0.0: error)
*-----------------------------------------------------------------------------*/
func Sat2Freq(sat int, code uint8, nav *Nav) float64 {
	var i, fcn, sys, prn int

	sys = SatSys(sat, &prn)

	if sys == SYS_GLO {
		if nav == nil {
			return 0.0
		}
		for i = 0; i < nav.Ng(); i++ {
			if nav.Geph[i].Sat == sat {
				break
			}
		}
		if i < nav.Ng() {
			fcn = nav.Geph[i].Frq
		} else if nav.Glo_fcn[prn-1] > 0 {
			fcn = nav.Glo_fcn[prn-1] - 8
		} else {
			return 0.0
		}
	}
	return Code2Freq(sys, code, fcn)
}

/* set code priority -----------------------------------------------------------
* set code priority for multiple codes in a frequency
* args   : int    sys       I   system (or of SYS_???)
*          int    idx       I   frequency index (0- )
*          char   *pri      I   priority of codes (series of code characters)
*                               (higher priority precedes lower)
* return : none
*-----------------------------------------------------------------------------*/
func SetCodePri(sys, idx int, pri string) {
	Trace(4, "setcodepri:sys=%d idx=%d pri=%s\n", sys, idx, pri)

	if idx < 0 || idx >= MAXFREQ {
		return
	}
	if sys&SYS_GPS != 0 {
		codepris[0][idx] = pri
	}
	if sys&SYS_GLO != 0 {
		codepris[1][idx] = pri
	}
	if sys&SYS_GAL != 0 {
		codepris[2][idx] = pri
	}
	if sys&SYS_QZS != 0 {
		codepris[3][idx] = pri
	}
	if sys&SYS_SBS != 0 {
		codepris[4][idx] = pri
	}
	if sys&SYS_CMP != 0 {
		codepris[5][idx] = pri
	}
	if sys&SYS_IRN != 0 {
		codepris[6][idx] = pri
	}
}

/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys       I   system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
*          char   *opt      I   code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
func GetCodePri(sys int, code uint8, opt string) int {
	var (
		optstr   string
		obs, str string
		i, j, n  int
	)

	switch sys {
	case SYS_GPS:
		i = 0
		optstr = "-GL%2s"
	case SYS_GLO:
		i = 1
		optstr = "-RL%2s"
	case SYS_GAL:
		i = 2
		optstr = "-EL%2s"
	case SYS_QZS:
		i = 3
		optstr = "-JL%2s"
	case SYS_SBS:
		i = 4
		optstr = "-SL%2s"
	case SYS_CMP:
		i = 5
		optstr = "-CL%2s"
	case SYS_IRN:
		i = 6
		optstr = "-IL%2s"
	default:
		return 0
	}
	if j = Code2Idx(sys, code); j < 0 {
		return 0
	}
	obs = Code2Obs(code)

	/* parse code options */
	p := strings.Split(opt, "-")
	for _, q := range p {
		if n, _ = fmt.Sscanf(q, optstr, str); n < 1 || str[0] != obs[0] {
			continue
		}
		if str[1] == obs[1] {
			return 15
		} else {
			return 0
		}
	}
	/* search code priority */
	if n = strings.Index(codepris[i][j], string(obs[1])); n >= 0 {
		return 14 - n
	} else {
		return 0
	}
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : uint8_t *buff    I   byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
func GetBitU(buff []uint8, pos, len int) uint32 {
	var bits uint32

	for i := pos; i < pos+len; i++ {
		bits = (bits << 1) + uint32((buff[i/8]>>(7-i%8))&1)
	}
	return bits
}

func GetBits(buff []uint8, pos, len int) int32 {
	var bits = GetBitU(buff, pos, len)
	if len <= 0 || 32 <= len || bits&(1<<(len-1)) == 0 {
		return int32(bits)
	}
	return int32(bits | (math.MaxUint32 << len)) /* extend sign */
}

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : uint8_t *buff IO byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
*          [u]int32_t data  I   unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
func SetBitU(buff []uint8, pos, len int, data uint32) {
	var mask uint32 = 1 << (len - 1)
	if len <= 0 || 32 < len {
		return
	}
	for i := pos; i < pos+len; i, mask = i+1, mask>>1 {
		if data&mask > 0 {
			buff[i/8] |= 1 << (7 - i%8)
		} else {
			buff[i/8] &= ^(1 << (7 - i%8))
		}

	}
}

func SetBits(buff []uint8, pos, len int, data int32) {
	if data < 0 {
		data |= 1 << (len - 1)
	} else {
		data &= ^(1 << (len - 1)) /* set sign bit */
	}
	SetBitU(buff, pos, len, uint32(data))
}

/* crc-32 parity ---------------------------------------------------------------
* compute crc-32 parity for novatel raw
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-32 parity
* notes  : see NovAtel OEMV firmware manual 1.7 32-bit CRC
*-----------------------------------------------------------------------------*/
func Rtk_CRC32(buff []uint8, len int) uint32 {
	var crc uint32 = 0

	for i := 0; i < len; i++ {
		crc ^= uint32(buff[i])
		for j := 0; j < 8; j++ {
			if crc&1 > 0 {
				crc = uint32(uint64((crc >> 1)) ^ POLYCRC32)
			} else {
				crc >>= 1
			}
		}
	}
	return crc
}

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
func Rtk_CRC24q(buff []uint8, len int) uint32 {
	var crc uint32 = 0
	for i := 0; i < len; i++ {
		crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc>>16)^uint32(buff[i])]
	}
	return crc
}

/* crc-16 parity ---------------------------------------------------------------
* compute crc-16 parity for binex, nvs
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-16 parity
* notes  : see reference [10] A.3.
*-----------------------------------------------------------------------------*/
func Rtk_CRC16(buff []uint8, len int) uint16 {
	var crc uint16 = 0

	for i := 0; i < len; i++ {
		crc = (crc << 8) ^ tbl_CRC16[((crc>>8)^uint16(buff[i]))&0xFF]
	}
	return crc
}

/* decode navigation data word -------------------------------------------------
* check party and decode navigation data word
* args   : uint32_t word    I   navigation data word (2+30bit)
*                               (previous word D29*-30* + current word D1-30)
*          uint8_t *data    O   decoded navigation data without parity
*                               (8bitx3)
* return : status (1:ok,0:parity error)
* notes  : see reference [1] 20.3.5.2 user parity algorithm
*-----------------------------------------------------------------------------*/
func Decode_Word(word uint32, data []uint8) int {
	var hamming []uint32 = []uint32{
		0xBB1F3480, 0x5D8F9A40, 0xAEC7CD00, 0x5763E680, 0x6BB1F340, 0x8B7A89C0}
	var parity, w uint32 = 0, 0

	Trace(5, "decodeword: word=%08x\n", word)

	if word&0x40000000 > 0 {
		word = word ^ 0x3FFFFFC0
	}

	for i := 0; i < 6; i++ {
		parity = parity << 1
		for w = (word & hamming[i]) >> 6; w > 0; w = w >> 1 {
			parity = parity ^ (w & 1)
		}
	}
	if parity != (word & 0x3F) {
		return 0
	}

	for i := 0; i < 3; i++ {
		data[i] = uint8(word >> (22 - i*8))
	}
	return 1
}

/* new matrix ------------------------------------------------------------------
* allocate memory of matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
func Mat(n, m int) []float64 {
	var p []float64

	if n <= 0 || m <= 0 {
		return nil
	}
	p = make([]float64, n*m)
	return p
}

/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/

func IMat(n, m int) []int {
	var p []int

	if n <= 0 || m <= 0 {
		return nil
	}
	p = make([]int, n*m)
	return p
}

/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
func Zeros(n, m int) []float64 {
	var p []float64 = Mat(n, m)

	if p != nil {
		for n = n*m - 1; n >= 0; n-- {
			p[n] = 0.0
		}
	}

	return p
}

/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)
*-----------------------------------------------------------------------------*/
func Eye(n int) []float64 {
	var p []float64 = Zeros(n, n)
	if p != nil {
		for i := 0; i < n; i++ {
			p[i+i*n] = 1.0
		}
	}
	return p
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
func Dot(a, b []float64, n int) float64 {
	c := 0.0

	for n--; n >= 0; n-- {
		c += a[n] * b[n]
	}
	return c
}

/* euclid Norm -----------------------------------------------------------------
* euclid Norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
func Norm(a []float64, n int) float64 {
	return math.Sqrt(Dot(a, a, n))
}

/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
func Cross3(a, b, c []float64) {
	if len(a) > 3 || len(b) > 3 || len(c) > 3 {
		Trace(5, "cross3 argument length > 3")
	}
	c[0] = a[1]*b[2] - a[2]*b[1]
	c[1] = a[2]*b[0] - a[0]*b[2]
	c[2] = a[0]*b[1] - a[1]*b[0]
}

/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func NormV3(a, b []float64) int {
	r := Norm(a, 3)
	if r <= 0.0 {
		return 0
	}
	b[0] = a[0] / r
	b[1] = a[1] / r
	b[2] = a[2] / r
	return 1
}

/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
func MatCpy(A, B []float64, n, m int) {
	copy(A, B)
}

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
func MatMul(tr string, n, k, m int, alpha float64, A, B []float64, beta float64, C []float64) {
	var (
		d          float64
		i, j, x, f int
	)
	if tr[0] == 'N' {
		if tr[1] == 'N' {
			f = 1
		} else {
			f = 2
		}
	} else {
		if tr[1] == 'N' {
			f = 3
		} else {
			f = 4
		}
	}

	for i = 0; i < n; i++ {
		for j = 0; j < k; j++ {
			d = 0.0
			switch f {
			case 1:
				for x = 0; x < m; x++ {
					d += A[i+x*n] * B[x+j*m]
				}
			case 2:
				for x = 0; x < m; x++ {
					d += A[i+x*n] * B[j+x*k]
				}
			case 3:
				for x = 0; x < m; x++ {
					d += A[x+i*m] * B[x+j*m]
				}
			case 4:
				for x = 0; x < m; x++ {
					d += A[x+i*m] * B[j+x*k]
				}
			}
			if beta == 0.0 {
				C[i+j*n] = alpha * d
			} else {
				C[i+j*n] = alpha*d + beta*C[i+j*n]
			}
		}
	}
}

func LUDcmp(A []float64, n int, indx []int, d *float64) int {
	var (
		big, s, tmp   float64
		i, imax, j, k int
	)
	vv := Mat(n, 1)

	*d = 1.0
	for i = 0; i < n; i++ {
		big = 0.0
		for j = 0; j < n; j++ {
			if tmp = math.Abs(A[i+j*n]); tmp > big {
				big = tmp
			}
		}
		if big > 0.0 {
			vv[i] = 1.0 / big
		} else {
			return -1
		}
	}
	for j = 0; j < n; j++ {
		for i = 0; i < j; i++ {
			s = A[i+j*n]
			for k = 0; k < i; k++ {
				s -= A[i+k*n] * A[k+j*n]
			}
			A[i+j*n] = s
		}
		big = 0.0
		for i = j; i < n; i++ {
			s = A[i+j*n]
			for k = 0; k < j; k++ {
				s -= A[i+k*n] * A[k+j*n]
			}
			A[i+j*n] = s
			if tmp = vv[i] * math.Abs(s); tmp >= big {
				big = tmp
				imax = i
			}
		}
		if j != imax {
			for k = 0; k < n; k++ {
				A[imax+k*n], A[j+k*n] = A[j+k*n], A[imax+k*n]
			}
			*d = -(*d)
			vv[imax] = vv[j]
		}
		indx[j] = imax
		if A[j+j*n] == 0.0 {
			return -1
		}
		if j != n-1 {
			tmp = 1.0 / A[j+j*n]
			for i = j + 1; i < n; i++ {
				A[i+j*n] *= tmp
			}
		}
	}

	return 0
}

/* LU back-substitution ------------------------------------------------------*/
func LUBksb(A []float64, n int, indx []int, b []float64) {
	var s float64

	ii := -1
	for i := 0; i < n; i++ {
		ip := indx[i]
		s = b[ip]
		b[ip] = b[i]
		if ii >= 0 {
			for j := ii; j < i; j++ {
				s -= A[i+j*n] * b[j]
			}
		} else {
			if s != 0.0 {
				ii = i
			}
		}
		b[i] = s
	}
	for i := n - 1; i >= 0; i-- {
		s = b[i]
		for j := i + 1; j < n; j++ {
			s -= A[i+j*n] * b[j]
		}
		b[i] = s / A[i+i*n]

	}
}

/* inverse of matrix ---------------------------------------------------------*/
func MatInv(A []float64, n int) int {
	var d float64
	var B []float64

	indx := IMat(n, 1)
	B = Mat(n, n)
	MatCpy(B, A, n, n)
	if LUDcmp(B, n, indx, &d) != 0 {
		return -1
	}
	for j := 0; j < n; j++ {
		for i := 0; i < n; i++ {
			A[i+j*n] = 0.0
		}
		A[j+j*n] = 1.0
		LUBksb(B, n, indx, A[j*n:])
	}
	return 0
}

/* Solve linear equation -----------------------------------------------------*/
func Solve(tr string, A, Y []float64, n, m int, X []float64) int {
	B := Mat(n, n)
	var info int
	var tmp string

	MatCpy(B, A, n, n)

	if info = MatInv(B, n); info == 0 {
		if tr[0] == 'N' {
			tmp = "NN"
		} else {
			tmp = "TN"
		}
		MatMul(tmp, n, m, n, 1.0, B, Y, 0.0, X)
	}
	return info
}

/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : double *A        I   transpose of (weighted) design matrix (n x m)
*          double *y        I   (weighted) measurements (m x 1)
*          int    n,m       I   number of parameters and measurements (n<=m)
*          double *x        O   estmated parameters (n x 1)
*          double *Q        O   esimated parameters covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
func LSQ(A, y []float64, n, m int, x, Q []float64) int {
	var info int

	if m < n {
		return -1
	}
	Ay := Mat(n, 1)
	MatMul("NN", n, 1, m, 1.0, A, y, 0.0, Ay) /* Ay=A*y */
	MatMul("NT", n, n, m, 1.0, A, A, 0.0, Q)  /* Q=A*A' */
	if info = MatInv(Q, n); info == 0 {
		MatMul("NN", n, 1, n, 1.0, Q, Ay, 0.0, x) /* x=Q^-1*Ay */
	}

	return info
}

/* kalman filter ---------------------------------------------------------------
* kalman filter state update as follows:
*
*   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
*
* args   : double *x        I   states vector (n x 1)
*          double *P        I   covariance matrix of states (n x n)
*          double *H        I   transpose of design matrix (n x m)
*          double *v        I   innovation (measurement - model) (m x 1)
*          double *R        I   covariance matrix of measurement error (m x m)
*          int    n,m       I   number of states and measurements
*          double *xp       O   states vector after update (n x 1)
*          double *Pp       O   covariance matrix of states after update (n x n)
* return : status (0:ok,<0:error)
* notes  : matirix stored by column-major order (fortran convention)
*          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
*-----------------------------------------------------------------------------*/
func filter_(x, P, H, v, R []float64, n, m int, xp, Pp []float64) int {
	F := Mat(n, m)
	Q := Mat(m, m)
	K := Mat(n, m)
	I := Eye(n)
	var info int

	MatCpy(Q, R, m, m)
	MatCpy(xp, x, n, 1)
	MatMul("NN", n, m, n, 1.0, P, H, 0.0, F) /* Q=H'*P*H+R */
	MatMul("TN", m, m, n, 1.0, H, F, 1.0, Q)
	if info = MatInv(Q, m); info == 0 {
		MatMul("NN", n, m, m, 1.0, F, Q, 0.0, K)  /* K=P*H*Q^-1 */
		MatMul("NN", n, 1, m, 1.0, K, v, 1.0, xp) /* xp=x+K*v */
		MatMul("NT", n, n, m, -1.0, K, H, 1.0, I) /* Pp=(I-K*H')*P */
		MatMul("NN", n, n, n, 1.0, I, P, 0.0, Pp)
	}

	return info
}
func Filter(x, P, H, v, R []float64, n, m int) int {
	var i, j, k, info int

	ix := IMat(n, 1)
	for i, k = 0, 0; i < n; i++ {
		if x[i] != 0.0 && P[i+i*n] > 0.0 {
			ix[k] = i
			k++
		}
	}
	x_ := Mat(k, 1)
	xp_ := Mat(k, 1)
	P_ := Mat(k, k)
	Pp_ := Mat(k, k)
	H_ := Mat(k, m)
	for i = 0; i < k; i++ {
		x_[i] = x[ix[i]]
		for j = 0; j < k; j++ {
			P_[i+j*k] = P[ix[i]+ix[j]*n]
		}
		for j = 0; j < m; j++ {
			H_[i+j*k] = H[ix[i]+j*n]
		}
	}
	info = filter_(x_, P_, H_, v, R, k, m, xp_, Pp_)
	for i = 0; i < k; i++ {
		x[ix[i]] = xp_[i]
		for j = 0; j < k; j++ {
			P[ix[i]+ix[j]*n] = Pp_[i+j*k]
		}
	}
	return info
}

/* Smoother --------------------------------------------------------------------
* combine forward and backward filters by fixed-interval Smoother as follows:
*
*   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
*
* args   : double *xf       I   forward solutions (n x 1)
* args   : double *Qf       I   forward solutions covariance matrix (n x n)
*          double *xb       I   backward solutions (n x 1)
*          double *Qb       I   backward solutions covariance matrix (n x n)
*          int    n         I   number of solutions
*          double *xs       O   smoothed solutions (n x 1)
*          double *Qs       O   smoothed solutions covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : see reference [4] 5.2
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
func Smoother(xf, Qf, xb, Qb []float64, n int, xs, Qs []float64) int {
	invQf := Mat(n, n)
	invQb := Mat(n, n)
	xx := Mat(n, 1)
	var i, info int = 0, -1

	MatCpy(invQf, Qf, n, n)
	MatCpy(invQb, Qb, n, n)
	if MatInv(invQf, n) == 0 && MatInv(invQb, n) == 0 {
		for i = 0; i < n*n; i++ {
			Qs[i] = invQf[i] + invQb[i]
		}
		if info = MatInv(Qs, n); info == 0 {
			MatMul("NN", n, 1, n, 1.0, invQf, xf, 0.0, xx)
			MatMul("NN", n, 1, n, 1.0, invQb, xb, 1.0, xx)
			MatMul("NN", n, 1, n, 1.0, Qs, xx, 0.0, xs)
		}
	}

	return info
}

/* print matrix ----------------------------------------------------------------
* print matrix to stdout
* args   : double *A        I   matrix A (n x m)
*          int    n,m       I   number of rows and columns of A
*          int    p,q       I   total columns, columns under decimal point
*         (FILE  *fp        I   output file pointer)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
func matfprint(A []float64, n, m, p, q int, fp *os.File) {

	for i := 0; i < n; i++ {
		for j := 0; j < m; j++ {
			fmt.Fprintf(fp, " %*.*f", p, q, A[i+j*n])
		}
		fp.WriteString("\n")
	}
}
func MatPrint(A []float64, n, m, p, q int) {
	matfprint(A, n, m, p, q, os.Stdout)
}

/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
func Str2Num(s string, i, n int) float64 {
	var value float64
	var err error

	if i < 0 || len(s) < i {
		return 0.0
	}
	if i+n > len(s) {
		s = s[i:]
	} else {
		s = s[i : i+n]
	}
	nr := strings.NewReplacer("d", "E", "D", "E")
	str := nr.Replace(s)
	str = strings.TrimSpace(str)
	value, err = strconv.ParseFloat(str, 64)
	if err != nil {
		return 0.0
	}
	return value
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
func Epoch2Time(ep []float64) Gtime {
	var (
		doy []int = []int{1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}

		ret            Gtime = Gtime{0, 0}
		days, sec      int
		year, mon, day = int(ep[0]), int(ep[1]), int(ep[2])
	)

	if year < 1970 || 2099 < year || mon < 1 || 12 < mon {
		return ret
	}

	/* leap year if year%4==0 in 1901-2099 */
	if year%4 == 0 && mon >= 3 {
		days = (year-1970)*365 + (year-1969)/4 + doy[mon-1] + day - 2 + 1

	} else {
		days = (year-1970)*365 + (year-1969)/4 + doy[mon-1] + day - 2
	}
	sec = int(math.Floor(ep[5]))
	ret.Time = uint64(days*86400 + int(ep[3])*3600 + int(ep[4])*60 + sec)
	ret.Sec = ep[5] - float64(sec)
	return ret
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
func Time2Epoch(t Gtime, ep []float64) {
	var mday []int = []int{ /* # of days in a month */
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
		31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
	var days, sec, mon, day int

	/* leap year if year%4==0 in 1901-2099 */
	days = int(t.Time / 86400)
	sec = int(t.Time - uint64(days*86400))
	mon = 0
	for day = days % 1461; mon < 48; mon++ {
		if day >= mday[mon] {
			day -= mday[mon]
		} else {
			break
		}
	}
	ep[0] = float64(1970 + days/1461*4 + mon/12)
	ep[1] = float64(mon%12 + 1)
	ep[2] = float64(day + 1)
	ep[3] = float64(sec / 3600)
	ep[4] = float64(sec % 3600 / 60)
	ep[5] = float64(sec%60) + t.Sec
}

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
func GpsT2Time(week int, sec float64) Gtime {
	t := Epoch2Time(gpst0[:])

	if sec < (-1e9) || 1e9 < sec {
		sec = 0.0
	}
	t.Time += uint64(86400*7*week) + uint64(sec)
	t.Sec = sec - float64(int(sec))
	return t
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
func Time2GpsT(t Gtime, week *int) float64 {
	var (
		t0  = Epoch2Time(gpst0[:])
		sec = t.Time - t0.Time
		w   = (int)(sec / (86400 * 7))
	)

	if week != nil {
		*week = w
	}
	return float64(sec) - float64(w*86400*7) + t.Sec
}

/* galileo system time to time -------------------------------------------------
* convert week and tow in galileo system time (gst) to gtime_t struct
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
func GsT2Time(week int, sec float64) Gtime {
	var t = Epoch2Time(gst0[:])

	if sec < (-1e9) || 1e9 < sec {
		sec = 0.0
	}
	t.Time += uint64(86400*7*week) + uint64(sec)
	t.Sec = sec - float64(int(sec))
	return t
}

/* time to galileo system time -------------------------------------------------
* convert gtime_t struct to week and tow in galileo system time (gst)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gst (NULL: no output)
* return : time of week in gst (s)
*-----------------------------------------------------------------------------*/
func Time2GsT(t Gtime, week *int) float64 {
	var (
		t0  = Epoch2Time(gst0[:])
		sec = t.Time - t0.Time
		w   = (int)(sec / (86400 * 7))
	)

	if week != nil {
		*week = w
	}
	return float64(sec) - float64(w*86400*7) + t.Sec
}

/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
func BDT2Time(week int, sec float64) Gtime {
	var t = Epoch2Time(bdt0[:])

	if sec < (-1e9) || 1e9 < sec {
		sec = 0.0
	}
	t.Time += uint64(86400*7*week) + uint64(sec)
	t.Sec = sec - float64(int(sec))
	return t
}

/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
func Time2BDT(t Gtime, week *int) float64 {
	var (
		t0  = Epoch2Time(bdt0[:])
		sec = t.Time - t0.Time
		w   = (int)(sec / (86400 * 7))
	)

	if week != nil {
		*week = w
	}
	return float64(sec) - float64(w*86400*7) + t.Sec
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
func TimeAdd(t Gtime, sec float64) Gtime {
	t.Sec += sec
	var tt = math.Floor(t.Sec)
	t.Time += uint64(tt)
	t.Sec -= tt
	return t
}

/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
func TimeDiff(t1 Gtime, t2 Gtime) float64 {
	return float64(t1.Time) - float64(t2.Time) + t1.Sec - t2.Sec
}

/* get current time in utc -----------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-----------------------------------------------------------------------------*/
var timeoffset float64 = 0.0 /* time offset (s) */
var timeLock sync.Mutex

func TimeGet() Gtime {
	timeLock.Lock()
	defer timeLock.Unlock()

	var ts = time.Now()
	var ep []float64 = []float64{float64(ts.Year()),
		float64(ts.Month()),
		float64(ts.Day()),
		float64(ts.Hour()),
		float64(ts.Minute()),
		float64(ts.Second()) + float64(ts.Nanosecond())*1e-6}

	var t = Epoch2Time(ep)

	return TimeAdd(t, timeoffset)
}

/* set current time in utc -----------------------------------------------------
* set current time in utc
* args   : gtime_t          I   current time in utc
* return : none
* notes  : just set time offset between cpu time and current time
*          the time offset is reflected to only timeget()
*          not reentrant
*-----------------------------------------------------------------------------*/
func TimeSet(t Gtime) {
	timeoffset += TimeDiff(t, TimeGet())
}

/* reset current time ----------------------------------------------------------
* reset current time
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
func TimeReset() {
	timeoffset = 0.0
}

var leaps = [MAXLEAPS + 1][7]float64{ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017, 1, 1, 0, 0, 0, -18},
	{2015, 7, 1, 0, 0, 0, -17},
	{2012, 7, 1, 0, 0, 0, -16},
	{2009, 1, 1, 0, 0, 0, -15},
	{2006, 1, 1, 0, 0, 0, -14},
	{1999, 1, 1, 0, 0, 0, -13},
	{1997, 7, 1, 0, 0, 0, -12},
	{1996, 1, 1, 0, 0, 0, -11},
	{1994, 7, 1, 0, 0, 0, -10},
	{1993, 7, 1, 0, 0, 0, -9},
	{1992, 7, 1, 0, 0, 0, -8},
	{1991, 1, 1, 0, 0, 0, -7},
	{1990, 1, 1, 0, 0, 0, -6},
	{1988, 1, 1, 0, 0, 0, -5},
	{1985, 7, 1, 0, 0, 0, -4},
	{1983, 7, 1, 0, 0, 0, -3},
	{1982, 7, 1, 0, 0, 0, -2},
	{1981, 7, 1, 0, 0, 0, -1},
}

/* read leap seconds table by text -------------------------------------------*/
func ReadLeapsText(fp *os.File) int {
	var (
		ep   [6]int
		ls   int
		i, n = 0, 0
	)
	_, _ = fp.Seek(0, io.SeekStart)
	reader := bufio.NewReader(fp)

	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		l := strings.Split(line, "#")
		fmt.Sscanf(l[0], "%d %d %d %d %d %d %d", &ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5], &ls)
		if err != nil {
			if err == io.EOF {
				break
			}
		}
		for i = 0; i < 6; i++ {
			leaps[n][i] = float64(ep[i])
		}
		leaps[n][6] = float64(ls)
		n++
	}

	return n
}

/* read leap seconds table by usno -------------------------------------------*/
func ReadLeapsUsno(fp *os.File) int {
	var (
		months = []string{
			"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"}
		jd, tai_utc      float64
		month            [32]byte
		ls               [MAXLEAPS][7]byte
		i, j, y, m, d, n int = 0, 0, 0, 0, 0, 0
	)

	_, _ = fp.Seek(0, io.SeekStart)
	reader := bufio.NewReader(fp)

	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			if err == io.EOF {
				break
			}
		}

		n, _ = fmt.Sscanf(line, "%d %s %d =JD %f TAI-UTC= %f", &y, month, &d, &jd, &tai_utc)
		if n < 5 {
			continue
		}
		if y < 1980 {
			continue
		}
		for m = 1; m <= 12; m++ {

			if strings.Compare(months[m-1], string(month[:])) != 0 {
				break
			}
		}
		if m >= 13 {
			continue
		}

		ls[n][0] = byte(y)
		ls[n][1] = byte(m)
		ls[n][2] = byte(d)
		ls[n][6] = byte(19.0 - tai_utc)
		n++
	}
	for i = 0; i < n; i++ {
		for j = 0; j < 7; j++ {
			leaps[i][j] = float64(ls[n-i-1][j])
		}
	}
	return n
}

/* read leap seconds table -----------------------------------------------------
* read leap seconds table
* args   : char    *file    I   leap seconds table file
* return : status (1:ok,0:error)
* notes  : The leap second table should be as follows or leapsec.dat provided
*          by USNO.
*          (1) The records in the table file cosist of the following fields:
*              year month day hour min sec UTC-GPST(s)
*          (2) The date and time indicate the start UTC time for the UTC-GPST
*          (3) The date and time should be descending order.
*-----------------------------------------------------------------------------*/
func Read_Leaps(file string) int {
	var fp *os.File

	fp, err := os.OpenFile(file, os.O_RDONLY, 0666)
	if err != nil {
		return 0
	}
	defer fp.Close()
	/* read leap seconds table by text or usno */
	n1 := ReadLeapsText(fp)
	n2 := ReadLeapsUsno(fp)
	if n1 == 0 && n2 == 0 {
		return 0
	}
	for i := 0; i < 7; i++ {
		leaps[n2][i] = 0.0
	}
	return 1
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
func GpsT2Utc(t Gtime) Gtime {
	var tu Gtime

	for i := 0; leaps[i][0] > 0; i++ {
		tu = TimeAdd(t, leaps[i][6])

		if TimeDiff(tu, Epoch2Time(leaps[i][:])) >= 0.0 {
			return tu
		}
	}
	return t
}

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
func Utc2GpsT(t Gtime) Gtime {

	for i := 0; leaps[i][0] > 0; i++ {
		if TimeDiff(t, Epoch2Time(leaps[i][:])) >= 0.0 {
			return TimeAdd(t, -leaps[i][6])
		}
	}
	return t
}

/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
func GpsT2BDT(t Gtime) Gtime {
	return TimeAdd(t, -14.0)
}

/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
func BDT2GpsT(t Gtime) Gtime {
	return TimeAdd(t, 14.0)
}

/* time to day and sec -------------------------------------------------------*/
func Time2Sec(time Gtime, day *Gtime) float64 {
	var ep [6]float64
	Time2Epoch(time, ep[:])
	sec := ep[3]*3600.0 + ep[4]*60.0 + ep[5]
	ep[3] = 0.0
	ep[4] = 0.0
	ep[5] = 0.0
	*day = Epoch2Time(ep[:])
	return sec
}

/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
func Utc2GmsT(t Gtime, ut1_utc float64) float64 {
	var (
		ep2000    []float64 = []float64{2000.0, 1.0, 1.0, 12.0, 0.0, 0.0}
		tut, tut0 Gtime
	)

	tut = TimeAdd(t, ut1_utc)
	ut := Time2Sec(tut, &tut0)
	t1 := TimeDiff(tut0, Epoch2Time(ep2000)) / 86400.0 / 36525.0
	t2 := t1 * t1
	t3 := t2 * t1
	gmst0 := 24110.54841 + 8640184.812866*t1 + 0.093104*t2 - 6.2e-6*t3
	gmst := gmst0 + 1.002737909350795*ut

	return math.Mod(gmst, 86400.0) * PI / 43200.0 /* 0 <= gmst <= 2*PI */
}

/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
func Time2Str(t Gtime, s *string, n int) {
	var ep [6]float64 = [6]float64{0, 0, 0, 0, 0, 0}

	if n < 0 {
		n = 0
	} else if n > 12 {
		n = 12
	}
	if 1.0-t.Sec < 0.5/math.Pow(10.0, float64(n)) {
		t.Time++
		t.Sec = 0.0
	}
	Time2Epoch(t, ep[:])
	var n1, n2 int
	if n <= 0 {
		n1 = 2
		n2 = 0
	} else {
		n1 = n + 3
		n2 = n
	}

	*s = fmt.Sprintf("%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2],
		ep[3], ep[4], n1, n2, ep[5])
}

/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
func TimeStr(t Gtime, n int) string {
	var buff string
	Time2Str(t, &buff, n)
	return buff
}

/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
func Time2DayOfYeay(t Gtime) float64 {
	var ep [6]float64

	Time2Epoch(t, ep[:])
	ep[1] = 1.0
	ep[2] = 1.0
	ep[3] = 0.0
	ep[4] = 0.0
	ep[5] = 0.0
	return TimeDiff(t, Epoch2Time(ep[:]))/86400.0 + 1.0
}

/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number (0-1023)
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
func AdjGpsWeek(week int) int {
	var w int
	Time2GpsT(Utc2GpsT(TimeGet()), &w)
	if w < 1560 {
		w = 1560 /* use 2009/12/1 if time is earlier than 2009/12/1 */
	}
	return week + (w-week+1)/1024*1024
}

/* get tick time ---------------------------------------------------------------
* get current tick in ms
* args   : none
* return : current tick in ms
*-----------------------------------------------------------------------------*/
func TickGet() int64 {
	timeObj := time.Now()
	return timeObj.UnixMilli()
}

/* sleep ms --------------------------------------------------------------------
* sleep ms
* args   : int   ms         I   miliseconds to sleep (<0:no sleep)
* return : none
*-----------------------------------------------------------------------------*/
func Sleepms(ms int) {
	time.Sleep(time.Duration(ms) * time.Millisecond)
}

/* convert degree to deg-min-sec -----------------------------------------------
* convert degree to degree-minute-second
* args   : double deg       I   degree
*          double *dms      O   degree-minute-second {deg,min,sec}
*          int    ndec      I   number of decimals of second
* return : none
*-----------------------------------------------------------------------------*/
func Deg2Dms(deg float64, dms []float64, ndec int) {
	var sign float64
	if deg < 0.0 {
		sign = -1.0
	} else {
		sign = 1.0
	}

	a := math.Abs(deg)
	unit := math.Pow(0.1, float64(ndec))

	dms[0] = math.Floor(a)
	a = (a - dms[0]) * 60.0

	dms[1] = math.Floor(a)
	a = (a - dms[1]) * 60.0

	dms[2] = math.Floor(a/unit+0.5) * unit
	if dms[2] >= 60.0 {
		dms[2] = 0.0
		dms[1] += 1.0

		if dms[1] >= 60.0 {
			dms[1] = 0.0
			dms[0] += 1.0
		}
	}
	dms[0] *= sign
}

/* convert deg-min-sec to degree -----------------------------------------------
* convert degree-minute-second to degree
* args   : double *dms      I   degree-minute-second {deg,min,sec}
* return : degree
*-----------------------------------------------------------------------------*/
func Dms2Deg(dms []float64) float64 {
	var sign float64
	if dms[0] < 0 {
		sign = -1
	} else {
		sign = 1.0
	}

	return sign * (math.Abs(dms[0]) + dms[1]/60.0 + dms[2]/3600.0)
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
func Ecef2Pos(r, pos []float64) {
	e2 := FE_WGS84 * (2.0 - FE_WGS84)
	r2 := Dot(r, r, 2)
	var z, zk, sinp float64
	v := RE_WGS84
	zk = 0
	for z = r[2]; math.Abs(z-zk) >= 1e-4; {
		zk = z
		sinp = z / math.Sqrt(r2+z*z)
		v = RE_WGS84 / math.Sqrt(1.0-e2*sinp*sinp)
		z = r[2] + v*e2*sinp
	}
	if r2 > 1e-12 {
		pos[0] = math.Atan(z / math.Sqrt(r2))
	} else if r[2] > 0.0 {
		pos[0] = PI / 2.0
	} else {
		pos[0] = -PI / 2.0
	}
	if r2 > 1e-12 {
		pos[1] = math.Atan2(r[1], r[0])
	} else {
		pos[1] = 0.0
	}
	pos[2] = math.Sqrt(r2+z*z) - v
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
func Pos2Ecef(pos, r []float64) {
	sinp := math.Sin(pos[0])
	cosp := math.Cos(pos[0])
	sinl := math.Sin(pos[1])
	cosl := math.Cos(pos[1])
	e2 := FE_WGS84 * (2.0 - FE_WGS84)
	v := RE_WGS84 / math.Sqrt(1.0-e2*sinp*sinp)

	r[0] = (v + pos[2]) * cosp * cosl
	r[1] = (v + pos[2]) * cosp * sinl
	r[2] = (v*(1.0-e2) + pos[2]) * sinp
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
func XYZ2Enu(pos, E []float64) {
	sinp := math.Sin(pos[0])
	cosp := math.Cos(pos[0])
	sinl := math.Sin(pos[1])
	cosl := math.Cos(pos[1])

	E[0] = -sinl
	E[3] = cosl
	E[6] = 0.0
	E[1] = -sinp * cosl
	E[4] = -sinp * sinl
	E[7] = cosp
	E[2] = cosp * cosl
	E[5] = cosp * sinl
	E[8] = sinp
}

/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
func Ecef2Enu(pos, r, e []float64) {
	var E [9]float64

	XYZ2Enu(pos, E[:])
	MatMul("NN", 3, 1, 3, 1.0, E[:], r, 0.0, e)
}

/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
func Enu2Ecef(pos, e, r []float64) {
	var E [9]float64

	XYZ2Enu(pos, E[:])
	MatMul("TN", 3, 1, 3, 1.0, E[:], e, 0.0, r)
}

/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
func Cov2Enu(pos, P, Q []float64) {
	var E, EP [9]float64

	XYZ2Enu(pos, E[:])
	MatMul("NN", 3, 3, 3, 1.0, E[:], P, 0.0, EP[:])
	MatMul("NT", 3, 3, 3, 1.0, EP[:], E[:], 0.0, Q)
}

/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
func Cov2Ecef(pos, Q, P []float64) {
	var E, EQ [9]float64

	XYZ2Enu(pos, E[:])
	MatMul("TN", 3, 3, 3, 1.0, E[:], Q, 0.0, EQ[:])
	MatMul("NN", 3, 3, 3, 1.0, EQ[:], E[:], 0.0, P)
}

/* coordinate rotation matrix ------------------------------------------------*/
// #define Rx(t,X) do { \
//     (X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
//     (X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
// } while (0)
func Rx(t float64, X []float64) {
	X[0] = 1.0
	X[1], X[2], X[3], X[6] = 0.0, 0.0, 0.0, 0.0
	tmp := math.Cos(t)
	X[4], X[8] = tmp, tmp
	X[7] = math.Sin(t)
	X[5] = -X[7]
}

//	#define Ry(t,X) do { \
//	    (X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
//	    (X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
//	} while (0)
func Ry(t float64, X []float64) {
	X[4] = 1.0
	X[1], X[3], X[5], X[7] = 0.0, 0.0, 0.0, 0.0
	tmp := math.Cos(t)
	X[0], X[8] = tmp, tmp
	X[2] = math.Sin(t)
	X[6] = -X[2]
}

//	#define Rz(t,X) do { \
//	    (X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
//	    (X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
//	} while (0)
func Rz(t float64, X []float64) {
	X[8] = 1.0
	X[2], X[5], X[6], X[7] = 0.0, 0.0, 0.0, 0.0
	tmp := math.Cos(t)
	X[0], X[4] = tmp, tmp
	X[3] = math.Sin(t)
	X[1] = -X[3]
}

/* astronomical arguments: f={l,l',F,D,OMG} (rad) ----------------------------*/
func ast_args(t float64, f []float64) {
	var (
		fc [][5]float64 = [][5]float64{ /* coefficients for iau 1980 nutation */
			{134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470},
			{357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149},
			{93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417},
			{297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169},
			{125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939},
		}
		tt   [4]float64
		i, j int = 1, 0
	)

	for tt[0] = t; i < 4; i++ {
		tt[i] = tt[i-1] * t
	}
	for i = 0; i < 5; i++ {
		f[i] = fc[i][0] * 3600.0
		for j = 0; j < 4; j++ {
			f[i] += fc[i][j+1] * tt[j]
		}
		f[i] = math.Mod(f[i]*AS2R, 2.0*PI)
	}
}

/* iau 1980 nutation ---------------------------------------------------------*/
func nut_iau1980(t float64, f []float64, dpsi, deps *float64) {
	var nut [106][10]float64 = [106][10]float64{
		{0, 0, 0, 0, 1, -6798.4, -171996, -174.2, 92025, 8.9},
		{0, 0, 2, -2, 2, 182.6, -13187, -1.6, 5736, -3.1},
		{0, 0, 2, 0, 2, 13.7, -2274, -0.2, 977, -0.5},
		{0, 0, 0, 0, 2, -3399.2, 2062, 0.2, -895, 0.5},
		{0, -1, 0, 0, 0, -365.3, -1426, 3.4, 54, -0.1},
		{1, 0, 0, 0, 0, 27.6, 712, 0.1, -7, 0.0},
		{0, 1, 2, -2, 2, 121.7, -517, 1.2, 224, -0.6},
		{0, 0, 2, 0, 1, 13.6, -386, -0.4, 200, 0.0},
		{1, 0, 2, 0, 2, 9.1, -301, 0.0, 129, -0.1},
		{0, -1, 2, -2, 2, 365.2, 217, -0.5, -95, 0.3},
		{-1, 0, 0, 2, 0, 31.8, 158, 0.0, -1, 0.0},
		{0, 0, 2, -2, 1, 177.8, 129, 0.1, -70, 0.0},
		{-1, 0, 2, 0, 2, 27.1, 123, 0.0, -53, 0.0},
		{1, 0, 0, 0, 1, 27.7, 63, 0.1, -33, 0.0},
		{0, 0, 0, 2, 0, 14.8, 63, 0.0, -2, 0.0},
		{-1, 0, 2, 2, 2, 9.6, -59, 0.0, 26, 0.0},
		{-1, 0, 0, 0, 1, -27.4, -58, -0.1, 32, 0.0},
		{1, 0, 2, 0, 1, 9.1, -51, 0.0, 27, 0.0},
		{-2, 0, 0, 2, 0, -205.9, -48, 0.0, 1, 0.0},
		{-2, 0, 2, 0, 1, 1305.5, 46, 0.0, -24, 0.0},
		{0, 0, 2, 2, 2, 7.1, -38, 0.0, 16, 0.0},
		{2, 0, 2, 0, 2, 6.9, -31, 0.0, 13, 0.0},
		{2, 0, 0, 0, 0, 13.8, 29, 0.0, -1, 0.0},
		{1, 0, 2, -2, 2, 23.9, 29, 0.0, -12, 0.0},
		{0, 0, 2, 0, 0, 13.6, 26, 0.0, -1, 0.0},
		{0, 0, 2, -2, 0, 173.3, -22, 0.0, 0, 0.0},
		{-1, 0, 2, 0, 1, 27.0, 21, 0.0, -10, 0.0},
		{0, 2, 0, 0, 0, 182.6, 17, -0.1, 0, 0.0},
		{0, 2, 2, -2, 2, 91.3, -16, 0.1, 7, 0.0},
		{-1, 0, 0, 2, 1, 32.0, 16, 0.0, -8, 0.0},
		{0, 1, 0, 0, 1, 386.0, -15, 0.0, 9, 0.0},
		{1, 0, 0, -2, 1, -31.7, -13, 0.0, 7, 0.0},
		{0, -1, 0, 0, 1, -346.6, -12, 0.0, 6, 0.0},
		{2, 0, -2, 0, 0, -1095.2, 11, 0.0, 0, 0.0},
		{-1, 0, 2, 2, 1, 9.5, -10, 0.0, 5, 0.0},
		{1, 0, 2, 2, 2, 5.6, -8, 0.0, 3, 0.0},
		{0, -1, 2, 0, 2, 14.2, -7, 0.0, 3, 0.0},
		{0, 0, 2, 2, 1, 7.1, -7, 0.0, 3, 0.0},
		{1, 1, 0, -2, 0, -34.8, -7, 0.0, 0, 0.0},
		{0, 1, 2, 0, 2, 13.2, 7, 0.0, -3, 0.0},
		{-2, 0, 0, 2, 1, -199.8, -6, 0.0, 3, 0.0},
		{0, 0, 0, 2, 1, 14.8, -6, 0.0, 3, 0.0},
		{2, 0, 2, -2, 2, 12.8, 6, 0.0, -3, 0.0},
		{1, 0, 0, 2, 0, 9.6, 6, 0.0, 0, 0.0},
		{1, 0, 2, -2, 1, 23.9, 6, 0.0, -3, 0.0},
		{0, 0, 0, -2, 1, -14.7, -5, 0.0, 3, 0.0},
		{0, -1, 2, -2, 1, 346.6, -5, 0.0, 3, 0.0},
		{2, 0, 2, 0, 1, 6.9, -5, 0.0, 3, 0.0},
		{1, -1, 0, 0, 0, 29.8, 5, 0.0, 0, 0.0},
		{1, 0, 0, -1, 0, 411.8, -4, 0.0, 0, 0.0},
		{0, 0, 0, 1, 0, 29.5, -4, 0.0, 0, 0.0},
		{0, 1, 0, -2, 0, -15.4, -4, 0.0, 0, 0.0},
		{1, 0, -2, 0, 0, -26.9, 4, 0.0, 0, 0.0},
		{2, 0, 0, -2, 1, 212.3, 4, 0.0, -2, 0.0},
		{0, 1, 2, -2, 1, 119.6, 4, 0.0, -2, 0.0},
		{1, 1, 0, 0, 0, 25.6, -3, 0.0, 0, 0.0},
		{1, -1, 0, -1, 0, -3232.9, -3, 0.0, 0, 0.0},
		{-1, -1, 2, 2, 2, 9.8, -3, 0.0, 1, 0.0},
		{0, -1, 2, 2, 2, 7.2, -3, 0.0, 1, 0.0},
		{1, -1, 2, 0, 2, 9.4, -3, 0.0, 1, 0.0},
		{3, 0, 2, 0, 2, 5.5, -3, 0.0, 1, 0.0},
		{-2, 0, 2, 0, 2, 1615.7, -3, 0.0, 1, 0.0},
		{1, 0, 2, 0, 0, 9.1, 3, 0.0, 0, 0.0},
		{-1, 0, 2, 4, 2, 5.8, -2, 0.0, 1, 0.0},
		{1, 0, 0, 0, 2, 27.8, -2, 0.0, 1, 0.0},
		{-1, 0, 2, -2, 1, -32.6, -2, 0.0, 1, 0.0},
		{0, -2, 2, -2, 1, 6786.3, -2, 0.0, 1, 0.0},
		{-2, 0, 0, 0, 1, -13.7, -2, 0.0, 1, 0.0},
		{2, 0, 0, 0, 1, 13.8, 2, 0.0, -1, 0.0},
		{3, 0, 0, 0, 0, 9.2, 2, 0.0, 0, 0.0},
		{1, 1, 2, 0, 2, 8.9, 2, 0.0, -1, 0.0},
		{0, 0, 2, 1, 2, 9.3, 2, 0.0, -1, 0.0},
		{1, 0, 0, 2, 1, 9.6, -1, 0.0, 0, 0.0},
		{1, 0, 2, 2, 1, 5.6, -1, 0.0, 1, 0.0},
		{1, 1, 0, -2, 1, -34.7, -1, 0.0, 0, 0.0},
		{0, 1, 0, 2, 0, 14.2, -1, 0.0, 0, 0.0},
		{0, 1, 2, -2, 0, 117.5, -1, 0.0, 0, 0.0},
		{0, 1, -2, 2, 0, -329.8, -1, 0.0, 0, 0.0},
		{1, 0, -2, 2, 0, 23.8, -1, 0.0, 0, 0.0},
		{1, 0, -2, -2, 0, -9.5, -1, 0.0, 0, 0.0},
		{1, 0, 2, -2, 0, 32.8, -1, 0.0, 0, 0.0},
		{1, 0, 0, -4, 0, -10.1, -1, 0.0, 0, 0.0},
		{2, 0, 0, -4, 0, -15.9, -1, 0.0, 0, 0.0},
		{0, 0, 2, 4, 2, 4.8, -1, 0.0, 0, 0.0},
		{0, 0, 2, -1, 2, 25.4, -1, 0.0, 0, 0.0},
		{-2, 0, 2, 4, 2, 7.3, -1, 0.0, 1, 0.0},
		{2, 0, 2, 2, 2, 4.7, -1, 0.0, 0, 0.0},
		{0, -1, 2, 0, 1, 14.2, -1, 0.0, 0, 0.0},
		{0, 0, -2, 0, 1, -13.6, -1, 0.0, 0, 0.0},
		{0, 0, 4, -2, 2, 12.7, 1, 0.0, 0, 0.0},
		{0, 1, 0, 0, 2, 409.2, 1, 0.0, 0, 0.0},
		{1, 1, 2, -2, 2, 22.5, 1, 0.0, -1, 0.0},
		{3, 0, 2, -2, 2, 8.7, 1, 0.0, 0, 0.0},
		{-2, 0, 2, 2, 2, 14.6, 1, 0.0, -1, 0.0},
		{-1, 0, 0, 0, 2, -27.3, 1, 0.0, -1, 0.0},
		{0, 0, -2, 2, 1, -169.0, 1, 0.0, 0, 0.0},
		{0, 1, 2, 0, 1, 13.1, 1, 0.0, 0, 0.0},
		{-1, 0, 4, 0, 2, 9.1, 1, 0.0, 0, 0.0},
		{2, 1, 0, -2, 0, 131.7, 1, 0.0, 0, 0.0},
		{2, 0, 0, 2, 0, 7.1, 1, 0.0, 0, 0.0},
		{2, 0, 2, -2, 1, 12.8, 1, 0.0, -1, 0.0},
		{2, 0, -2, 0, 1, -943.2, 1, 0.0, 0, 0.0},
		{1, -1, 0, -2, 0, -29.3, 1, 0.0, 0, 0.0},
		{-1, 0, 0, 1, 1, -388.3, 1, 0.0, 0, 0.0},
		{-1, -1, 0, 2, 1, 35.0, 1, 0.0, 0, 0.0},
		{0, 1, 0, 1, 0, 27.3, 1, 0.0, 0, 0.0},
	}
	var ang float64

	*dpsi, *deps = 0.0, 0.0

	for i := 0; i < 106; i++ {
		ang = 0.0
		for j := 0; j < 5; j++ {
			ang += nut[i][j] * f[j]
		}
		*dpsi += (nut[i][6] + nut[i][7]*t) * math.Sin(ang)
		*deps += (nut[i][8] + nut[i][9]*t) * math.Cos(ang)
	}
	*dpsi *= 1e-4 * AS2R /* 0.1 mas . rad */
	*deps *= 1e-4 * AS2R
}

/* eci to ecef transformation matrix -------------------------------------------
* compute eci to ecef transformation matrix
* args   : gtime_t tutc     I   time in utc
*          double *erpv     I   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *U        O   eci to ecef transformation matrix (3 x 3)
*          double *gmst     IO  greenwich mean sidereal time (rad)
*                               (NULL: no output)
* return : none
* note   : see ref [3] chap 5
*          not thread-safe
*-----------------------------------------------------------------------------*/
func Eci2Ecef(tutc Gtime, erpv, U []float64, gmst *float64) {
	var (
		ep2000                                      []float64 = []float64{2000, 1, 1, 12, 0, 0}
		tutc_, tgps                                 Gtime
		U_                                          [9]float64
		gmst_                                       float64
		eps, ze, th, z, t, t2, t3, dpsi, deps, gast float64
		f                                           [5]float64
		R1, R2, R3, R, W, N, P, NP                  [9]float64
		i                                           int
	)
	if math.Abs(TimeDiff(tutc, tutc_)) < 0.01 { /* read cache */
		for i = 0; i < 9; i++ {
			U[i] = U_[i]
		}
		if gmst != nil {
			*gmst = gmst_
		}
		return
	}
	tutc_ = tutc

	/* terrestrial time */
	tgps = Utc2GpsT(tutc_)
	t = (TimeDiff(tgps, Epoch2Time(ep2000)) + 19.0 + 32.184) / 86400.0 / 36525.0
	t2 = t * t
	t3 = t2 * t

	/* astronomical arguments */
	ast_args(t, f[:])

	/* iau 1976 precession */
	ze = (2306.2181*t + 0.30188*t2 + 0.017998*t3) * AS2R
	th = (2004.3109*t - 0.42665*t2 - 0.041833*t3) * AS2R
	z = (2306.2181*t + 1.09468*t2 + 0.018203*t3) * AS2R
	eps = (84381.448 - 46.8150*t - 0.00059*t2 + 0.001813*t3) * AS2R
	Rz(-z, R1[:])
	Ry(th, R2[:])
	Rz(-ze, R3[:])
	MatMul("NN", 3, 3, 3, 1.0, R1[:], R2[:], 0.0, R[:])
	MatMul("NN", 3, 3, 3, 1.0, R[:], R3[:], 0.0, P[:]) /* P=Rz(-z)*Ry(th)*Rz(-ze) */

	/* iau 1980 nutation */
	nut_iau1980(t, f[:], &dpsi, &deps)
	Rx(-eps-deps, R1[:])
	Rz(-dpsi, R2[:])
	Rx(eps, R3[:])
	MatMul("NN", 3, 3, 3, 1.0, R1[:], R2[:], 0.0, R[:])
	MatMul("NN", 3, 3, 3, 1.0, R[:], R3[:], 0.0, N[:]) /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

	/* greenwich aparent sidereal time (rad) */
	gmst_ = Utc2GmsT(tutc_, erpv[2])
	gast = gmst_ + dpsi*math.Cos(eps)
	gast += (0.00264*math.Sin(f[4]) + 0.000063*math.Sin(2.0*f[4])) * AS2R

	/* eci to ecef transformation matrix */
	Ry(-erpv[0], R1[:])
	Rx(-erpv[1], R2[:])
	Rz(gast, R3[:])
	MatMul("NN", 3, 3, 3, 1.0, R1[:], R2[:], 0.0, W[:])
	MatMul("NN", 3, 3, 3, 1.0, W[:], R3[:], 0.0, R[:]) /* W=Ry(-xp)*Rx(-yp) */
	MatMul("NN", 3, 3, 3, 1.0, N[:], P[:], 0.0, NP[:])
	MatMul("NN", 3, 3, 3, 1.0, R[:], NP[:], 0.0, U_[:]) /* U=W*Rz(gast)*N*P */

	for i = 0; i < 9; i++ {
		U[i] = U_[i]
	}
	if gmst != nil {
		*gmst = gmst_
	}

	Trace(5, "gmst=%.12f gast=%.12f\n", gmst_, gast)
	Trace(5, "P=\n")
	tracemat(5, P[:], 3, 3, 15, 12)
	Trace(5, "N=\n")
	tracemat(5, N[:], 3, 3, 15, 12)
	Trace(5, "W=\n")
	tracemat(5, W[:], 3, 3, 15, 12)
	Trace(5, "U=\n")
	tracemat(5, U[:], 3, 3, 15, 12)
}

/* decode antenna parameter field --------------------------------------------*/
func DecodeF(p string, n int, v []float64) int {
	var i int
	for i = 0; i < n; i++ {
		v[i] = 0.0
	}
	ps := strings.Fields(p)
	for i = 0; i < len(ps) && i < n; i++ {
		v[i], _ = strconv.ParseFloat(ps[i], 64)
		v[i] *= 1e-3
	}
	return i
}

/* add antenna parameter -----------------------------------------------------*/
func (pcvs *Pcvs) AddPcv(pcv *Pcv) {
	pcvs.Pcv = append(pcvs.Pcv, *pcv)
}

/* read ngs antenna parameter file -------------------------------------------*/
func ReadNgsPcv(file string, pcvs *Pcvs) int {
	var (
		fp   *os.File
		pcv0 Pcv
		pcv  Pcv
		neu  [3]float64
		n    int = 0
	)

	if fp, _ := os.Open(file); fp == nil {
		Trace(2, "ngs pcv file open error: %s\n", file)
		return 0
	}
	defer fp.Close()

	reader := bufio.NewReader(fp)
	for {
		buff, err := reader.ReadBytes('\n')
		if err != nil {
			break
		}

		if len(buff) >= 62 && buff[61] == '|' {
			continue
		}

		if buff[0] != ' ' {
			n = 0 /* start line */
		}
		n++
		switch n {
		case 1:
			pcv = pcv0
			pcv.Type = string(buff[:61])
		case 2:
			if DecodeF(string(buff), 3, neu[:]) < 3 {
				continue
			}
			pcv.Offset[0][0] = neu[1]
			pcv.Offset[0][1] = neu[0]
			pcv.Offset[0][2] = neu[2]
		case 3:
			DecodeF(string(buff), 10, pcv.Variation[0][:])
		case 4:
			DecodeF(string(buff), 9, pcv.Variation[0][10:])
		case 5:
			if DecodeF(string(buff), 3, neu[:]) < 3 {
				continue
			}
			pcv.Offset[1][0] = neu[1]
			pcv.Offset[1][1] = neu[0]
			pcv.Offset[1][2] = neu[2]
		case 6:
			DecodeF(string(buff), 10, pcv.Variation[1][:])
		case 7:
			DecodeF(string(buff), 9, pcv.Variation[1][10:])
			pcvs.AddPcv(&pcv)
		}
	}

	return 1
}

/* read antex file ----------------------------------------------------------*/
func ReadAntex(file string, pcvs *Pcvs) int {
	var (
		fp                   *os.File
		pcv0                 Pcv
		pcv                  Pcv
		neu                  [3]float64
		i, n, f, state, freq int
		freqs                []int = []int{1, 2, 5, 0}
	)

	Trace(5, "readantex: file=%s\n", file)

	if fp, _ = os.Open(file); fp == nil {
		Trace(2, "antex pcv file open error: %s\n", file)
		return 0
	}
	defer fp.Close()

	reader := bufio.NewReader(fp)
	for {
		buff, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		if len(buff) < 60 || strings.Contains(buff[60:], "COMMENT") {
			continue
		}

		if strings.Contains(buff[60:], "START OF ANTENNA") {
			pcv = pcv0
			state = 1
		}
		if strings.Contains(buff[60:], "END OF ANTENNA") {
			pcvs.AddPcv(&pcv)
			state = 0
		}
		if state == 0 {
			continue
		}

		switch {
		case strings.Contains(string(buff[60:]), "TYPE / SERIAL NO"):
			pcv.Type = string(buff[:20])
			pcv.Code = string(buff[20:40])
			if strings.Compare(string(pcv.Code[3:11]), "        ") == 0 {
				pcv.Sat = SatId2No(string(pcv.Code[:]))
			}
		case strings.Contains(string(buff[60:]), "VALID FROM"):
			if Str2Time(string(buff[:]), 0, 43, &pcv.Ts) == 0 {
				continue
			}
		case strings.Contains(string(buff[60:]), "VALID UNTIL"):
			if Str2Time(string(buff), 0, 43, &pcv.Te) == 0 {
				continue
			}
		case strings.Contains(string(buff[60:]), "START OF FREQUENCY"):
			if pcv.Sat == 0 && buff[3] != 'G' /* only read rec ant for GPS */ {
				continue
			}
			if n, _ = fmt.Sscanf(string(buff[4:]), "%d", &f); n < 1 {
				continue
			}
			for i = 0; freqs[i] > 0; i++ {
				if freqs[i] == f {
					break
				}
			}
			if freqs[i] != 0 {
				freq = i + 1
			}
		case strings.Contains(string(buff[60:]), "END OF FREQUENCY"):
			freq = 0
		case strings.Contains(string(buff[60:]), "NORTH / EAST / UP"):
			if freq < 1 || NFREQ < freq {
				continue
			}
			if DecodeF(string(buff[:]), 3, neu[:]) < 3 {
				continue
			}
			if pcv.Sat > 0 {
				pcv.Offset[freq-1][0] = neu[0] /* x or e */
				pcv.Offset[freq-1][1] = neu[1] /* y or n */

			} else {
				pcv.Offset[freq-1][0] = neu[1] /* x or e */
				pcv.Offset[freq-1][1] = neu[0] /* y or n */
			}
			pcv.Offset[freq-1][2] = neu[2] /* z or u */
		case strings.Contains(string(buff[:]), "NOAZI"):
			if freq < 1 || NFREQ < freq {
				continue
			}
			if i = DecodeF(string(buff[8:]), 19, pcv.Variation[freq-1][:]); i <= 0 {
				continue
			}
			for ; i < 19; i++ {
				pcv.Variation[freq-1][i] = pcv.Variation[freq-1][i-1]
			}
		}
	}

	return 1
}

/* read antenna parameters ------------------------------------------------------
* read antenna parameters
* args   : char   *file       I   antenna parameter file (antex)
*          pcvs_t *pcvs       IO  antenna parameters
* return : status (1:ok,0:file open error)
* notes  : file with the externsion .atx or .ATX is recognized as antex
*          file except for antex is recognized ngs antenna parameters
*          see reference [3]
*          only support non-azimuth-depedent parameters
*-----------------------------------------------------------------------------*/
func ReadPcv(file string, pcvs *Pcvs) int {
	var (
		pcv        *Pcv
		i, j, stat int
		ext        string
	)

	Trace(4, "readpcv: file=%s\n", file)
	if idx := strings.LastIndex(file, "."); idx >= 0 {
		ext = file[idx:]
	}

	if strings.EqualFold(ext, ".atx") {
		stat = ReadAntex(file, pcvs)
	} else {
		stat = ReadNgsPcv(file, pcvs)
	}
	for i = 0; i < pcvs.N(); i++ {
		pcv = &pcvs.Pcv[i]
		Trace(4, "sat=%2d type=%20s code=%s off=%8.4f %8.4f %8.4f  %8.4f %8.4f %8.4f\n",
			pcv.Sat, pcv.Type, pcv.Code, pcv.Offset[0][0], pcv.Offset[0][1],
			pcv.Offset[0][2], pcv.Offset[1][0], pcv.Offset[1][1], pcv.Offset[1][2])

		/* apply L2 to L3,L4,... if no pcv data */
		for j = 2; j < NFREQ; j++ { /* L3,L4,... */
			if Norm(pcv.Offset[j][:], 3) > 0.0 {
				continue
			}
			MatCpy(pcv.Offset[j][:], pcv.Offset[1][:], 3, 1)
			MatCpy(pcv.Variation[j][:], pcv.Variation[1][:], 19, 1)
		}
	}
	return stat
}

/* search antenna parameter ----------------------------------------------------
* read satellite antenna phase center position
* args   : int    sat         I   satellite number (0: receiver antenna)
*          char   *type       I   antenna type for receiver antenna
*          gtime_t time       I   time to search parameters
*          pcvs_t *pcvs       IO  antenna parameters
* return : antenna parameter (NULL: no antenna)
*-----------------------------------------------------------------------------*/
func SearchPcv(sat int, ctype string, time Gtime, pcvs *Pcvs) *Pcv {
	var (
		pcv     *Pcv
		types   [2]string
		i, j, n int
		buff, v string
	)

	Trace(3, "searchpcv: sat=%2d type=%s\n", sat, ctype)

	if sat > 0 { /* search satellite antenna */
		for i = 0; i < pcvs.N(); i++ {
			pcv = &pcvs.Pcv[i]
			if pcv.Sat != sat {
				continue
			}
			if pcv.Ts.Time != 0 && TimeDiff(pcv.Ts, time) > 0.0 {
				continue
			}
			if pcv.Te.Time != 0 && TimeDiff(pcv.Te, time) < 0.0 {
				continue
			}
			return pcv
		}
	} else {
		buff = ctype
		p := strings.Fields(buff)
		for i, v = range p {
			types[i] = v
		}
		if i <= 0 {
			return nil
		}
		n = i
		/* search receiver antenna with radome at first */
		for i = 0; i < pcvs.N(); i++ {
			pcv = &pcvs.Pcv[i]
			for j = 0; j < n; j++ {
				if strings.Index(string(pcv.Type[:]), types[j]) == 0 {
					break
				}
				if j >= n {
					return pcv
				}
			}
		}
		/* search receiver antenna without radome */
		for i = 0; i < pcvs.N(); i++ {
			pcv = &pcvs.Pcv[i]
			if strings.Index(string(pcv.Type[:]), types[0]) != 0 {
				continue
			}
			return pcv
		}
	}
	return nil
}

/* read station positions ------------------------------------------------------
* read positions from station position file
* args   : char  *file      I   station position file containing
*                               lat(deg) lon(deg) height(m) name in a line
*          char  *rcvs      I   station name
*          double *pos      O   station position {lat,lon,h} (rad/m)
*                               (all 0 if search error)
* return : none
*-----------------------------------------------------------------------------*/
func ReadPos(file, rcv string, pos []float64) {
	var (
		poss     [2048][3]float64
		stas     [2048][16]byte
		fp       *os.File
		i, j, np int
		str      string
	)

	Trace(3, "readpos: file=%s\n", file)
	fp, err := os.OpenFile(file, os.O_RDONLY, os.ModePerm)
	if err != nil {
		log.Printf("reference position file open error : %s\n", file)
		return
	}
	defer fp.Close()

	reader := bufio.NewReader(fp)
	for ; np < 2048; np++ {
		buff, err := reader.ReadBytes('\n')
		if err != nil {
			break
		}
		if buff[0] == '%' || buff[0] == '#' {
			continue
		}
		if n, _ := fmt.Sscanf(string(buff[:]), "%f %f %f %s", &poss[np][0], &poss[np][1], &poss[np][2], str); n < 4 {
			continue
		}
		copy(stas[np][:], fmt.Sprintf("%.15s", str))
	}

	length := len(rcv)
	for i = 0; i < np; i++ {
		if strings.Compare(string(stas[i][:length]), rcv) != 0 {
			continue
		}
		for j = 0; j < 3; j++ {
			pos[j] = poss[i][j]
		}
		pos[0] *= D2R
		pos[1] *= D2R
		return
	}
	pos[0], pos[1], pos[2] = 0.0, 0.0, 0.0
}

/* read blq record -----------------------------------------------------------*/
func readblqrecord(fp *os.File, odisp []float64) int {
	var (
		v    [11]float64
		i, n int
	)

	reader := bufio.NewReader(fp)
	for {
		buff, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		if strings.Compare(buff[:2], "$$") == 0 {
			continue
		}
		if n, _ := fmt.Sscanf(buff, "%f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10]); n < 11 {
			continue
		}
		for i = 0; i < 11; i++ {
			odisp[n+i*6] = v[i]
		}

		if n++; n == 6 {
			return 1
		}
	}
	return 0
}

/* read blq ocean tide loading parameters --------------------------------------
* read blq ocean tide loading parameters
* args   : char   *file       I   BLQ ocean tide loading parameter file
*          char   *sta        I   station name
*          double *odisp      O   ocean tide loading parameters
* return : status (1:ok,0:file open error)
*-----------------------------------------------------------------------------*/
func ReadBlq(file, sta string, odisp []float64) int {
	fp, err := os.Open(file)
	if err != nil {
		log.Printf("blq file open error: file=%s\n", file)
		return 0
	}
	defer fp.Close()

	var name, staname string
	/* station name to upper case */
	fmt.Sscanf(sta, "%16s", staname)
	staname = strings.ToUpper(staname)
	reader := bufio.NewReader(fp)
	for {
		buff, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		if strings.Compare(buff[:2], "$$") == 0 {
			continue
		}

		if n, _ := fmt.Sscanf(buff[2:], "%16s", name); n < 1 {
			continue
		}

		name = strings.ToUpper(name)

		if strings.Compare(name, staname) != 0 {
			continue
		}

		/* read blq record */
		if readblqrecord(fp, odisp) != 0 {

			return 1
		}
	}
	Trace(2, "no otl parameters: sta=%s file=%s\n", sta, file)
	return 0
}

/* read earth rotation parameters ----------------------------------------------
* read earth rotation parameters
* args   : char   *file       I   IGS ERP file (IGS ERP ver.2)
*          erp_t  *erp        O   earth rotation parameters
* return : status (1:ok,0:file open error)
*-----------------------------------------------------------------------------*/
func ReadErp(file string, erp *Erp) int {
	var fp *os.File
	var v [14]float64 = [14]float64{0}
	var n int
	var data ErpD

	Trace(4, "readerp: file=%s\n", file)

	fp, err := os.Open(file)
	if err != nil {
		log.Printf("erp file open error: file=%s\n", file)
		return 0
	}
	defer fp.Close()

	reader := bufio.NewReader(fp)
	for ; ; n++ {
		buff, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		if n, _ := fmt.Sscanf(buff, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
			&v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11], &v[12], &v[13]); n < 5 {
			continue
		}
		// if erp.N >= erp.Nmax {
		// 	if erp.Nmax <= 0 {
		// 		erp.Nmax = 128
		// 	} else {
		// 		erp.Nmax = erp.Nmax * 2
		// 	}
		// 	erp.Data = make([]ErpD, erp.Nmax)

		// }
		data.Mjd = v[0]
		data.Xp = v[1] * 1e-6 * (AS2R)
		data.Yp = v[2] * 1e-6 * AS2R
		data.Ut1_utc = v[3] * 1e-7
		data.Lod = v[4] * 1e-7
		data.Xpr = v[12] * 1e-6 * AS2R
		data.Ypr = v[13] * 1e-6 * AS2R
		erp.Data = append(erp.Data, data)
		// erp.Data[erp.N].Mjd = v[0]
		// erp.Data[erp.N].Xp = v[1] * 1e-6 * (AS2R)
		// erp.Data[erp.N].Yp = v[2] * 1e-6 * AS2R
		// erp.Data[erp.N].Ut1_utc = v[3] * 1e-7
		// erp.Data[erp.N].Lod = v[4] * 1e-7
		// erp.Data[erp.N].Xpr = v[12] * 1e-6 * AS2R
		// erp.Data[erp.N].Ypr = v[13] * 1e-6 * AS2R
	}

	return 1
}

/* get earth rotation parameter values -----------------------------------------
* get earth rotation parameter values
* args   : erp_t  *erp        I   earth rotation parameters
*          gtime_t time       I   time (gpst)
*          double *erpv       O   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
func GetErp(erp *Erp, time Gtime, erpv []float64) int {
	var (
		ep          []float64 = []float64{2000, 1, 1, 12, 0, 0}
		mjd, day, a float64
		i, j, k     int
	)

	Trace(4, "geterp:\n")

	if erp.N() <= 0 {
		return 0
	}

	mjd = 51544.5 + (TimeDiff(GpsT2Utc(time), Epoch2Time(ep)))/86400.0

	if mjd <= erp.Data[0].Mjd {
		day = mjd - erp.Data[0].Mjd
		erpv[0] = erp.Data[0].Xp + erp.Data[0].Xpr*day
		erpv[1] = erp.Data[0].Yp + erp.Data[0].Ypr*day
		erpv[2] = erp.Data[0].Ut1_utc - erp.Data[0].Lod*day
		erpv[3] = erp.Data[0].Lod
		return 1
	}
	if mjd >= erp.Data[erp.N()-1].Mjd {
		day = mjd - erp.Data[erp.N()-1].Mjd
		erpv[0] = erp.Data[erp.N()-1].Xp + erp.Data[erp.N()-1].Xpr*day
		erpv[1] = erp.Data[erp.N()-1].Yp + erp.Data[erp.N()-1].Ypr*day
		erpv[2] = erp.Data[erp.N()-1].Ut1_utc - erp.Data[erp.N()-1].Lod*day
		erpv[3] = erp.Data[erp.N()-1].Lod
		return 1
	}
	for k = erp.N() - 1; j < k-1; {
		i = (j + k) / 2
		if mjd < erp.Data[i].Mjd {
			k = i
		} else {
			j = i
		}
	}
	if erp.Data[j].Mjd == erp.Data[j+1].Mjd {
		a = 0.5
	} else {
		a = (mjd - erp.Data[j].Mjd) / (erp.Data[j+1].Mjd - erp.Data[j].Mjd)
	}
	erpv[0] = (1.0-a)*erp.Data[j].Xp + a*erp.Data[j+1].Xp
	erpv[1] = (1.0-a)*erp.Data[j].Yp + a*erp.Data[j+1].Yp
	erpv[2] = (1.0-a)*erp.Data[j].Ut1_utc + a*erp.Data[j+1].Ut1_utc
	erpv[3] = (1.0-a)*erp.Data[j].Lod + a*erp.Data[j+1].Lod
	return 1
}

/* compare ephemeris ---------------------------------------------------------*/
func cmpeph(p1, p2 *Eph) int {
	if p1.Ttr.Time != p2.Ttr.Time {
		return int(p1.Ttr.Time - p2.Ttr.Time)
	} else {
		if p1.Toe.Time != p2.Toe.Time {
			return int(p1.Toe.Time - p2.Toe.Time)
		} else {
			return p1.Sat - p2.Sat
		}
	}

}

/* sort and unique ephemeris -------------------------------------------------*/
func (nav *Nav) UniqEph() {
	var i, j int

	Trace(4, "uniqeph: n=%d\n", nav.N())

	if nav.N() <= 0 {
		return
	}

	sort.Slice(nav.Ephs, func(i, j int) bool {
		return cmpeph(&nav.Ephs[i], &nav.Ephs[j]) < 0
	})

	for i, j = 1, 0; i < nav.N(); i++ {
		if nav.Ephs[i].Sat != nav.Ephs[j].Sat ||
			nav.Ephs[i].Iode != nav.Ephs[j].Iode {
			j++
			nav.Ephs[j] = nav.Ephs[i]
		}
	}
	nav.Ephs = nav.Ephs[:j+1]

	Trace(5, "uniqeph: n=%d\n", nav.N())
}

/* compare glonass ephemeris -------------------------------------------------*/
func cmpgeph(q1 *GEph, q2 *GEph) int {
	if q1.Tof.Time != q2.Tof.Time {
		return int(q1.Tof.Time - q2.Tof.Time)
	} else if q1.Toe.Time != q2.Toe.Time {
		return int(q1.Toe.Time - q2.Toe.Time)
	} else {
		return q1.Sat - q2.Sat
	}

}

/* sort and unique glonass ephemeris -----------------------------------------*/
func (nav *Nav) UniqGEph() {
	var i, j int

	Trace(4, "uniqgeph: ng=%d\n", nav.Ng())

	if nav.Ng() <= 0 {
		return
	}

	sort.Slice(nav.Geph, func(i, j int) bool {
		return cmpgeph(&nav.Geph[i], &nav.Geph[j]) < 0
	})

	for i, j = 0, 0; i < nav.Ng(); i++ {
		if nav.Geph[i].Sat != nav.Geph[j].Sat ||
			nav.Geph[i].Toe.Time != nav.Geph[j].Toe.Time ||
			nav.Geph[i].Svh != nav.Geph[j].Svh {
			j++
			nav.Geph[j] = nav.Geph[i]
		}
	}
	nav.Geph = nav.Geph[:j+1]

	Trace(5, "uniqgeph: ng=%d\n", nav.Ng())
}

/* compare sbas ephemeris ----------------------------------------------------*/
func cmpseph(q1, q2 *SEph) int {
	if q1.Tof.Time != q2.Tof.Time {
		return int(q1.Tof.Time - q2.Tof.Time)
	} else if q1.T0.Time != q2.T0.Time {
		return int(q1.T0.Time - q2.T0.Time)
	} else {
		return q1.Sat - q2.Sat
	}
}

/* sort and unique sbas ephemeris --------------------------------------------*/
func (nav *Nav) UniqSEph() {
	var i, j int

	Trace(4, "uniqseph: ns=%d\n", nav.Ns())

	if nav.Ns() <= 0 {
		return
	}

	sort.Slice(nav.Seph, func(i, j int) bool {
		return cmpseph(&nav.Seph[i], &nav.Seph[j]) < 0
	})

	for i, j = 0, 0; i < nav.Ns(); i++ {
		if nav.Seph[i].Sat != nav.Seph[j].Sat ||
			nav.Seph[i].T0.Time != nav.Seph[j].T0.Time {
			j++
			nav.Seph[j] = nav.Seph[i]
		}
	}
	nav.Seph = nav.Seph[:j+1]

	Trace(5, "uniqseph: ns=%d\n", nav.Ns())
}

/* unique ephemerides ----------------------------------------------------------
* unique ephemerides in navigation data
* args   : nav_t *nav    IO     navigation data
* return : number of epochs
*-----------------------------------------------------------------------------*/
func (nav *Nav) UniqNav() {
	Trace(4, "uniqnav: neph=%d ngeph=%d nseph=%d\n", nav.N, nav.Ng, nav.Ns)

	/* unique ephemeris */
	nav.UniqEph()
	nav.UniqGEph()
	nav.UniqSEph()
}

/* compare observation data -------------------------------------------------*/
func cmpobs(q1, q2 *ObsD) int {
	tt := TimeDiff(q1.Time, q2.Time)
	if float32(math.Abs(tt)) > DTTOL {
		if tt < 0 {
			return -1
		} else {
			return 1
		}
	}
	if q1.Rcv != q2.Rcv {
		return int(q1.Rcv - q2.Rcv)
	}
	return int(q1.Sat - q2.Sat)
}

/* sort and unique observation data --------------------------------------------
* sort and unique observation data by time, rcv, sat
* args   : obs_t *obs    IO     observation data
* return : number of epochs
*-----------------------------------------------------------------------------*/
func (obs *Obs) SortObs() int {
	var i, j, n int

	Trace(4, "sortobs: nobs=%d\n", obs.N())

	if obs.N() <= 0 {
		return 0
	}

	sort.Slice(obs.Data, func(i, j int) bool {
		return cmpobs(&obs.Data[i], &obs.Data[j]) < 0
	})

	/* delete duplicated data */
	for i, j = 0, 0; i < obs.N(); i++ {
		if obs.Data[i].Sat != obs.Data[j].Sat ||
			obs.Data[i].Rcv != obs.Data[j].Rcv ||
			TimeDiff(obs.Data[i].Time, obs.Data[j].Time) != 0.0 {
			j++
			obs.Data[j] = obs.Data[i]
		}
	}
	obs.Data = obs.Data[:j+1]

	for i, n = 0, 0; i < obs.N(); i, n = j, n+1 {
		for j = i + 1; j < obs.N(); j++ {
			if math.Abs(TimeDiff(obs.Data[j].Time, obs.Data[i].Time)) > float64(DTTOL) {
				break
			}
		}
	}
	return n
}

/* screen by time --------------------------------------------------------------
* screening by time start, time end, and time interval
* args   : gtime_t time  I      time
*          gtime_t ts    I      time start (ts.time==0:no screening by ts)
*          gtime_t te    I      time end   (te.time==0:no screening by te)
*          double  tint  I      time interval (s) (0.0:no screen by tint)
* return : 1:on condition, 0:not on condition
*-----------------------------------------------------------------------------*/
func ScreenTime(time, ts, te Gtime, tint float64) int {
	if (tint <= 0.0 || float32(math.Mod(Time2GpsT(time, nil)+float64(DTTOL), tint)) <= DTTOL*2.0) &&
		(ts.Time == 0 || TimeDiff(time, ts) >= float64(-DTTOL)) &&
		(te.Time == 0 || TimeDiff(time, te) < float64(DTTOL)) {
		return 1
	} else {
		return 0
	}

}

/* read/save navigation data ---------------------------------------------------
* save or load navigation data
* args   : char    file  I      file path
*          nav_t   nav   O/I    navigation data
* return : status (1:ok,0:no file)
*-----------------------------------------------------------------------------*/
func (nav *Nav) ReadNav(file string) int {
	var (
		fp    *os.File
		eph0  Eph
		geph0 GEph

		toe_time, tof_time, toc_time, ttr_time uint64
		i, sat, prn, n                         int
	)

	Trace(4, "loadnav: file=%s\n", file)

	fp, err := os.Open(file)
	if err != nil {
		Trace(2, "nav file open error: file=%s\n", file)
		return 0
	}
	defer fp.Close()

	if nav == nil {
		Trace(2, "nav is not initialized\n")
		return 0
	}

	reader := bufio.NewReader(fp)
	for ; ; n++ {
		buff, err := reader.ReadString('\n')
		if err == io.EOF {
			break
		}

		if strings.Compare(buff[:6], "IONUTC") == 0 {
			for i = 0; i < 8; i++ {
				nav.Ion_gps[i] = 0.0
			}
			for i = 0; i < 8; i++ {
				nav.Utc_gps[i] = 0.0
			}
			fmt.Sscanf(buff, "IONUTC,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&nav.Ion_gps[0], &nav.Ion_gps[1], &nav.Ion_gps[2], &nav.Ion_gps[3],
				&nav.Ion_gps[4], &nav.Ion_gps[5], &nav.Ion_gps[6], &nav.Ion_gps[7],
				&nav.Utc_gps[0], &nav.Utc_gps[1], &nav.Utc_gps[2], &nav.Utc_gps[3],
				&nav.Utc_gps[4])
			continue

		}
		if n = strings.Index(buff, ","); n > 0 {
			//strings.TrimRight(buff, ",")
		} else {
			continue
		}
		if sat = SatId2No(buff[:n]); sat == 0 {
			continue
		}
		if SatSys(sat, &prn) == SYS_GLO {
			var tmp = geph0

			tmp.Sat = sat
			toe_time, tof_time = 0, 0
			fmt.Sscanf(buff[n+1:], "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&tmp.Iode, &tmp.Frq, &tmp.Svh,
				&tmp.Sva, &tmp.Age,
				&toe_time, &tof_time,
				&tmp.Pos[0], &tmp.Pos[1], &tmp.Pos[2],
				&tmp.Vel[0], &tmp.Vel[1], &tmp.Vel[2],
				&tmp.Acc[0], &tmp.Acc[1], &tmp.Acc[2],
				&tmp.Taun, &tmp.Gamn, &tmp.DTaun)
			tmp.Toe.Time = toe_time
			tmp.Tof.Time = tof_time
			nav.Geph[prn-1] = tmp
		} else {
			var tmp = eph0

			tmp.Sat = sat
			toe_time, toc_time, ttr_time = 0, 0, 0
			fmt.Sscanf(buff[n+1:], "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d",
				&tmp.Iode, &tmp.Iodc, &tmp.Sva,
				&tmp.Svh,
				&toe_time, &toc_time, &ttr_time,
				&tmp.A, &tmp.E, &tmp.I0,
				&tmp.OMG0, &tmp.Omg, &tmp.M0,
				&tmp.Deln, &tmp.OMGd, &tmp.Idot,
				&tmp.Crc, &tmp.Crs, &tmp.Cuc,
				&tmp.Cus, &tmp.Cic, &tmp.Cis,
				&tmp.Toes, &tmp.Fit, &tmp.F0,
				&tmp.F1, &tmp.F2, &tmp.Tgd[0],
				&tmp.Code, &tmp.Flag)
			tmp.Toe.Time = toe_time
			tmp.Toc.Time = toc_time
			tmp.Ttr.Time = ttr_time
			nav.Ephs[sat-1] = tmp
		}
	}
	return 1
}

func (nav *Nav) SaveNav(file string) int {
	var (
		fp      *os.File
		i       int
		id, tmp string
	)

	Trace(4, "savenav: file=%s\n", file)

	fp, err := os.OpenFile(file, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModeAppend|os.ModePerm)
	if err != nil {
		Trace(2, "nav file open error: file=%s\n", file)
		return 0
	}
	defer fp.Close()

	if nav == nil {
		Trace(2, "nav is not initialized\n")
		return 0
	}
	if nav.Ephs == nil {
		Trace(2, "nav eph is not initialized\n")
		return 0
	}
	if nav.Geph == nil {
		Trace(2, "nav geph is not initialized\n")
		return 0
	}
	if nav.Seph == nil {
		Trace(2, "nav seph is not initialized\n")
		return 0
	}

	for i = 0; i < MAXSAT; i++ {
		if nav.Ephs[i].Ttr.Time == 0 {
			continue
		}
		SatNo2Id(nav.Ephs[i].Sat, &id)
		tmp = fmt.Sprintf("%s,%d,%d,%d,%d,%d,%d,%d,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E, %.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%d,%d\n",
			id, nav.Ephs[i].Iode, nav.Ephs[i].Iodc, nav.Ephs[i].Sva,
			nav.Ephs[i].Svh, nav.Ephs[i].Toe.Time,
			nav.Ephs[i].Toc.Time, nav.Ephs[i].Ttr.Time,
			nav.Ephs[i].A, nav.Ephs[i].E, nav.Ephs[i].I0, nav.Ephs[i].OMG0,
			nav.Ephs[i].Omg, nav.Ephs[i].M0, nav.Ephs[i].Deln, nav.Ephs[i].OMGd,
			nav.Ephs[i].Idot, nav.Ephs[i].Crc, nav.Ephs[i].Crs, nav.Ephs[i].Cuc,
			nav.Ephs[i].Cus, nav.Ephs[i].Cic, nav.Ephs[i].Cis, nav.Ephs[i].Toes,
			nav.Ephs[i].Fit, nav.Ephs[i].F0, nav.Ephs[i].F1, nav.Ephs[i].F2,
			nav.Ephs[i].Tgd[0], nav.Ephs[i].Code, nav.Ephs[i].Flag)
		fp.WriteString(tmp)
	}
	for i = 0; i < MAXPRNGLO; i++ {
		if nav.Geph[i].Tof.Time == 0 {
			continue
		}
		SatNo2Id(nav.Geph[i].Sat, &id)
		tmp = fmt.Sprintf("%s,%d,%d,%d,%d,%d,%d,%d,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E\n",
			id, nav.Geph[i].Iode, nav.Geph[i].Frq, nav.Geph[i].Svh,
			nav.Geph[i].Sva, nav.Geph[i].Age, nav.Geph[i].Toe.Time,
			nav.Geph[i].Tof.Time,
			nav.Geph[i].Pos[0], nav.Geph[i].Pos[1], nav.Geph[i].Pos[2],
			nav.Geph[i].Vel[0], nav.Geph[i].Vel[1], nav.Geph[i].Vel[2],
			nav.Geph[i].Acc[0], nav.Geph[i].Acc[1], nav.Geph[i].Acc[2],
			nav.Geph[i].Taun, nav.Geph[i].Gamn, nav.Geph[i].DTaun)
		fp.WriteString(tmp)
	}

	tmp = fmt.Sprintf("IONUTC,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.0f",
		nav.Ion_gps[0], nav.Ion_gps[1], nav.Ion_gps[2], nav.Ion_gps[3],
		nav.Ion_gps[4], nav.Ion_gps[5], nav.Ion_gps[6], nav.Ion_gps[7],
		nav.Utc_gps[0], nav.Utc_gps[1], nav.Utc_gps[2], nav.Utc_gps[3],
		nav.Utc_gps[4])
	fp.WriteString(tmp)

	return 1
}

/* free observation data -------------------------------------------------------
* free memory for observation data
* args   : obs_t *obs    IO     observation data
* return : none
*-----------------------------------------------------------------------------*/
func (obs *Obs) FreeObs() {
	obs.Data = nil
}

/* free navigation data ---------------------------------------------------------
* free memory for navigation data
* args   : nav_t *nav    IO     navigation data
*          int   opt     I      option (or of followings)
*                               (0x01: gps/qzs ephmeris, 0x02: glonass ephemeris,
*                                0x04: sbas ephemeris,   0x08: precise ephemeris,
*                                0x10: precise clock     0x20: almanac,
*                                0x40: tec data)
* return : none
*-----------------------------------------------------------------------------*/
func (nav *Nav) FreeNav(opt int) {
	if opt&0x01 == 1 {
		nav.Ephs = nil
	}
	if opt&0x02 == 1 {
		nav.Geph = nil
	}
	if opt&0x04 == 1 {
		nav.Seph = nil
	}
	if opt&0x08 == 1 {
		nav.Peph = nil
	}
	if opt&0x10 == 1 {
		nav.Pclk = nil
	}
	if opt&0x20 == 1 {
		nav.Alm = nil
	}
	if opt&0x40 == 1 {
		nav.Tec = nil
	}
}

var fp_trace *os.File
var file_trace string
var level_trace int
var tick_trace int64 = 0 /* tick time at traceopen (ms) */
var time_trace Gtime     /* time at traceopen */
/* debug trace functions -----------------------------------------------------*/
func traceswap() {
	time := Utc2GpsT(TimeGet())
	var path string

	if int(Time2GpsT(time, nil)/float64(INT_SWAP_TRAC)) ==
		int(Time2GpsT(time_trace, nil)/float64(INT_SWAP_TRAC)) {
		return
	}
	time_trace = time

	if RepPath(file_trace, &path, time, "", "") == 0 {
		return
	}
	if fp_trace != nil {
		fp_trace.Close()
	}

	var err error
	fp_trace, err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644)
	if err != nil {
		Trace(2, "open log file failed, err:%s", err)
		fp_trace = os.Stderr
		return
	}
}
func TraceOpen(file string) {
	time := Utc2GpsT(TimeGet())
	var path string

	RepPath(file, &path, time, "", "")
	if len(path) == 0 {
		fp_trace = os.Stdout
	} else {
		var err error
		fp_trace, err = os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644)
		if err != nil {
			Trace(2, "open log file failed, err:%s", err)
			return
		}
	}
	tick_trace = TickGet()
	log.SetOutput(fp_trace)
	file_trace = file
}
func TraceClose() {
	if fp_trace != nil && fp_trace != os.Stderr {
		fp_trace.Close()
	}
	fp_trace = nil
	file_trace = ""
}
func TraceLevel(level int) {
	level_trace = level
}
func Trace(level int, format string, v ...interface{}) {
	/* print error message to stderr */
	if level <= 1 {
		fmt.Printf(format, v...)
	}
	if fp_trace == nil || level > level_trace {
		return
	}
	traceswap()
	fmt.Fprintf(fp_trace, "%d ", level)
	fmt.Fprintf(fp_trace, format, v...)
}
func Tracet(level int, format string, v ...interface{}) {

	if fp_trace == nil || level > level_trace {
		return
	}
	traceswap()
	fmt.Fprintf(fp_trace, "%d %9.3f: ", level, float64(TickGet()-int64(tick_trace))/1000.0)
	fmt.Fprintf(fp_trace, format, v...)
}
func tracemat(level int, A []float64, n, m, p, q int) {
	if fp_trace == nil || level > level_trace {
		return
	}
	matfprint(A, n, m, p, q, fp_trace)
}
func traceobs(level int, obs []ObsD, n int) {
	var str, id string

	if fp_trace == nil || level > level_trace {
		return
	}
	for i := 0; i < n; i++ {
		Time2Str(obs[i].Time, &str, 3)
		SatNo2Id(obs[i].Sat, &id)
		fmt.Fprintf(fp_trace, " (%2d) %s %-3s rcv%d %13.3f %13.3f %13.3f %13.3f %d %d %d %d %3.1f %3.1f\n",
			i+1, str, id, obs[i].Rcv, obs[i].L[0], obs[i].L[1], obs[i].P[0],
			obs[i].P[1], obs[i].LLI[0], obs[i].LLI[1], obs[i].Code[0],
			obs[i].Code[1], float32(obs[i].SNR[0])*SNR_UNIT, float32(obs[i].SNR[1])*SNR_UNIT)
	}
}
func (nav *Nav) TraceNav(level int) {
	var s1, s2, id string

	if fp_trace == nil || level > level_trace {
		return
	}
	for i := 0; i < nav.N(); i++ {
		Time2Str(nav.Ephs[i].Toe, &s1, 0)
		Time2Str(nav.Ephs[i].Ttr, &s2, 0)
		SatNo2Id(nav.Ephs[i].Sat, &id)
		Trace(1, "(%3d) %-3s : %s %s %3d %3d %02x\n", i+1,
			id, s1, s2, nav.Ephs[i].Iode, nav.Ephs[i].Iodc, nav.Ephs[i].Svh)
	}
	fmt.Fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav.Ion_gps[0],
		nav.Ion_gps[1], nav.Ion_gps[2], nav.Ion_gps[3])
	fmt.Fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav.Ion_gps[4],
		nav.Ion_gps[5], nav.Ion_gps[6], nav.Ion_gps[7])
	fmt.Fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav.Ion_gal[0],
		nav.Ion_gal[1], nav.Ion_gal[2], nav.Ion_gal[3])
}
func (nav *Nav) tracegnav(level int) {
	var s1, s2, id string

	if fp_trace == nil || level > level_trace {
		return
	}
	for i := 0; i < nav.Ng(); i++ {
		Time2Str(nav.Geph[i].Toe, &s1, 0)
		Time2Str(nav.Geph[i].Tof, &s2, 0)
		SatNo2Id(nav.Geph[i].Sat, &id)
		fmt.Fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d %8.3f\n", i+1,
			id, s1, s2, nav.Geph[i].Frq, nav.Geph[i].Svh, nav.Geph[i].Taun*1e6)
	}
}
func (nav *Nav) tracehnav(level int) {
	var s1, s2, id string

	if fp_trace == nil || level > level_trace {
		return
	}
	for i := 0; i < nav.Ns(); i++ {
		Time2Str(nav.Seph[i].T0, &s1, 0)
		Time2Str(nav.Seph[i].Tof, &s2, 0)
		SatNo2Id(nav.Seph[i].Sat, &id)
		fmt.Fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d\n", i+1,
			id, s1, s2, nav.Seph[i].Svh, nav.Seph[i].Sva)
	}
}
func (nav *Nav) tracepeph(level int) {
	var s, id string

	if fp_trace == nil || level > level_trace {
		return
	}

	for i := 0; i < nav.Ne(); i++ {
		Time2Str(nav.Peph[i].Time, &s, 0)
		for j := 0; j < MAXSAT; j++ {
			SatNo2Id(j+1, &id)
			fmt.Fprintf(fp_trace, "%-3s %d %-3s %13.3f %13.3f %13.3f %13.3f %6.3f %6.3f %6.3f %6.3f\n",
				s, nav.Peph[i].Index, id,
				nav.Peph[i].Pos[j][0], nav.Peph[i].Pos[j][1],
				nav.Peph[i].Pos[j][2], nav.Peph[i].Pos[j][3]*1e9,
				nav.Peph[i].Std[j][0], nav.Peph[i].Std[j][1],
				nav.Peph[i].Std[j][2], nav.Peph[i].Std[j][3]*1e9)
		}
	}
}
func (nav *Nav) tracepclk(level int) {
	var s, id string

	if fp_trace == nil || level > level_trace {
		return
	}

	for i := 0; i < nav.Nc(); i++ {
		Time2Str(nav.Pclk[i].Time, &s, 0)
		for j := 0; j < MAXSAT; j++ {
			SatNo2Id(j+1, &id)
			fmt.Fprintf(fp_trace, "%-3s %d %-3s %13.3f %6.3f\n",
				s, nav.Pclk[i].Index, id,
				nav.Pclk[i].Clk[j][0]*1e9, nav.Pclk[i].Std[j][0]*1e9)
		}
	}
}
func Traceb(level int, p []uint8, n int) {
	if fp_trace == nil || level > level_trace {
		return
	}
	for i := 0; i < n; i++ {
		if i%8 == 7 {
			fmt.Fprintf(fp_trace, "%02X%s", p[i], " ")
		} else {
			fmt.Fprintf(fp_trace, "%02X%s", p[i], "")
		}
	}
	Trace(5, "\n")
}

/* execute command -------------------------------------------------------------
* execute command line by operating system shell
* args   : char   *cmd      I   command line
* return : execution status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
func ExecCmd(cmd string) int {

	cmdagrs := strings.Fields(cmd)
	cmdObj := exec.Command(cmdagrs[0], cmdagrs[1:]...)
	err := cmdObj.Start()
	if err == nil {
		return 0
	} else {
		return 1
	}

}

/* expand file path ------------------------------------------------------------
* expand file path with wild-card (*) in file
* args   : char   *path     I   file path to expand (captal insensitive)
*          char   *paths    O   expanded file paths
*          int    nmax      I   max number of expanded file paths
* return : number of expanded file paths
* notes  : the order of expanded files is alphabetical order
*-----------------------------------------------------------------------------*/
func ExPath(path string, paths []string, nmax int) int {
	// var err error
	matchs, err := filepath.Glob(path)

	if err != nil {
		return 0
	}
	if paths == nil {
		return 0
	}
	var n int = 0
	for i, v := range matchs {
		if i >= nmax {
			break
		}
		paths[i] = v
		n++
	}
	return n
}

/* generate local directory recursively --------------------------------------*/
func mkdir_r(dir string) int {

	err := os.Mkdir(dir, os.ModeDir|os.ModePerm)
	if err != nil && !os.IsExist(err) {
		return 0
	}
	return 1
}

/* create directory ------------------------------------------------------------
* create directory if not exists
* args   : char   *path     I   file path to be saved
* return : none
* notes  : recursively.
*-----------------------------------------------------------------------------*/
func CreateDir(path string) {
	var buff string
	Trace(4, "createdir: path=%s\n", path)
	buff = path
	if index := strings.LastIndex(path, FILEPATHSEP); index > 0 {
		buff = path[:index]
	} else {
		return
	}

	mkdir_r(buff)
}

/* replace string ------------------------------------------------------------*/
func RepStr(str *string, pat, rep string) int {
	if !strings.Contains(*str, pat) {
		return 0
	}
	*str = strings.Replace(*str, pat, rep, -1)
	return 1
}

/* replace keywords in file path -----------------------------------------------
* replace keywords in file path with date, time, rover and base station id
* args   : char   *path     I   file path (see below)
*          char   *rpath    O   file path in which keywords replaced (see below)
*          gtime_t time     I   time (gpst)  (time.time==0: not replaced)
*          char   *rov      I   rover id string        ("": not replaced)
*          char   *base     I   base station id string ("": not replaced)
* return : status (1:keywords replaced, 0:no valid keyword in the path,
*                  -1:no valid time)
* notes  : the following keywords in path are replaced by date, time and name
*              %Y . yyyy : year (4 digits) (1900-2099)
*              %y . yy   : year (2 digits) (00-99)
*              %m . mm   : month           (01-12)
*              %d . dd   : day of month    (01-31)
*              %h . hh   : hours           (00-23)
*              %M . mm   : minutes         (00-59)
*              %S . ss   : seconds         (00-59)
*              %n . ddd  : day of year     (001-366)
*              %W . wwww : gps week        (0001-9999)
*              %D . d    : day of gps week (0-6)
*              %H . h    : hour code       (a=0,b=1,c=2,...,x=23)
*              %ha. hh   : 3 hours         (00,03,06,...,21)
*              %hb. hh   : 6 hours         (00,06,12,18)
*              %hc. hh   : 12 hours        (00,12)
*              %t . mm   : 15 minutes      (00,15,30,45)
*              %r . rrrr : rover id
*              %b . bbbb : base station id
*-----------------------------------------------------------------------------*/
func RepPath(path string, rpath *string, time Gtime, rov, base string) int {
	var ep [6]float64
	var ep0 [6]float64 = [6]float64{2000, 1, 1, 0, 0, 0}
	var week, dow, doy, stat int = 0, 0, 0, 0
	//char rep[64];
	var rep string
	*rpath = path

	if !strings.Contains(path, "%") {
		return 0
	}
	if len(rov) > 0 {
		stat |= RepStr(rpath, "%r", rov)
	}
	if len(base) > 0 {
		stat |= RepStr(rpath, "%b", base)
	}
	if time.Time != 0 {
		Time2Epoch(time, ep[:])
		ep0[0] = ep[0]
		dow = int(math.Floor(Time2GpsT(time, &week) / 86400.0))
		doy = int(math.Floor(TimeDiff(time, Epoch2Time(ep0[:]))/86400.0)) + 1
		rep = fmt.Sprintf("%02d", int(ep[3]/3)*3)
		stat |= RepStr(rpath, "%ha", rep)
		rep = fmt.Sprintf("%02d", int(ep[3]/6)*6)
		stat |= RepStr(rpath, "%hb", rep)
		rep = fmt.Sprintf("%02d", int(ep[3]/12)*12)
		stat |= RepStr(rpath, "%hc", rep)
		rep = fmt.Sprintf("%04.0f", ep[0])
		stat |= RepStr(rpath, "%Y", rep)
		rep = fmt.Sprintf("%02.0f", math.Mod(ep[0], 100.0))
		stat |= RepStr(rpath, "%y", rep)
		rep = fmt.Sprintf("%02.0f", ep[1])
		stat |= RepStr(rpath, "%m", rep)
		rep = fmt.Sprintf("%02.0f", ep[2])
		stat |= RepStr(rpath, "%d", rep)
		rep = fmt.Sprintf("%02.0f", ep[3])
		stat |= RepStr(rpath, "%h", rep)
		rep = fmt.Sprintf("%02.0f", ep[4])
		stat |= RepStr(rpath, "%M", rep)
		rep = fmt.Sprintf("%02.0f", math.Floor(ep[5]))
		stat |= RepStr(rpath, "%S", rep)
		rep = fmt.Sprintf("%03d", doy)
		stat |= RepStr(rpath, "%n", rep)
		rep = fmt.Sprintf("%04d", week)
		stat |= RepStr(rpath, "%W", rep)
		rep = fmt.Sprintf("%d", dow)
		stat |= RepStr(rpath, "%D", rep)
		rep = fmt.Sprintf("%c", 'a'+int(ep[3]))
		stat |= RepStr(rpath, "%H", rep)
		rep = fmt.Sprintf("%02d", int(ep[4]/15)*15)
		stat |= RepStr(rpath, "%t", rep)
	} else {
		var tmps []string = []string{"%ha", "%hb", "%hc", "%Y", "%y", "%m", "%d", "%h", "%M", "%S", "%n", "%W", "%D", "%H", "%t"}
		for k := range tmps {
			if strings.Contains(*rpath, tmps[k]) {
				return -1 /* no valid time */
			}
		}
	}
	return stat
}

/* replace keywords in file path and generate multiple paths -------------------
* replace keywords in file path with date, time, rover and base station id
* generate multiple keywords-replaced paths
* args   : char   *path     I   file path (see below)
*          char   *rpath[]  O   file paths in which keywords replaced
*          int    nmax      I   max number of output file paths
*          gtime_t ts       I   time start (gpst)
*          gtime_t te       I   time end   (gpst)
*          char   *rov      I   rover id string        ("": not replaced)
*          char   *base     I   base station id string ("": not replaced)
* return : number of replaced file paths
* notes  : see reppath() for replacements of keywords.
*          minimum interval of time replaced is 900s.
*-----------------------------------------------------------------------------*/
func RepPaths(path string, rpath []string, nmax int, ts, te Gtime, rov, base string) int {
	var time Gtime
	var tow, tint float64 = 0, 86400.0
	var i, n, week int = 0, 0, 0

	Trace(4, "reppaths: path =%s nmax=%d rov=%s base=%s\n", path, nmax, rov, base)

	if ts.Time == 0 || te.Time == 0 || TimeDiff(ts, te) > 0.0 {
		return 0
	}

	if strings.Contains(path, "%S") || strings.Contains(path, "%M") || strings.Contains(path, "%t") {
		tint = 900.0
	} else if strings.Contains(path, "%h") || strings.Contains(path, "%H") {
		tint = 3600.0
	}

	tow = Time2GpsT(ts, &week)
	time = GpsT2Time(week, math.Floor(tow/tint)*tint)

	for ; TimeDiff(time, te) <= 0.0 && n < nmax; time = TimeAdd(time, tint) {
		RepPath(path, &rpath[n], time, rov, base)
		if n == 0 || strings.Compare(rpath[n], rpath[n-1]) != 0 {
			n++
		}

	}
	for i = 0; i < n; i++ {
		Trace(2, "reppaths: rpath=%s\n", rpath[i])
	}
	return n
}

/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
func GeoDist(rs, rr, e []float64) float64 {
	var i int

	if Norm(rs, 3) < RE_WGS84 {
		return -1.0
	}
	for i = 0; i < 3; i++ {
		e[i] = rs[i] - rr[i]
	}
	r := Norm(e, 3)
	for i = 0; i < 3; i++ {
		e[i] /= r
	}
	return r + OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT
}

/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
func SatAzel(pos, e, azel []float64) float64 {
	var az, el float64 = 0.0, PI / 2.0
	var enu [3]float64

	if pos[2] > -RE_WGS84 {
		Ecef2Enu(pos, e, enu[:])
		if Dot(enu[:], enu[:], 2) < 1e-12 {
			az = 0.0
		} else {
			az = math.Atan2(enu[0], enu[1])
		}
		if az < 0.0 {
			az += 2 * PI
		}
		el = math.Asin(enu[2])
	}
	if azel != nil {
		azel[0] = az
		azel[1] = el
	}
	return el
}

/* compute DOPs ----------------------------------------------------------------
* compute DOP (dilution of precision)
* args   : int    ns        I   number of satellites
*          double *azel     I   satellite azimuth/elevation angle (rad)
*          double elmin     I   elevation cutoff angle (rad)
*          double *dop      O   DOPs {GDOP,PDOP,HDOP,VDOP}
* return : none
* notes  : dop[0]-[3] return 0 in case of dop computation error
*-----------------------------------------------------------------------------*/
func DOPs(ns int, azel []float64, elmin float64, dop []float64) {
	var H [4 * MAXSAT]float64
	var Q [16]float64
	var cosel, sinel float64
	var i, n int

	for i = 0; i < 4; i++ {
		dop[i] = 0.0
	}
	for i, n = 0, 0; i < ns && i < MAXSAT; i++ {
		if azel[1+i*2] < elmin || azel[1+i*2] <= 0.0 {
			continue
		}
		cosel = math.Cos(azel[1+i*2])
		sinel = math.Sin(azel[1+i*2])
		H[4*n] = cosel * math.Sin(azel[i*2])
		H[1+4*n] = cosel * math.Cos(azel[i*2])
		H[2+4*n] = sinel
		H[3+4*n] = 1.0
		n++
	}
	if n < 4 {
		return
	}

	MatMul("NT", 4, 4, n, 1.0, H[:], H[:], 0.0, Q[:])
	if MatInv(Q[:], 4) == 0 {
		dop[0] = SQRT(Q[0] + Q[5] + Q[10] + Q[15]) /* GDOP */
		dop[1] = SQRT(Q[0] + Q[5] + Q[10])         /* PDOP */
		dop[2] = SQRT(Q[0] + Q[5])                 /* HDOP */
		dop[3] = SQRT(Q[10])                       /* VDOP */
	}
}

/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
func IonModel(t Gtime, ion, pos, azel []float64) float64 {
	var ion_default []float64 = []float64{ /* 2004/1/1 */
		0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06,
		0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07,
	}
	var tt, f, psi, phi, lam, amp, per, x float64
	var week int

	if pos[2] < (-1e3) || azel[1] <= 0 {
		return 0.0
	}
	if Norm(ion, 8) <= 0.0 {
		ion = ion_default
	}

	/* earth centered angle (semi-circle) */
	psi = 0.0137/(azel[1]/PI+0.11) - 0.022

	/* subionospheric latitude/longitude (semi-circle) */
	phi = pos[0]/PI + psi*math.Cos(azel[0])
	if phi > 0.416 {
		phi = 0.416
	} else if phi < (-0.416) {
		phi = -0.416
	}
	lam = pos[1]/PI + psi*math.Sin(azel[0])/math.Cos(phi*PI)

	/* geomagnetic latitude (semi-circle) */
	phi += 0.064 * math.Cos((lam-1.617)*PI)

	/* local time (s) */
	tt = 43200.0*lam + Time2GpsT(t, &week)
	tt -= math.Floor(tt/86400.0) * 86400.0 /* 0<=tt<86400 */

	/* slant factor */
	f = 1.0 + 16.0*math.Pow(0.53-azel[1]/PI, 3.0)

	/* ionospheric delay */
	amp = ion[0] + phi*(ion[1]+phi*(ion[2]+phi*ion[3]))
	per = ion[4] + phi*(ion[5]+phi*(ion[6]+phi*ion[7]))
	if amp < 0.0 {
		amp = 0.0
	}
	if per < 72000.0 {
		per = 72000.0
	}
	x = 2.0 * PI * (tt - 50400.0) / per
	if math.Abs(x) < 1.57 {
		return CLIGHT * f * (5e-9 + amp*(1.0+x*x*(-0.5+x*x/24.0)))
	} else {

		return CLIGHT * f * (5e-9)
	}
}

/* ionosphere mapping function -------------------------------------------------
* compute ionospheric delay mapping function by single layer model
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric mapping function
*-----------------------------------------------------------------------------*/
func IonMapf(pos, azel []float64) float64 {
	if pos[2] >= HION {
		return 1.0
	}
	return 1.0 / math.Cos(math.Asin((RE_WGS84+pos[2])/(RE_WGS84+HION)*math.Sin(PI/2.0-azel[1])))
}

/* ionospheric pierce point position -------------------------------------------
* compute ionospheric pierce point (ipp) position and slant factor
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double re        I   earth radius (km)
*          double hion      I   altitude of ionosphere (km)
*          double *posp     O   pierce point position {lat,lon,h} (rad,m)
* return : slant factor
* notes  : see ref [2], only valid on the earth surface
*          fixing bug on ref [2] A.4.4.10.1 A-22,23
*-----------------------------------------------------------------------------*/
func IonPPP(pos, azel []float64, re, hion float64, posp []float64) float64 {
	var cosaz, rp, ap, sinap, tanap float64

	rp = re / (re + hion) * math.Cos(azel[1])
	ap = PI/2.0 - azel[1] - math.Asin(rp)
	sinap = math.Sin(ap)
	tanap = math.Tan(ap)
	cosaz = math.Cos(azel[0])
	posp[0] = math.Asin(math.Sin(pos[0])*math.Cos(ap) + math.Cos(pos[0])*sinap*cosaz)

	if (pos[0] > 70.0*D2R && tanap*cosaz > math.Tan(PI/2.0-pos[0])) || (pos[0] < -70.0*D2R && -tanap*cosaz > math.Tan(PI/2.0+pos[0])) {
		posp[1] = pos[1] + PI - math.Asin(sinap*math.Sin(azel[0])/math.Cos(posp[0]))
	} else {
		posp[1] = pos[1] + math.Asin(sinap*math.Sin(azel[0])/math.Cos(posp[0]))
	}
	return 1.0 / math.Sqrt(1.0-rp*rp)
}

/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
func TropModel(time Gtime, pos, azel []float64, humi float64) float64 {
	var temp0 float64 = 15.0 /* temparature at sea level */
	var hgt, pres, temp, e, z, trph, trpw float64

	if pos[2] < -100.0 || 1e4 < pos[2] || azel[1] <= 0 {
		return 0.0
	}

	/* standard atmosphere */
	if pos[2] < 0.0 {
		hgt = 0.0
	} else {
		hgt = pos[2]
	}

	pres = 1013.25 * math.Pow(1.0-2.2557e-5*hgt, 5.2568)
	temp = temp0 - 6.5e-3*hgt + 273.16
	e = 6.108 * humi * math.Exp((17.15*temp-4684.0)/(temp-38.45))

	/* saastamoninen model */
	z = PI/2.0 - azel[1]
	trph = 0.0022768 * pres / (1.0 - 0.00266*math.Cos(2.0*pos[0]) - 0.00028*hgt/1e3) / math.Cos(z)
	trpw = 0.002277 * (1255.0/temp + 0.05) * e / math.Cos(z)
	return trph + trpw
}
func interpc(coef []float64, lat float64) float64 {
	i := int(lat / 15.0)
	if i < 1 {
		return coef[0]
	} else if i > 4 {
		return coef[4]
	}
	return coef[i-1]*(1.0-lat/15.0+float64(i)) + coef[i]*(lat/15.0-float64(i))
}
func mapf(el, a, b, c float64) float64 {
	sinel := math.Sin(el)
	return (1.0 + a/(1.0+b/(1.0+c))) / (sinel + (a / (sinel + b/(sinel+c))))
}
func nmf(time Gtime, pos, azel []float64, mapfw *float64) float64 {
	/* ref [5] table 3 */
	/* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
	var coef [][5]float64 = [][5]float64{
		{1.2769934e-3, 1.2683230e-3, 1.2465397e-3, 1.2196049e-3, 1.2045996e-3},
		{2.9153695e-3, 2.9152299e-3, 2.9288445e-3, 2.9022565e-3, 2.9024912e-3},
		{62.610505e-3, 62.837393e-3, 63.721774e-3, 63.824265e-3, 64.258455e-3},

		{0.0000000e-0, 1.2709626e-5, 2.6523662e-5, 3.4000452e-5, 4.1202191e-5},
		{0.0000000e-0, 2.1414979e-5, 3.0160779e-5, 7.2562722e-5, 11.723375e-5},
		{0.0000000e-0, 9.0128400e-5, 4.3497037e-5, 84.795348e-5, 170.37206e-5},

		{5.8021897e-4, 5.6794847e-4, 5.8118019e-4, 5.9727542e-4, 6.1641693e-4},
		{1.4275268e-3, 1.5138625e-3, 1.4572752e-3, 1.5007428e-3, 1.7599082e-3},
		{4.3472961e-2, 4.6729510e-2, 4.3908931e-2, 4.4626982e-2, 5.4736038e-2},
	}
	var aht []float64 = []float64{2.53e-5, 5.49e-3, 1.14e-3} /* height correction */

	var ah, aw [3]float64
	el := azel[1]
	lat := pos[0] * R2D
	hgt := pos[2]
	var i int

	if el <= 0.0 {
		if mapfw != nil {
			*mapfw = 0.0
		}
		return 0.0
	}
	/* year from doy 28, added half a year for southern latitudes */
	var lat2 float64
	if lat < 0.0 {
		lat2 = 0.5
	} else {
		lat2 = 0.0
	}

	y := (Time2DayOfYeay(time)-28.0)/365.25 + lat2

	cosy := math.Cos(2.0 * PI * y)
	lat = math.Abs(lat)

	for i = 0; i < 3; i++ {
		ah[i] = interpc(coef[i][:], lat) - interpc(coef[i+3][:], lat)*cosy
		aw[i] = interpc(coef[i+6][:], lat)
	}
	/* ellipsoidal height is used instead of height above sea level */
	dm := (1.0/math.Sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1e3

	if mapfw != nil {
		*mapfw = mapf(el, aw[0], aw[1], aw[2])
	}

	return mapf(el, ah[0], ah[1], ah[2]) + dm
}

/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF
* args   : gtime_t t        I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [5] (NMF) and [9] (GMF)
*          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
func TropMapFunc(time Gtime, pos, azel []float64, mapfw *float64) float64 {
	// #ifdef IERS_MODEL
	// const double ep[]={2000,1,1,12,0,0};
	// double mjd,lat,lon,hgt,zd,gmfh,gmfw;
	// #endif
	// trace(4,"tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n",
	// pos[0]*R2D,pos[1]*R2D,pos[2],azel[0]*R2D,azel[1]*R2D);

	// if (pos[2]<-1000.0||pos[2]>20000.0) {
	// if (mapfw) *mapfw=0.0;
	// return 0.0;
	// }
	// #ifdef IERS_MODEL
	// mjd=51544.5+(timediff(time,epoch2time(ep)))/86400.0;
	// lat=pos[0];
	// lon=pos[1];
	// hgt=pos[2]-geoidh(pos); /* height in m (mean sea level) */
	// zd =PI/2.0-azel[1];

	// /* call GMF */
	// gmf_(&mjd,&lat,&lon,&hgt,&zd,&gmfh,&gmfw);

	// if (mapfw) *mapfw=gmfw;
	// return gmfh;
	// #else
	Trace(4, "tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n", pos[0]*R2D, pos[1]*R2D, pos[2], azel[0]*R2D, azel[1]*R2D)

	if pos[2] < -1000.0 || pos[2] > 20000.0 {
		if mapfw != nil {
			*mapfw = 0.0
		}
		return 0.0
	}
	return nmf(time, pos, azel, mapfw) /* NMF */

}

/* interpolate antenna phase center variation --------------------------------*/
func InterPVar(ang float64, vari []float64) float64 {
	a := ang / 5.0 /* ang=0-90 */
	i := int(a)
	if i < 0 {
		return vari[0]
	} else if i >= 18 {
		return vari[18]
	}

	return vari[i]*(1.0-a+float64(i)) + vari[i+1]*(a-float64(i))
}

/* receiver antenna model ------------------------------------------------------
* compute antenna offset by antenna phase center parameters
* args   : pcv_t *pcv       I   antenna phase center parameters
*          double *del      I   antenna delta {e,n,u} (m)
*          double *azel     I   azimuth/elevation for receiver {az,el} (rad)
*          int     opt      I   option (0:only offset,1:offset+pcv)
*          double *dant     O   range offsets for each frequency (m)
* return : none
* notes  : current version does not support azimuth dependent terms
*-----------------------------------------------------------------------------*/
func AntModel(pcv *Pcv, del, azel []float64, opt int, dant []float64) {
	var e, off [3]float64
	cosel := math.Cos(azel[1])
	var i, j int

	Trace(4, "antmodel: azel=%6.1f %4.1f opt=%d\n", azel[0]*R2D, azel[1]*R2D, opt)

	e[0] = math.Sin(azel[0]) * cosel
	e[1] = math.Cos(azel[0]) * cosel
	e[2] = math.Sin(azel[1])

	for i = 0; i < NFREQ; i++ {
		for j = 0; j < 3; j++ {
			off[j] = pcv.Offset[i][j] + del[j]

		}
		var intvar float64 = 0.0
		if opt > 0 {
			intvar = InterPVar(90.0-azel[1]*R2D, pcv.Variation[i][:])
		}
		dant[i] = -Dot(off[:], e[:], 3) + intvar
	}
	Trace(5, "antmodel: dant=%6.3f %6.3f\n", dant[0], dant[1])
}

/* satellite antenna model ------------------------------------------------------
* compute satellite antenna phase center parameters
* args   : pcv_t *pcv       I   antenna phase center parameters
*          double nadir     I   nadir angle for satellite (rad)
*          double *dant     O   range offsets for each frequency (m)
* return : none
*-----------------------------------------------------------------------------*/
func AntModel_s(pcv *Pcv, nadir float64, dant []float64) {

	Trace(4, "antmodel_s: nadir=%6.1f\n", nadir*R2D)

	for i := 0; i < NFREQ; i++ {
		dant[i] = InterPVar(nadir*R2D*5.0, pcv.Variation[i][:])
	}
	Trace(5, "antmodel_s: dant=%6.3f %6.3f\n", dant[0], dant[1])
}

/* sun and moon position in eci (ref [4] 5.1.1, 5.2.1) -----------------------*/
func sunmoonpos_eci(tut Gtime, rsun, rmoon []float64) {
	var ep2000 []float64 = []float64{2000, 1, 1, 12, 0, 0}
	var t, eps, Ms, ls, rs, lm, pm, rm, sine, cose, sinp, cosp, sinl, cosl float64
	var f [5]float64
	Trace(4, "sunmoonpos_eci: tut=%s\n", TimeStr(tut, 3))

	t = TimeDiff(tut, Epoch2Time(ep2000)) / 86400.0 / 36525.0

	/* astronomical arguments */
	ast_args(t, f[:])

	/* obliquity of the ecliptic */
	eps = 23.439291 - 0.0130042*t
	sine = math.Sin(eps * D2R)
	cose = math.Cos(eps * D2R)

	/* sun position in eci */
	if rsun != nil {
		Ms = 357.5277233 + 35999.05034*t
		ls = 280.460 + 36000.770*t + 1.914666471*math.Sin(Ms*D2R) + 0.019994643*math.Sin(2.0*Ms*D2R)
		rs = AU * (1.000140612 - 0.016708617*math.Cos(Ms*D2R) - 0.000139589*math.Cos(2.0*Ms*D2R))
		sinl = math.Sin(ls * D2R)
		cosl = math.Cos(ls * D2R)
		rsun[0] = rs * cosl
		rsun[1] = rs * cose * sinl
		rsun[2] = rs * sine * sinl

		Trace(2, "rsun =%.3f %.3f %.3f\n", rsun[0], rsun[1], rsun[2])
	}
	/* moon position in eci */
	if rmoon != nil {
		lm = 218.32 + 481267.883*t + 6.29*math.Sin(f[0]) - 1.27*math.Sin(f[0]-2.0*f[3]) +
			0.66*math.Sin(2.0*f[3]) + 0.21*math.Sin(2.0*f[0]) - 0.19*math.Sin(f[1]) - 0.11*math.Sin(2.0*f[2])
		pm = 5.13*math.Sin(f[2]) + 0.28*math.Sin(f[0]+f[2]) - 0.28*math.Sin(f[2]-f[0]) -
			0.17*math.Sin(f[2]-2.0*f[3])
		rm = RE_WGS84 / math.Sin((0.9508+0.0518*math.Cos(f[0])+0.0095*math.Cos(f[0]-2.0*f[3])+
			0.0078*math.Cos(2.0*f[3])+0.0028*math.Cos(2.0*f[0]))*D2R)
		sinl = math.Sin(lm * D2R)
		cosl = math.Cos(lm * D2R)
		sinp = math.Sin(pm * D2R)
		cosp = math.Cos(pm * D2R)
		rmoon[0] = rm * cosp * cosl
		rmoon[1] = rm * (cose*cosp*sinl - sine*sinp)
		rmoon[2] = rm * (sine*cosp*sinl + cose*sinp)

		Trace(2, "rmoon=%.3f %.3f %.3f\n", rmoon[0], rmoon[1], rmoon[2])
	}
}

/* sun and moon position -------------------------------------------------------
* get sun and moon position in ecef
* args   : gtime_t tut      I   time in ut1
*          double *erpv     I   erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *rsun     IO  sun position in ecef  (m) (NULL: not output)
*          double *rmoon    IO  moon position in ecef (m) (NULL: not output)
*          double *gmst     O   gmst (rad)
* return : none
*-----------------------------------------------------------------------------*/
func SunMoonPos(tutc Gtime, erpv, rsun, rmoon []float64, gmst *float64) {
	var tut Gtime
	var rs, rm [3]float64
	var U [9]float64
	var gmst_ float64

	Trace(4, "sunmoonpos: tutc=%s\n", TimeStr(tutc, 3))

	tut = TimeAdd(tutc, erpv[2]) /* utc . ut1 */

	/* sun and moon position in eci */
	var rse, rme []float64
	if rsun != nil {
		rse = rs[:]
	} else {
		rse = nil
	}
	if rmoon != nil {
		rme = rm[:]
	} else {
		rme = nil
	}
	sunmoonpos_eci(tut, rse, rme)

	/* eci to ecef transformation matrix */
	Eci2Ecef(tutc, erpv, U[:], &gmst_)

	/* sun and moon postion in ecef */
	if rsun != nil {
		MatMul("NN", 3, 1, 3, 1.0, U[:], rs[:], 0.0, rsun[:])
	}
	if rmoon != nil {
		MatMul("NN", 3, 1, 3, 1.0, U[:], rm[:], 0.0, rmoon[:])
	}
	if gmst != nil {
		*gmst = gmst_
	}
}

/* uncompress file -------------------------------------------------------------
* uncompress (uncompress/unzip/uncompact hatanaka-compression/tar) file
* args   : char   *file     I   input file
*          char   *uncfile  O   uncompressed file
* return : status (-1:error,0:not compressed file,1:uncompress completed)
* note   : creates uncompressed file in tempolary directory
*          gzip, tar and crx2rnx commands have to be installed in commands path
*-----------------------------------------------------------------------------*/
func Rtk_Uncompress(file string, uncfile *string) int {
	var stat = 0
	var cmd, tmpfile, buff, fname, dir string
	Trace(4, "rtk_uncompress: file=%s\n", file)
	var index int
	tmpfile = file
	if index = strings.LastIndex(tmpfile, "."); index < 0 {
		return 0
	}
	/* uncompress by gzip */
	if strings.EqualFold(tmpfile[index:], ".z") || strings.EqualFold(tmpfile[index:], ".gz") ||
		strings.EqualFold(tmpfile[index:], ".zip") {
		*uncfile = tmpfile[:index]
		cmd = fmt.Sprintf("gzip -f -d -c \"%s\" > \"%s\"", tmpfile, tmpfile[:index])

		if ExecCmd(cmd) > 0 {

			os.Remove(*uncfile)
			return -1
		}
		tmpfile = tmpfile[:index]
		stat = 1
	}
	/* extract tar file */
	if strings.EqualFold(tmpfile[index:], ".tar") {
		*uncfile = tmpfile[:index]
		buff = tmpfile

		if runtime.GOOS == "windows" {
			if index = strings.Index(buff, "\\"); index > 0 {
				dir = buff[:index]
				fname = buff[index+1:]
			}
			cmd = fmt.Sprintf("set PATH=%%CD%%;%%PATH%% & cd /D \"%s\" & tar -xf \"%s\"", dir, fname)
		} else {
			if index = strings.Index(tmpfile, "/"); index > 0 {
				dir = buff[:index]
			}
			cmd = fmt.Sprintf("tar -C \"%s\" -xf \"%s\"", dir, tmpfile)
		}
		if ExecCmd(cmd) > 0 {
			if stat > 0 {
				os.Remove(tmpfile)
			}
			return -1
		}
		if stat > 0 {
			os.Remove(tmpfile)
		}
		stat = 1
	} else if index > 0 {
		buff = tmpfile[index:]
		if (len(buff) > 3 && (buff[3] == 'd' || buff[3] == 'D')) ||
			strings.EqualFold(buff, ".crx") {
			/* extract hatanaka-compressed file by cnx2rnx */
			tmp := []byte(tmpfile)
			if buff[3] == 'D' {
				tmp[3] = 'O'
			} else {
				tmp[3] = 'o'
			}
			*uncfile = string(tmp)

			cmd = fmt.Sprintf("crx2rnx < \"%s\" > \"%s\"", tmpfile, *uncfile)

			if ExecCmd(cmd) > 0 {
				os.Remove(*uncfile)
				if stat > 0 {
					os.Remove(tmpfile)
				}
				return -1
			}
			if stat > 0 {
				os.Remove(tmpfile)
			}
			stat = 1
		}
	}

	Trace(5, "rtk_uncompress: stat=%d\n", stat)
	return stat
}

// pointer func for showmsg() defined by app
var ShowMsg_Ptr func(format string, v ...interface{}) int = showmsg

// default showmsg() func
func showmsg(format string, v ...interface{}) int {
	return 0
}
