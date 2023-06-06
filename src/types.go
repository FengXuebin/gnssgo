package gnssgo

import (
	"sync"
)

// 定义Gnss的基本数据类型

// some macro definition
const (
	RRCENA int = 0

	VER_GNSSGO               = "0.0.1" /* library version */
	PATCH_LEVEL              = "001"   /* patch level */
	COPYRIGHT_GNSSGO         = "Copyright (C) 2022-2023 Feng Xuebin\nAll rights reserved."
	PI               float64 = 3.1415926535897932    /* pi */
	D2R                      = (PI / 180.0)          /* deg to rad */
	R2D                      = (180.0 / PI)          /* rad to deg */
	CLIGHT           float64 = 299792458.0           /* speed of light (m/s) */
	SC2RAD           float64 = 3.1415926535898       /* semi-circle to radian (IS-GPS) */
	AU               float64 = 149597870691.0        /* 1 AU (m) */
	AS2R                     = (D2R / 3600.0)        /* arc sec to radian */
	OMGE             float64 = 7.2921151467e-5       /* earth angular velocity (IS-GPS) (rad/s) */
	RE_WGS84         float64 = 6378137.0             /* earth semimajor axis (WGS84) (m) */
	FE_WGS84         float64 = (1.0 / 298.257223563) /* earth flattening (WGS84) */
	HION                     = 350000.0              /* ionosphere height (m) */
	MAXFREQ                  = 7 /* max NFREQ */)

const (
	FREQ1        float64 = 1.57542e9                   /* L1/E1/B1C  frequency (Hz) */
	FREQ2        float64 = 1.22760e9                   /* L2         frequency (Hz) */
	FREQ5        float64 = 1.17645e9                   /* L5/E5a/B2a frequency (Hz) */
	FREQ6        float64 = 1.27875e9                   /* E6/L6  frequency (Hz) */
	FREQ7        float64 = 1.20714e9                   /* E5b    frequency (Hz) */
	FREQ8        float64 = 1.191795e9                  /* E5a+b  frequency (Hz) */
	FREQ9        float64 = 2.492028e9                  /* S      frequency (Hz) */
	FREQ1_GLO    float64 = 1.60200e9                   /* GLONASS G1 base frequency (Hz) */
	DFRQ1_GLO    float64 = 0.56250e6                   /* GLONASS G1 bias frequency (Hz/n) */
	FREQ2_GLO    float64 = 1.24600e9                   /* GLONASS G2 base frequency (Hz) */
	DFRQ2_GLO    float64 = 0.43750e6                   /* GLONASS G2 bias frequency (Hz/n) */
	FREQ3_GLO    float64 = 1.202025e9                  /* GLONASS G3 frequency (Hz) */
	FREQ1a_GLO   float64 = 1.600995e9                  /* GLONASS G1a frequency (Hz) */
	FREQ2a_GLO   float64 = 1.248060e9                  /* GLONASS G2a frequency (Hz) */
	FREQ1_CMP    float64 = 1.561098e9                  /* BDS B1I     frequency (Hz) */
	FREQ2_CMP    float64 = 1.20714e9                   /* BDS B2I/B2b frequency (Hz) */
	FREQ3_CMP    float64 = 1.26852e9                   /* BDS B3      frequency (Hz) */
	EFACT_GPS            = 1.0                         /* error factor: GPS */
	EFACT_GLO            = 1.5                         /* error factor: GLONASS */
	EFACT_GAL            = 1.0                         /* error factor: Galileo */
	EFACT_QZS            = 1.0                         /* error factor: QZSS */
	EFACT_CMP            = 1.0                         /* error factor: BeiDou */
	EFACT_IRN            = 1.5                         /* error factor: IRNSS */
	EFACT_SBS            = 3.0                         /* error factor: SBAS */
	SYS_NONE             = 0x00                        /* navigation system: none */
	SYS_GPS              = 0x01                        /* navigation system: GPS */
	SYS_SBS              = 0x02                        /* navigation system: SBAS */
	SYS_GLO              = 0x04                        /* navigation system: GLONASS */
	SYS_GAL              = 0x08                        /* navigation system: Galileo */
	SYS_QZS              = 0x10                        /* navigation system: QZSS */
	SYS_CMP              = 0x20                        /* navigation system: BeiDou */
	SYS_IRN              = 0x40                        /* navigation system: IRNS */
	SYS_LEO              = 0x80                        /* navigation system: LEO */
	SYS_ALL              = 0xFF                        /* navigation system: all */
	TSYS_GPS             = 0                           /* time system: GPS time */
	TSYS_UTC             = 1                           /* time system: UTC */
	TSYS_GLO             = 2                           /* time system: GLONASS time */
	TSYS_GAL             = 3                           /* time system: Galileo time */
	TSYS_QZS             = 4                           /* time system: QZSS time */
	TSYS_CMP             = 5                           /* time system: BeiDou time */
	TSYS_IRN             = 6                           /* time system: IRNSS time */
	NFREQ                = 3                           /* number of carrier frequencies */
	NFREQGLO             = 2                           /* number of carrier frequencies of GLONASS */
	NEXOBS               = 0                           /* number of extended obs codes */
	SNR_UNIT             = 0.001                       /* SNR unit (dBHz) */
	MINPRNGPS            = 1                           /* min satellite PRN number of GPS */
	MAXPRNGPS            = 32                          /* max satellite PRN number of GPS */
	NSATGPS              = (MAXPRNGPS - MINPRNGPS + 1) /* number of GPS satellites */
	NSYSGPS              = 1
	NFREQ_NEXOBS         = 3 /* NFREQ + NEXOBS */
)

// #ifdef ENAGLO
const (
	MINPRNGLO = 1                           /* min satellite slot number of GLONASS */
	MAXPRNGLO = 27                          /* max satellite slot number of GLONASS */
	NSATGLO   = (MAXPRNGLO - MINPRNGLO + 1) /* number of GLONASS satellites */
	NSYSGLO   = 1
)

// const MINPRNGLO int = 0 /* min satellite slot number of GLONASS */
// const MAXPRNGLO int = 0 /* max satellite slot number of GLONASS */
// const NSATGLO int = 0   /* number of GLONASS satellites */
// const NSYSGLO int = 0

// #ifdef ENAGAL
const (
	MINPRNGAL = 1                           /* min satellite PRN number of Galileo */
	MAXPRNGAL = 36                          /* max satellite PRN number of Galileo */
	NSATGAL   = (MAXPRNGAL - MINPRNGAL + 1) /* number of Galileo satellites */
	NSYSGAL   = 1
)

// const MINPRNGAL int = 0 /* min satellite PRN number of Galileo */
// const MAXPRNGAL int = 0 /* max satellite PRN number of Galileo */
// const NSATGAL int = 0   /* number of Galileo satellites */
// const NSYSGAL int = 0

// #ifdef ENAQZS
const (
	MINPRNQZS   = 193                         /* min satellite PRN number of QZSS */
	MAXPRNQZS   = 202                         /* max satellite PRN number of QZSS */
	MINPRNQZS_S = 183                         /* min satellite PRN number of QZSS L1S */
	MAXPRNQZS_S = 191                         /* max satellite PRN number of QZSS L1S */
	NSATQZS     = (MAXPRNQZS - MINPRNQZS + 1) /* number of QZSS satellites */
	NSYSQZS     = 1
)

// const (
// 	MINPRNQZS   = 0 /* min satellite PRN number of QZSS */
// 	MAXPRNQZS   = 0 /* max satellite PRN number of QZSS */
// 	MINPRNQZS_S = 0 /* min satellite PRN number of QZSS L1S */
// 	MAXPRNQZS_S = 0 /* max satellite PRN number of QZSS L1S */
// 	NSATQZS     = 0 /* number of QZSS satellites */
// 	NSYSQZS     = 0
// )

// #ifdef ENACMP
const (
	MINPRNCMP = 1                           /* min satellite sat number of BeiDou */
	MAXPRNCMP = 63                          /* max satellite sat number of BeiDou */
	NSATCMP   = (MAXPRNCMP - MINPRNCMP + 1) /* number of BeiDou satellites */
	NSYSCMP   = 1
)

// const MINPRNCMP int = 0 /* min satellite sat number of BeiDou */
// const MAXPRNCMP int = 0 /* max satellite sat number of BeiDou */
// const NSATCMP int = 0   /* number of BeiDou satellites */
// const NSYSCMP int = 0

const (
	MINPRNIRN = 1                           /* min satellite sat number of IRNSS */
	MAXPRNIRN = 14                          /* max satellite sat number of IRNSS */
	NSATIRN   = (MAXPRNIRN - MINPRNIRN + 1) /* number of IRNSS satellites */
	NSYSIRN   = 1
)

// const MINPRNIRN int = 0 /* min satellite sat number of IRNSS */
// const MAXPRNIRN int = 0 /* max satellite sat number of IRNSS */
// const NSATIRN int = 0   /* number of IRNSS satellites */
// const NSYSIRN int = 0

const (
	MINPRNLEO = 1                           /* min satellite sat number of LEO */
	MAXPRNLEO = 10                          /* max satellite sat number of LEO */
	NSATLEO   = (MAXPRNLEO - MINPRNLEO + 1) /* number of LEO satellites */
	NSYSLEO   = 1
)

// const MINPRNLEO int = 0 /* min satellite sat number of LEO */
// const MAXPRNLEO int = 0 /* max satellite sat number of LEO */
// const NSATLEO int = 0   /* number of LEO satellites */
// const NSYSLEO int = 0

const (
	NSYS              = 7                         /* NSYSGPS + NSYSGLO + NSYSGAL + NSYSQZS + NSYSCMP + NSYSIRN + NSYSLEO */ /* number of systems */
	MINPRNSBS         = 120                       /* min satellite PRN number of SBAS */
	MAXPRNSBS         = 158                       /* max satellite PRN number of SBAS */
	NSATSBS           = MAXPRNSBS - MINPRNSBS + 1 /* number of SBAS satellites */
	MAXSAT            = 231                       /* NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATSBS + NSATLEO */
	MAXSTA            = 255                       /* max satellite number (1 to MAXSAT) */
	MAXOBS            = 96                        /* max number of obs in an epoch */
	MAXRCV            = 64                        /* max receiver number (1 to MAXRCV) */
	MAXOBSTYPE        = 64                        /* max number of obs type in RINEX */
	DTTOL             = 0.025                     /* tolerance of time difference (s) */
	MAXDTOE           = 7200.0                    /* max time difference to GPS Toe (s) */
	MAXDTOE_QZS       = 7200.0                    /* max time difference to QZSS Toe (s) */
	MAXDTOE_GAL       = 14400.0                   /* max time difference to Galileo Toe (s) */
	MAXDTOE_CMP       = 21600.0                   /* max time difference to BeiDou Toe (s) */
	MAXDTOE_GLO       = 1800.0                    /* max time difference to GLONASS Toe (s) */
	MAXDTOE_IRN       = 7200.0                    /* max time difference to IRNSS Toe (s) */
	MAXDTOE_SBS       = 360.0                     /* max time difference to SBAS Toe (s) */
	MAXDTOE_S         = 86400.0                   /* max time difference to ephem toe (s) for other */
	MAXGDOP           = 300.0                     /* max GDOP */
	INT_SWAP_TRAC     = 86400.0                   /* swap interval of trace file (s) */
	INT_SWAP_STAT     = 86400.0                   /* swap interval of solution status file (s) */
	MAXEXFILE         = 1024                      /* max number of expanded files */
	MAXSBSAGEF        = 30.0                      /* max age of SBAS fast correction (s) */
	MAXSBSAGEL        = 1800.0                    /* max age of SBAS long term corr (s) */
	MAXSBSURA         = 8                         /* max URA of SBAS satellite */
	MAXBAND           = 10                        /* max SBAS band of IGP */
	MAXNIGP           = 201                       /* max number of IGP in SBAS band */
	MAXNGEO           = 4                         /* max number of GEO satellites */
	MAXCOMMENT        = 100                       /* max number of RINEX comments */
	MAXSTRPATH        = 1024                      /* max length of stream path */
	MAXSTRMSG         = 1024                      /* max length of stream message */
	MAXSTRRTK         = 8                         /* max number of stream in RTK server */
	MAXSBSMSG         = 32                        /* max number of SBAS msg in RTK server */
	MAXSOLMSG         = 8191                      /* max length of solution message */
	MAXRAWLEN         = 16384                     /* max length of receiver raw message */
	MAXERRMSG         = 4096                      /* max length of error/warning message */
	MAXANT            = 64                        /* max length of station name/antenna type */
	MAXSOLBUF         = 256                       /* max number of solution buffer */
	MAXOBSBUF         = 128                       /* max number of observation data buffer */
	MAXNRPOS          = 16                        /* max number of reference positions */
	MAXLEAPS          = 64                        /* max number of leap seconds table */
	MAXGISLAYER       = 32                        /* max number of GIS data layers */
	MAXRCVCMD         = 4096                      /* max length of receiver commands */
	RNX2VER           = "2.10"                    /* RINEX ver.2 default output version */
	RNX3VER           = "3.00"                    /* RINEX ver.3 default output version */
	OBSTYPE_PR        = 0x01                      /* observation type: pseudorange */
	OBSTYPE_CP        = 0x02                      /* observation type: carrier-phase */
	OBSTYPE_DOP       = 0x04                      /* observation type: doppler-freq */
	OBSTYPE_SNR       = 0x08                      /* observation type: SNR */
	OBSTYPE_ALL       = 0xFF                      /* observation type: all */
	FREQTYPE_L1       = 0x01                      /* frequency type: L1/E1/B1 */
	FREQTYPE_L2       = 0x02                      /* frequency type: L2/E5b/B2 */
	FREQTYPE_L3       = 0x04                      /* frequency type: L5/E5a/L3 */
	FREQTYPE_L4       = 0x08                      /* frequency type: L6/E6/B3 */
	FREQTYPE_L5       = 0x10                      /* frequency type: E5ab */
	FREQTYPE_ALL      = 0xFF                      /* frequency type: all */
	CODE_NONE         = 0                         /* obs code: none or unknown */
	CODE_L1C          = 1                         /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
	CODE_L1P          = 2                         /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
	CODE_L1W          = 3                         /* obs code: L1 Z-track (GPS) */
	CODE_L1Y          = 4                         /* obs code: L1Y        (GPS) */
	CODE_L1M          = 5                         /* obs code: L1M        (GPS) */
	CODE_L1N          = 6                         /* obs code: L1codeless,B1codeless (GPS,BDS) */
	CODE_L1S          = 7                         /* obs code: L1C(D)     (GPS,QZS) */
	CODE_L1L          = 8                         /* obs code: L1C(P)     (GPS,QZS) */
	CODE_L1E          = 9                         /* (not used) */
	CODE_L1A          = 10                        /* obs code: E1A,B1A    (GAL,BDS) */
	CODE_L1B          = 11                        /* obs code: E1B        (GAL) */
	CODE_L1X          = 12                        /* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
	CODE_L1Z          = 13                        /* obs code: E1A+B+C,L1S (GAL,QZS) */
	CODE_L2C          = 14                        /* obs code: L2C/A,G1C/A (GPS,GLO) */
	CODE_L2D          = 15                        /* obs code: L2 L1C/A-(P2-P1) (GPS) */
	CODE_L2S          = 16                        /* obs code: L2C(M)     (GPS,QZS) */
	CODE_L2L          = 17                        /* obs code: L2C(L)     (GPS,QZS) */
	CODE_L2X          = 18                        /* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
	CODE_L2P          = 19                        /* obs code: L2P,G2P    (GPS,GLO) */
	CODE_L2W          = 20                        /* obs code: L2 Z-track (GPS) */
	CODE_L2Y          = 21                        /* obs code: L2Y        (GPS) */
	CODE_L2M          = 22                        /* obs code: L2M        (GPS) */
	CODE_L2N          = 23                        /* obs code: L2codeless (GPS) */
	CODE_L5I          = 24                        /* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
	CODE_L5Q          = 25                        /* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
	CODE_L5X          = 26                        /* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
	CODE_L7I          = 27                        /* obs code: E5bI,B2bI  (GAL,BDS) */
	CODE_L7Q          = 28                        /* obs code: E5bQ,B2bQ  (GAL,BDS) */
	CODE_L7X          = 29                        /* obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
	CODE_L6A          = 30                        /* obs code: E6A,B3A    (GAL,BDS) */
	CODE_L6B          = 31                        /* obs code: E6B        (GAL) */
	CODE_L6C          = 32                        /* obs code: E6C        (GAL) */
	CODE_L6X          = 33                        /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
	CODE_L6Z          = 34                        /* obs code: E6A+B+C,L6D+E (GAL,QZS) */
	CODE_L6S          = 35                        /* obs code: L6S        (QZS) */
	CODE_L6L          = 36                        /* obs code: L6L        (QZS) */
	CODE_L8I          = 37                        /* obs code: E5abI      (GAL) */
	CODE_L8Q          = 38                        /* obs code: E5abQ      (GAL) */
	CODE_L8X          = 39                        /* obs code: E5abI+Q,B2abD+P (GAL,BDS) */
	CODE_L2I          = 40                        /* obs code: B1_2I      (BDS) */
	CODE_L2Q          = 41                        /* obs code: B1_2Q      (BDS) */
	CODE_L6I          = 42                        /* obs code: B3I        (BDS) */
	CODE_L6Q          = 43                        /* obs code: B3Q        (BDS) */
	CODE_L3I          = 44                        /* obs code: G3I        (GLO) */
	CODE_L3Q          = 45                        /* obs code: G3Q        (GLO) */
	CODE_L3X          = 46                        /* obs code: G3I+Q      (GLO) */
	CODE_L1I          = 47                        /* obs code: B1I        (BDS) (obsolute) */
	CODE_L1Q          = 48                        /* obs code: B1Q        (BDS) (obsolute) */
	CODE_L5A          = 49                        /* obs code: L5A SPS    (IRN) */
	CODE_L5B          = 50                        /* obs code: L5B RS(D)  (IRN) */
	CODE_L5C          = 51                        /* obs code: L5C RS(P)  (IRN) */
	CODE_L9A          = 52                        /* obs code: SA SPS     (IRN) */
	CODE_L9B          = 53                        /* obs code: SB RS(D)   (IRN) */
	CODE_L9C          = 54                        /* obs code: SC RS(P)   (IRN) */
	CODE_L9X          = 55                        /* obs code: SB+C       (IRN) */
	CODE_L1D          = 56                        /* obs code: B1D        (BDS) */
	CODE_L5D          = 57                        /* obs code: L5D(L5S),B2aD (QZS,BDS) */
	CODE_L5P          = 58                        /* obs code: L5P(L5S),B2aP (QZS,BDS) */
	CODE_L5Z          = 59                        /* obs code: L5D+P(L5S) (QZS) */
	CODE_L6E          = 60                        /* obs code: L6E        (QZS) */
	CODE_L7D          = 61                        /* obs code: B2bD       (BDS) */
	CODE_L7P          = 62                        /* obs code: B2bP       (BDS) */
	CODE_L7Z          = 63                        /* obs code: B2bD+P     (BDS) */
	CODE_L8D          = 64                        /* obs code: B2abD      (BDS) */
	CODE_L8P          = 65                        /* obs code: B2abP      (BDS) */
	CODE_L4A          = 66                        /* obs code: G1aL1OCd   (GLO) */
	CODE_L4B          = 67                        /* obs code: G1aL1OCd   (GLO) */
	CODE_L4X          = 68                        /* obs code: G1al1OCd+p (GLO) */
	MAXCODE           = 68                        /* max number of obs code */
	PMODE_SINGLE      = 0                         /* positioning mode: single */
	PMODE_DGPS        = 1                         /* positioning mode: DGPS/DGNSS */
	PMODE_KINEMA      = 2                         /* positioning mode: kinematic */
	PMODE_STATIC      = 3                         /* positioning mode: static */
	PMODE_MOVEB       = 4                         /* positioning mode: moving-base */
	PMODE_FIXED       = 5                         /* positioning mode: fixed */
	PMODE_PPP_KINEMA  = 6                         /* positioning mode: PPP-kinemaric */
	PMODE_PPP_STATIC  = 7                         /* positioning mode: PPP-static */
	PMODE_PPP_FIXED   = 8                         /* positioning mode: PPP-fixed */
	SOLF_LLH          = 0                         /* solution format: lat/lon/height */
	SOLF_XYZ          = 1                         /* solution format: x/y/z-ecef */
	SOLF_ENU          = 2                         /* solution format: e/n/u-baseline */
	SOLF_NMEA         = 3                         /* solution format: NMEA-183 */
	SOLF_STAT         = 4                         /* solution format: solution status */
	SOLF_GSIF         = 5                         /* solution format: GSI F1/F2 */
	SOLQ_NONE         = 0                         /* solution status: no solution */
	SOLQ_FIX          = 1                         /* solution status: fix */
	SOLQ_FLOAT        = 2                         /* solution status: float */
	SOLQ_SBAS         = 3                         /* solution status: SBAS */
	SOLQ_DGPS         = 4                         /* solution status: DGPS/DGNSS */
	SOLQ_SINGLE       = 5                         /* solution status: single */
	SOLQ_PPP          = 6                         /* solution status: PPP */
	SOLQ_DR           = 7                         /* solution status: dead reconing */
	MAXSOLQ           = 7                         /* max number of solution status */
	TIMES_GPST        = 0                         /* time system: gps time */
	TIMES_UTC         = 1                         /* time system: utc */
	TIMES_JST         = 2                         /* time system: jst */
	IONOOPT_OFF       = 0                         /* ionosphere option: correction off */
	IONOOPT_BRDC      = 1                         /* ionosphere option: broadcast model */
	IONOOPT_SBAS      = 2                         /* ionosphere option: SBAS model */
	IONOOPT_IFLC      = 3                         /* ionosphere option: L1/L2 iono-free LC */
	IONOOPT_EST       = 4                         /* ionosphere option: estimation */
	IONOOPT_TEC       = 5                         /* ionosphere option: IONEX TEC model */
	IONOOPT_QZS       = 6                         /* ionosphere option: QZSS broadcast model */
	IONOOPT_STEC      = 8                         /* ionosphere option: SLANT TEC model */
	TROPOPT_OFF       = 0                         /* troposphere option: correction off */
	TROPOPT_SAAS      = 1                         /* troposphere option: Saastamoinen model */
	TROPOPT_SBAS      = 2                         /* troposphere option: SBAS model */
	TROPOPT_EST       = 3                         /* troposphere option: ZTD estimation */
	TROPOPT_ESTG      = 4                         /* troposphere option: ZTD+grad estimation */
	TROPOPT_ZTD       = 5                         /* troposphere option: ZTD correction */
	EPHOPT_BRDC       = 0                         /* ephemeris option: broadcast ephemeris */
	EPHOPT_PREC       = 1                         /* ephemeris option: precise ephemeris */
	EPHOPT_SBAS       = 2                         /* ephemeris option: broadcast + SBAS */
	EPHOPT_SSRAPC     = 3                         /* ephemeris option: broadcast + SSR_APC */
	EPHOPT_SSRCOM     = 4                         /* ephemeris option: broadcast + SSR_COM */
	ARMODE_OFF        = 0                         /* AR mode: off */
	ARMODE_CONT       = 1                         /* AR mode: continuous */
	ARMODE_INST       = 2                         /* AR mode: instantaneous */
	ARMODE_FIXHOLD    = 3                         /* AR mode: fix and hold */
	ARMODE_WLNL       = 4                         /* AR mode: wide lane/narrow lane */
	ARMODE_TCAR       = 5                         /* AR mode: triple carrier ar */
	SBSOPT_LCORR      = 1                         /* SBAS option: long term correction */
	SBSOPT_FCORR      = 2                         /* SBAS option: fast correction */
	SBSOPT_ICORR      = 4                         /* SBAS option: ionosphere correction */
	SBSOPT_RANGE      = 8                         /* SBAS option: ranging */
	POSOPT_POS        = 0                         /* pos option: LLH/XYZ */
	POSOPT_SINGLE     = 1                         /* pos option: average of single pos */
	POSOPT_FILE       = 2                         /* pos option: read from pos file */
	POSOPT_RINEX      = 3                         /* pos option: rinex header pos */
	POSOPT_RTCM       = 4                         /* pos option: rtcm/raw station pos */
	STR_NONE          = 0                         /* stream type: none */
	STR_SERIAL        = 1                         /* stream type: serial */
	STR_FILE          = 2                         /* stream type: file */
	STR_TCPSVR        = 3                         /* stream type: TCP server */
	STR_TCPCLI        = 4                         /* stream type: TCP client */
	STR_NTRIPSVR      = 5                         /* stream type: NTRIP server */
	STR_NTRIPCLI      = 6                         /* stream type: NTRIP client */
	STR_FTP           = 7                         /* stream type: ftp */
	STR_HTTP          = 8                         /* stream type: http */
	STR_NTRIPCAS      = 9                         /* stream type: NTRIP caster */
	STR_UDPSVR        = 10                        /* stream type: UDP server */
	STR_UDPCLI        = 11                        /* stream type: UDP server */
	STR_MEMBUF        = 12                        /* stream type: memory buffer */
	STRFMT_RTCM2      = 0                         /* stream format: RTCM 2 */
	STRFMT_RTCM3      = 1                         /* stream format: RTCM 3 */
	STRFMT_OEM4       = 2                         /* stream format: NovAtel OEMV/4 */
	STRFMT_OEM3       = 3                         /* stream format: NovAtel OEM3 */
	STRFMT_UBX        = 4                         /* stream format: u-blox LEA-*T */
	STRFMT_SS2        = 5                         /* stream format: NovAtel Superstar II */
	STRFMT_CRES       = 6                         /* stream format: Hemisphere */
	STRFMT_STQ        = 7                         /* stream format: SkyTraq S1315F */
	STRFMT_JAVAD      = 8                         /* stream format: JAVAD GRIL/GREIS */
	STRFMT_NVS        = 9                         /* stream format: NVS NVC08C */
	STRFMT_BINEX      = 10                        /* stream format: BINEX */
	STRFMT_RT17       = 11                        /* stream format: Trimble RT17 */
	STRFMT_SEPT       = 12                        /* stream format: Septentrio */
	STRFMT_RINEX      = 13                        /* stream format: RINEX */
	STRFMT_SP3        = 14                        /* stream format: SP3 */
	STRFMT_RNXCLK     = 15                        /* stream format: RINEX CLK */
	STRFMT_SBAS       = 16                        /* stream format: SBAS messages */
	STRFMT_NMEA       = 17                        /* stream format: NMEA 0183 */
	MAXRCVFMT         = 12                        /* max number of receiver format */
	STR_MODE_R        = 0x1                       /* stream mode: read */
	STR_MODE_W        = 0x2                       /* stream mode: write */
	STR_MODE_RW       = 0x3                       /* stream mode: read/write */
	GEOID_EMBEDDED    = 0                         /* geoid model: embedded geoid */
	GEOID_EGM96_M150  = 1                         /* geoid model: EGM96 15x15" */
	GEOID_EGM2008_M25 = 2                         /* geoid model: EGM2008 2.5x2.5" */
	GEOID_EGM2008_M10 = 3                         /* geoid model: EGM2008 1.0x1.0" */
	GEOID_GSI2000_M15 = 4                         /* geoid model: GSI geoid 2000 1.0x1.5" */
	GEOID_RAF09       = 5                         /* geoid model: IGN RAF09 for France 1.5"x2" */
	COMMENTH          = "%"                       /* comment line indicator for solution */
	MSG_DISCONN       = "$_DISCONNECT\r\n"        /* disconnect message */
	FILEPATHSEP       = "/"
	DLOPT_FORCE       = 0x01                  /* download option: force download existing */
	DLOPT_KEEPCMP     = 0x02                  /* download option: keep compressed file */
	DLOPT_HOLDERR     = 0x04                  /* download option: hold on error file */
	DLOPT_HOLDLST     = 0x08                  /* download option: hold on listing file */
	LLI_SLIP          = 0x01                  /* LLI: cycle-slip */
	LLI_HALFC         = 0x02                  /* LLI: half-cycle not resovled */
	LLI_BOCTRK        = 0x04                  /* LLI: boc tracking of mboc signal */
	LLI_HALFA         = 0x40                  /* LLI: half-cycle added */
	LLI_HALFS         = 0x80                  /* LLI: half-cycle subtracted */
	P2_5              = 0.03125               /* 2^-5 */
	P2_6              = 0.015625              /* 2^-6 */
	P2_10             = 0.0009765625          /* 2^-10 */
	P2_11             = 4.882812500000000e-04 /* 2^-11 */
	P2_15             = 3.051757812500000e-05 /* 2^-15 */
	P2_17             = 7.629394531250000e-06 /* 2^-17 */
	P2_19             = 1.907348632812500e-06 /* 2^-19 */
	P2_20             = 9.536743164062500e-07 /* 2^-20 */
	P2_21             = 4.768371582031250e-07 /* 2^-21 */
	P2_23             = 1.192092895507810e-07 /* 2^-23 */
	P2_24             = 5.960464477539063e-08 /* 2^-24 */
	P2_27             = 7.450580596923828e-09 /* 2^-27 */
	P2_28             = 3.725290298461914e-09 /* 2^-28 */
	P2_29             = 1.862645149230957e-09 /* 2^-29 */
	P2_30             = 9.313225746154785e-10 /* 2^-30 */
	P2_31             = 4.656612873077393e-10 /* 2^-31 */
	P2_32             = 2.328306436538696e-10 /* 2^-32 */
	P2_33             = 1.164153218269348e-10 /* 2^-33 */
	P2_34             = 5.820766091346740e-11 /* 2^-34 */
	P2_35             = 2.910383045673370e-11 /* 2^-35 */
	P2_38             = 3.637978807091710e-12 /* 2^-38 */
	P2_39             = 1.818989403545856e-12 /* 2^-39 */
	P2_40             = 9.094947017729280e-13 /* 2^-40 */
	P2_41             = 4.547473508864641e-13 /* 2^-41 */
	P2_43             = 1.136868377216160e-13 /* 2^-43 */
	P2_46             = 1.421085471520200e-14 /* 2^-46 */
	P2_48             = 3.552713678800501e-15 /* 2^-48 */
	P2_50             = 8.881784197001252e-16 /* 2^-50 */
	P2_51             = 4.440892098500626e-16 /* 2^-51 */
	P2_55             = 2.775557561562891e-17 /* 2^-55 */
	P2_59             = 1.734723475976810e-18 /* 2^-59 */
	P2_66             = 1.355252715606880e-20 /* 2^-66 */
	P2_68             = 3.388131789017201e-21 /* 2^-68 */
)

type Gtime struct {
	Time uint64 /* time (s) expressed by standard time_t */

	Sec float64 /* fraction of second under 1 s */
}

type ObsD struct { /* observation data record */
	Time     Gtime                   /* receiver sampling time (GPST) */
	Sat, Rcv int                     /* satellite/receiver number */
	SNR      [NFREQ + NEXOBS]uint16  /* signal strength (0.001 dBHz) */
	LLI      [NFREQ + NEXOBS]uint8   /* loss of lock indicator */
	Code     [NFREQ + NEXOBS]uint8   /* code indicator (CODE_???) */
	L        [NFREQ + NEXOBS]float64 /* observation data carrier-phase (cycle) */
	P        [NFREQ + NEXOBS]float64 /* observation data pseudorange (m) */
	D        [NFREQ + NEXOBS]float64 /* observation data doppler frequency (Hz) */
}

type Obs struct { /* observation data */
	//N, NMax int    /* number of obervation data/allocated */
	Data []ObsD /* observation data records */
	n    int    /* actual number of obervation data */
}

func (obs *Obs) N() int {
	if obs.n > 0 {
		return obs.n
	}
	return len(obs.Data)
}

// func (obs *Obs) NMax() int {
// 	return len(obs.Data)
// }

type ErpD struct { /* earth rotation parameter data type */
	Mjd      float64 /* mjd (days) */
	Xp, Yp   float64 /* pole offset (rad) */
	Xpr, Ypr float64 /* pole offset rate (rad/day) */
	Ut1_utc  float64 /* ut1-utc (s) */
	Lod      float64 /* length of day (s/day) */
}

type Erp struct { /* earth rotation parameter type */
	Data []ErpD /* earth rotation parameter data */
}

func (erp *Erp) N() int {
	return len(erp.Data)
}

type Pcv struct { /* antenna parameter type */
	Sat       int                /* satellite number (0:receiver) */
	Type      string             /* antenna type */
	Code      string             /* serial number or satellite code */
	Ts, Te    Gtime              /* valid time start and end */
	Offset    [NFREQ][3]float64  /* phase center offset e/n/u or x/y/z (m) */
	Variation [NFREQ][19]float64 /* phase center variation (m) */
	/* el=90,85,...,0 or nadir=0,1,2,3,... (deg) */
}

type Pcvs struct { /* antenna parameters type */
	Pcv []Pcv /* antenna parameters data */
}

func (pcvs *Pcvs) N() int {
	return len(pcvs.Pcv)
}

type Alm struct { /* almanac type */
	Sat    int   /* satellite number */
	Svh    int   /* sv health (0:ok) */
	SvConf int   /* as and sv config */
	Week   int   /* GPS/QZS: gps week, GAL: galileo week */
	Toa    Gtime /* Toa */
	/* SV orbit parameters */
	A, E, I0, OMG0, Omg, M0, OMGd float64
	Toas                          float64 /* Toa (s) in week */
	F0, F1                        float64 /* SV clock parameters (af0,af1) */
}

type Eph struct { /* GPS/QZS/GAL broadcast ephemeris type */
	Sat        int /* satellite number */
	Iode, Iodc int /* IODE,IODC */
	Sva        int /* SV accuracy (URA index) */
	Svh        int /* SV health (0:ok) */
	Week       int /* GPS/QZS: gps week, GAL: galileo week */
	Code       int /* GPS/QZS: code on L2 */
	/* GAL: data source defined as rinex 3.03 */
	/* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
	Flag int /* GPS/QZS: L2 P data flag */
	/* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	Toe, Toc, Ttr Gtime /* Toe,Toc,T_trans */
	/* SV orbit parameters */
	A, E, I0, OMG0, Omg, M0, Deln, OMGd, Idot float64
	Crc, Crs, Cuc, Cus, Cic, Cis              float64
	Toes                                      float64    /* Toe (s) in week */
	Fit                                       float64    /* fit interval (h) */
	F0, F1, F2                                float64    /* SV clock parameters (af0,af1,af2) */
	Tgd                                       [6]float64 /* group delay parameters */
	/* GPS/QZS:tgd[0]=TGD */
	/* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b */
	/* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
	/*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
	Adot, Ndot float64 /* Adot,ndot for CNAV */
}

type GEph struct { /* GLONASS broadcast ephemeris type */
	Sat           int        /* satellite number */
	Iode          int        /* IODE (0-6 bit of tb field) */
	Frq           int        /* satellite frequency number */
	Svh, Sva, Age int        /* satellite health, accuracy, age of operation */
	Toe           Gtime      /* epoch of epherides (gpst) */
	Tof           Gtime      /* message frame time (gpst) */
	Pos           [3]float64 /* satellite position (ecef) (m) */
	Vel           [3]float64 /* satellite velocity (ecef) (m/s) */
	Acc           [3]float64 /* satellite acceleration (ecef) (m/s^2) */
	Taun, Gamn    float64    /* SV clock bias (s)/relative freq bias */
	DTaun         float64    /* delay between L1 and L2 (s) */
}

type PEph struct { /* precise ephemeris type */
	Time   Gtime              /* time (GPST) */
	Index  int                /* ephemeris index for multiple files */
	Pos    [MAXSAT][4]float64 /* satellite position/clock (ecef) (m|s) */
	Std    [MAXSAT][4]float32 /* satellite position/clock std (m|s) */
	Vel    [MAXSAT][4]float64 /* satellite velocity/clk-rate (m/s|s/s) */
	Vst    [MAXSAT][4]float32 /* satellite velocity/clk-rate std (m/s|s/s) */
	PosCov [MAXSAT][3]float32 /* satellite position covariance (m^2) */
	VelCov [MAXSAT][3]float32 /* satellite velocity covariance (m^2) */
}

type PClk struct { /* precise clock type */
	Time  Gtime              /* time (GPST) */
	Index int                /* clock index for multiple files */
	Clk   [MAXSAT][1]float64 /* satellite clock (s) */
	Std   [MAXSAT][1]float32 /* satellite clock std (s) */
}

type SEph struct { /* SBAS ephemeris type */
	Sat      int        /* satellite number */
	T0       Gtime      /* reference epoch time (GPST) */
	Tof      Gtime      /* time of message frame (GPST) */
	Sva      int        /* SV accuracy (URA index) */
	Svh      int        /* SV health (0:ok) */
	Pos      [3]float64 /* satellite position (m) (ecef) */
	Vel      [3]float64 /* satellite velocity (m/s) (ecef) */
	Acc      [3]float64 /* satellite acceleration (m/s^2) (ecef) */
	Af0, Af1 float64    /* satellite clock-offset/drift (s,s/s) */
}

type TleD struct { /* NORAL TLE data type */
	Name     string  /* common name */
	Alias    string  /* alias name */
	SatNo    string  /* satellilte catalog number */
	SatClass byte    /* classification */
	Desig    string  /* international designator */
	Epoch    Gtime   /* element set epoch (UTC) */
	Ndot     float64 /* 1st derivative of mean motion */
	NDdot    float64 /* 2st derivative of mean motion */
	BStar    float64 /* B* drag term */
	EType    int     /* element set type */
	EleNo    int     /* element number */
	Inc      float64 /* orbit inclination (deg) */
	OMG      float64 /* right ascension of ascending node (deg) */
	Ecc      float64 /* eccentricity */
	Omg      float64 /* argument of perigee (deg) */
	M        float64 /* mean anomaly (deg) */
	N        float64 /* mean motion (rev/day) */
	RevNo    int     /* revolution number at epoch */
}

type Tle struct { /* NORAD TLE (two line element) type */
	N, Nmax int    /* number/max number of two line element data */
	Data    []TleD /* NORAD TLE data */
}

type Tec struct { /* TEC grid type */
	Time   Gtime      /* epoch time (GPST) */
	Ndata  [3]int     /* TEC grid data size {nlat,nlon,nhgt} */
	Radius float64    /* earth radius (km) */
	Lats   [3]float64 /* latitude start/interval (deg) */
	Lons   [3]float64 /* longitude start/interval (deg) */
	Hgts   [3]float64 /* heights start/interval (km) */
	Data   []float64  /* TEC grid data (tecu) */
	Rms    []float32  /* RMS values (tecu) */
}

type SbsMsg struct { /* SBAS message type */
	Week, Tow int       /* receiption time */
	Prn, Rcv  uint8     /* SBAS satellite PRN,receiver number */
	Msg       [29]uint8 /* SBAS message (226bit) padded by 0 */
}

type Sbs struct { /* SBAS messages type */
	Msgs []SbsMsg /* SBAS messages */
}

func (sbs *Sbs) N() int {
	return len(sbs.Msgs)
}

type SbsFCorr struct { /* SBAS fast correction type */
	t0   Gtime   /* time of applicability (TOF) */
	prc  float64 /* pseudorange correction (PRC) (m) */
	rrc  float64 /* range-rate correction (RRC) (m/s) */
	dt   float64 /* range-rate correction delta-time (s) */
	iodf int     /* IODF (issue of date fast corr) */
	udre int16   /* UDRE+1 */
	ai   int16   /* degradation factor indicator */
}

type SbsLCorr struct { /* SBAS long term satellite error correction type */
	t0         Gtime      /* correction time */
	iode       int        /* IODE (issue of date ephemeris) */
	dpos       [3]float64 /* delta position (m) (ecef) */
	dvel       [3]float64 /* delta velocity (m/s) (ecef) */
	daf0, daf1 float64    /* delta clock-offset/drift (s,s/s) */
}

type SbsSatP struct { /* SBAS satellite correction type */
	sat   int      /* satellite number */
	fcorr SbsFCorr /* fast correction */
	lcorr SbsLCorr /* long term correction */
}

type SbsSat struct { /* SBAS satellite corrections type */
	iodp int             /* IODP (issue of date mask) */
	nsat int             /* number of satellites */
	tlat int             /* system latency (s) */
	sat  [MAXSAT]SbsSatP /* satellite correction */
}

type SbsIgp struct { /* SBAS ionospheric correction type */
	t0       Gtime   /* correction time */
	lat, lon int16   /* latitude/longitude (deg) */
	give     int16   /* GIVI+1 */
	delay    float32 /* vertical delay estimate (m) */
}
type RefSbsIgp *SbsIgp

type SbsIgpBand struct { /* IGP band type */
	x    int16   /* longitude/latitude (deg) */
	y    []int16 /* latitudes/longitudes (deg) */
	bits uint8   /* IGP mask start bit */
	bite uint8   /* IGP mask end bit */
}

type SbsIon struct { /* SBAS ionospheric corrections type */
	iodi int             /* IODI (issue of date ionos corr) */
	nigp int             /* number of igps */
	igp  [MAXNIGP]SbsIgp /* ionospheric correction */
}

type DGps struct { /* DGPS/GNSS correction type */
	t0   Gtime   /* correction time */
	prc  float64 /* pseudorange correction (PRC) (m) */
	rrc  float64 /* range rate correction (RRC) (m/s) */
	iod  int     /* issue of data (IOD) */
	udre float64 /* UDRE */
}

type SSR struct { /* SSR correction type */
	T0                [6]Gtime         /* epoch time (GPST) {eph,clk,hrclk,ura,bias,pbias} */
	Udi               [6]float64       /* SSR update interval (s) */
	Iod               [6]int           /* iod ssr {eph,clk,hrclk,ura,bias,pbias} */
	Iode              int              /* issue of data */
	IodCrc            int              /* issue of data crc for beidou/sbas */
	Ura               int              /* URA indicator */
	Refd              int              /* sat ref datum (0:ITRF,1:regional) */
	Deph              [3]float64       /* delta orbit {radial,along,cross} (m) */
	Ddeph             [3]float64       /* dot delta orbit {radial,along,cross} (m/s) */
	Dclk              [3]float64       /* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
	Brclk             float64          /* high-rate clock corection (m) */
	Cbias             [MAXCODE]float32 /* code biases (m) */
	Pbias             [MAXCODE]float64 /* phase biases (m) */
	Stdpb             [MAXCODE]float32 /* std-dev of phase biases (m) */
	Yaw_ang, Yaw_rate float64          /* yaw angle and yaw rate (deg,deg/s) */
	Update            uint8            /* update flag (0:no update,1:update) */
}
type SatDCB [3]float64
type RcvDCB [2][3]float64

type Nav struct { /* navigation data type */
	// N, Nmax   int                   /* number of broadcast ephemeris */
	// Ng, NgMax int                   /* number of glonass ephemeris */
	// Ns, NsMax int                   /* number of sbas ephemeris */
	// Ne, NeMax int                   /* number of precise ephemeris */
	// Nc, NcMax int                   /* number of precise clock */
	// Na, NaMax int                   /* number of almanac data */
	// Nt, NtMax int                   /* number of tec grid data */
	Ephs    []Eph                 /* GPS/QZS/GAL/BDS/IRN ephemeris */ //[MAXSAT * 2]Eph
	Geph    []GEph                /* GLONASS ephemeris */
	Seph    []SEph                /* SBAS ephemeris */
	Peph    []PEph                /* precise ephemeris */
	Pclk    []PClk                /* precise clock */
	Alm     []Alm                 /* almanac data */
	Tec     []Tec                 /* tec grid data */
	Erp     Erp                   /* earth rotation parameters */
	Utc_gps [8]float64            /* GPS delta-UTC parameters {A0,A1,Tot,WNt,dt_LS,WN_LSF,DN,dt_LSF} */
	Utc_glo [8]float64            /* GLONASS UTC time parameters {tau_C,tau_GPS} */
	Utc_gal [8]float64            /* Galileo UTC parameters */
	Utc_qzs [8]float64            /* QZS UTC parameters */
	Utc_cmp [8]float64            /* BeiDou UTC parameters */
	Utc_irn [9]float64            /* IRNSS UTC parameters {A0,A1,Tot,...,dt_LSF,A2} */
	Utc_sbs [4]float64            /* SBAS UTC parameters */
	Ion_gps [8]float64            /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	Ion_gal [4]float64            /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	Ion_qzs [8]float64            /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	Ion_cmp [8]float64            /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	Ion_irn [8]float64            /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	Glo_fcn [32]int               /* GLONASS FCN + 8 */
	CBias   [MAXSAT][3]float64    /* satellite DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
	RBias   [MAXRCV][2][3]float64 /* receiver DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
	Pcvs    [MAXSAT]Pcv           /* satellite antenna pcv */
	SbasSat SbsSat                /* SBAS satellite corrections */
	SbasIon [MAXBAND + 1]SbsIon   /* SBAS ionosphere corrections */
	Dgps    [MAXSAT]DGps          /* DGPS corrections */
	Ssr     [MAXSAT]SSR           /* SSR corrections */
}

func (nav *Nav) N() int {
	return len(nav.Ephs)
}

func (nav *Nav) Ng() int {
	return len(nav.Geph)
}

func (nav *Nav) Ns() int {
	return len(nav.Seph)
}
func (nav *Nav) Ne() int {
	return len(nav.Peph)
}
func (nav *Nav) Nc() int {
	return len(nav.Pclk)
}
func (nav *Nav) Na() int {
	return len(nav.Alm)
}
func (nav *Nav) Nt() int {
	return len(nav.Tec)
}

type Sta struct { /* station parameter type */
	Name         string     /* marker name */
	Marker       string     /* marker number */
	AntDes       string     /* antenna descriptor */
	AntSno       string     /* antenna serial number */
	Type         string     /* receiver type descriptor */
	RecVer       string     /* receiver firmware version */
	RecSN        string     /* receiver serial number */
	AntSetup     int        /* antenna setup id */
	Itrf         int        /* ITRF realization year */
	DelType      int        /* antenna delta type (0:enu,1:xyz) */
	Pos          [3]float64 /* station position (ecef) (m) */
	Del          [3]float64 /* antenna position delta (e/n/u or x/y/z) (m) */
	Hgt          float64    /* antenna height (m) */
	glo_cp_align int        /* GLONASS code-phase alignment (0:no,1:yes) */
	glo_cp_bias  [4]float64 /* GLONASS code-phase biases {1C,1P,2C,2P} (m) */
}

type Sol struct { /* solution type */
	Time Gtime      /* time (GPST) */
	Rr   [6]float64 /* position/velocity (m|m/s) */
	/* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
	Qr [6]float32 /* position variance/covariance (m^2) */
	/* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
	/* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
	Qv    [6]float32 /* velocity variance/covariance (m^2/s^2) */
	Dtr   [6]float64 /* receiver clock bias to time systems (s) */
	Type  uint8      /* type (0:xyz-ecef,1:enu-baseline) */
	Stat  uint8      /* solution status (SOLQ_???) */
	Ns    uint8      /* number of valid satellites */
	Age   float32    /* age of differential (s) */
	Ratio float32    /* AR ratio factor for valiation */
	Thres float32    /* AR ratio threshold for valiation */
}

type SolBuf struct { /* solution buffer type */
	N, Nmax    int        /* number of solution/max number of buffer */
	Cyclic     int        /* cyclic buffer flag */
	Start, End int        /* start/end index */
	Time       Gtime      /* current solution time */
	Data       []Sol      /* solution data */
	Rb         [3]float64 /* reference position {x,y,z} (ecef) (m) */
	buff       []byte
}

type SolStat struct { /* solution status type */
	Time   Gtime   /* time (GPST) */
	Sat    uint8   /* satellite number */
	Frq    uint8   /* frequency (1:L1,2:L2,...) */
	Az, El float32 /* azimuth/elevation angle (rad) */
	Resp   float32 /* pseudorange residual (m) */
	Resc   float32 /* carrier-phase residual (m) */
	Flag   uint8   /* flags: (vsat<<5)+(slip<<3)+fix */
	Snr    uint16  /* signal strength (*SNR_UNIT dBHz) */
	lock   uint16  /* lock counter */
	outc   uint16  /* outage counter */
	slipc  uint16  /* slip counter */
	rejc   uint16  /* reject counter */
}

type SolStatBuf struct { /* solution status buffer type */
	n, nmax int       /* number of solution/max number of buffer */
	data    []SolStat /* solution status data */
}

type Rtcm struct { /* RTCM control struct type */
	StaId     int                             /* station id */
	StaHealth int                             /* station health */
	SeqNo     int                             /* sequence number for rtcm 2 or iods msm */
	OutType   int                             /* output message type */
	Time      Gtime                           /* message time */
	Time_s    Gtime                           /* message start time */
	ObsData   Obs                             /* observation data (uncorrected) */
	NavData   Nav                             /* satellite ephemerides */
	StaPara   Sta                             /* station parameters */
	Dgps      [MAXSAT]DGps                    /* output of dgps corrections */
	Ssr       [MAXSAT]SSR                     /* output of ssr corrections */
	Msg       string                          /* special message */
	MsgType   string                          /* last message type */
	MsmType   [7]string                       /* msm signal types */
	ObsFlag   int                             /* obs data complete flag (1:ok,0:not complete) */
	EphSat    int                             /* input ephemeris satellite number */
	EphSet    int                             /* input ephemeris set (0-1) */
	Cp        [MAXSAT][NFREQ + NEXOBS]float64 /* carrier-phase measurement */
	Lock      [MAXSAT][NFREQ + NEXOBS]uint16  /* lock time */
	Loss      [MAXSAT][NFREQ + NEXOBS]uint16  /* loss of lock count */
	Lltime    [MAXSAT][NFREQ + NEXOBS]Gtime   /* last lock time */
	Nbyte     int                             /* number of bytes in message buffer */
	Nbit      int                             /* number of bits in word buffer */
	MsgLen    int                             /* message length (bytes) */
	Buff      [1200]byte                      /* message buffer */
	Word      uint32                          /* word buffer for rtcm 2 */
	Nmsg2     [100]uint32                     /* message count of RTCM 2 (1-99:1-99,0:other) */
	Nmsg3     [400]uint32                     /* message count of RTCM 3 (1-299:1001-1299,300-329:4070-4099,0:ohter) */
	Opt       string                          /* RTCM dependent options */
}
type TOBS [8][MAXOBSTYPE]string
type RnxCtr struct { /* RINEX control struct type */
	time     Gtime   /* message time */
	ver      float64 /* RINEX version */
	filetype string  /* RINEX file type ('O','N',...) */
	sys      int     /* navigation system */
	tsys     int     /* time system */
	tobs     TOBS    /* rinex obs types */
	obs      Obs     /* observation data */
	nav      Nav     /* navigation data */
	sta      Sta     /* station info */
	ephsat   int     /* input ephemeris satellite number */
	ephset   int     /* input ephemeris set (0-1) */
	opt      string  /* rinex dependent options */
}

type Url struct { /* download URL type */
	dtype string  /* data type */
	path  string  /* URL path */
	dir   string  /* local directory */
	tint  float64 /* time interval (s) */
}

type Opt struct { /* option type */
	Name      string   /* option name */
	Format    byte     /* option format (0:int,1:float64,2:string,3:enum) */
	VarInt    *int     /* pointer to option variable */
	VarFloat  *float64 /* pointer to option variable */
	VarString *string  /* pointer to option variable */
	Comment   string   /* option comment/enum labels/unit */
}

type SnrMask struct { /* SNR mask type */
	ena  [2]int            /* enable flag {rover,base} */
	mask [NFREQ][9]float64 /* mask (dBHz) at 5,10,...85 deg */
}

type PrcOpt struct { /* processing options type */
	Mode       int            /* positioning mode (PMODE_???) */
	eratio     [NFREQ]float64 /* code/phase error ratio */
	SolType    int            /* solution type (0:forward,1:backward,2:combined) */
	Nf         int            /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
	NavSys     int            /* navigation system */
	Elmin      float64        /* elevation mask angle (rad) */
	SnrMask    SnrMask        /* SNR mask */
	SatEph     int            /* satellite ephemeris/clock (EPHOPT_???) */
	ModeAr     int            /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
	GloModeAr  int            /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
	BDSModeAr  int            /* BeiDou AR mode (0:off,1:on) */
	MaxOut     int            /* obs outage count to reset bias */
	MinLock    int            /* min lock count to fix ambiguity */
	MinFix     int            /* min fix count to hold ambiguity */
	ArMaxIter  int            /* max iteration to resolve ambiguity */
	IonoOpt    int            /* ionosphere option (IONOOPT_???) */
	TropOpt    int            /* troposphere option (TROPOPT_???) */
	Dynamics   int            /* dynamics model (0:none,1:velociy,2:accel) */
	TideCorr   int            /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
	NoIter     int            /* number of filter iteration */
	CodeSmooth int            /* code smoothing window size (0:none) */
	IntPref    int            /* interpolate reference obs (for post mission) */
	SbasCorr   int            /* SBAS correction options */
	SbasSatSel int            /* SBAS satellite selection (0:all) */
	RovPos     int            /* rover position for fixed mode */
	RefPos     int            /* base position for relative mode */
	/* (0:pos in prcopt,  1:average of single pos, */
	/*  2:read from file, 3:rinex header, 4:rtcm pos) */
	Err [5]float64 /* measurement error factor */
	/* [0]:reserved */
	/* [1-3]:error factor a/b/c of phase (m) */
	/* [4]:doppler frequency (hz) */
	Std        [3]float64         /* initial-state std [0]bias,[1]iono [2]trop */
	Prn        [6]float64         /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
	SatClkStab float64            /* satellite clock stability (sec/sec) */
	ThresAr    [8]float64         /* AR validation threshold */
	ElMaskAr   float64            /* elevation mask of AR for rising satellite (deg) */
	ElMaskHold float64            /* elevation mask to hold ambiguity (deg) */
	ThresSlip  float64            /* slip threshold of geometry-free phase (m) */
	MaxTmDiff  float64            /* max difference of time (sec) */
	MaxInno    float64            /* reject threshold of innovation (m) */
	MaxGdop    float64            /* reject threshold of gdop */
	Baseline   [2]float64         /* baseline length constraint {const,sigma} (m) */
	Ru         [3]float64         /* rover position for fixed mode {x,y,z} (ecef) (m) */
	Rb         [3]float64         /* base position for relative mode {x,y,z} (ecef) (m) */
	AntType    [2]string          /* antenna types {rover,base} */
	AntDel     [2][3]float64      /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
	Pcvr       [2]Pcv             /* receiver antenna parameters {rov,base} */
	ExSats     [MAXSAT]uint8      /* excluded satellites (1:excluded,2:included) */
	MaxAveEp   int                /* max averaging epoches */
	InitRst    int                /* initialize by restart */
	OutSingle  int                /* output single by dgps/float/fix/ppp outage */
	RnxOpt     [2]string          /* rinex options {rover,base} */
	PosOpt     [6]int             /* positioning options */
	SyncSol    int                /* solution sync mode (0:off,1:on) */
	Odisp      [2][6 * 11]float64 /* ocean tide loading parameters {rov,base} */
	FreqOpt    int                /* disable L2-AR */
	PPPOpt     string             /* ppp option */
}

type SolOpt struct { /* solution options type */
	Posf      int        /* solution format (SOLF_???) */
	TimeS     int        /* time system (TIMES_???) */
	TimeF     int        /* time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
	TimeU     int        /* time digits under decimal point */
	DegF      int        /* latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
	OutHead   int        /* output header (0:no,1:yes) */
	OutOpt    int        /* output processing options (0:no,1:yes) */
	OutVel    int        /* output velocity options (0:no,1:yes) */
	Datum     int        /* datum (0:WGS84,1:Tokyo) */
	Height    int        /* height (0:ellipsoidal,1:geodetic) */
	Geoid     int        /* geoid model (0:EGM96,1:JGD2000) */
	SolStatic int        /* solution of static mode (0:all,1:single) */
	SStat     int        /* solution statistics level (0:off,1:states,2:residuals) */
	Trace     int        /* debug trace level (0:off,1-5:debug) */
	NmeaIntv  [2]float64 /* nmea output interval (s) (<0:no,0:all) */
	/* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
	Sep       string  /* field separator */
	Prog      string  /* program name */
	MaxSolStd float64 /* max std-dev for solution output (m) (0:all) */
}

type FilOpt struct { /* file options type */
	SatAntPara string /* satellite antenna parameters file */
	RcvAntPara string /* receiver antenna parameters file */
	StaPos     string /* station positions file */
	Geoid      string /* external geoid data file */
	Iono       string /* ionosphere data file */
	Dcb        string /* dcb data file */
	Eop        string /* eop data file */
	Blq        string /* ocean tide loading blq file */
	TempDir    string /* ftp/http temporaly directory */
	GeExe      string /* google earth exec file */
	SolStat    string /* solution statistics file */
	Trace      string /* debug trace file */
}

type SSat struct { /* satellite status type */
	Sys   uint8              /* navigation system */
	Vs    uint8              /* valid satellite flag single */
	Azel  [2]float64         /* azimuth/elevation angles {az,el} (rad) */
	Resp  [NFREQ]float32     /* residuals of pseudorange (m) */
	Resc  [NFREQ]float32     /* residuals of carrier-phase (m) */
	Vsat  [NFREQ]uint8       /* valid satellite flag */
	Snr   [NFREQ]uint16      /* signal strength (*SNR_UNIT dBHz) */
	Fix   [NFREQ]uint8       /* ambiguity fix flag (1:fix,2:float,3:hold) */
	Slip  [NFREQ]uint8       /* cycle-slip flag */
	Half  [NFREQ]uint8       /* half-cycle valid flag */
	Lock  [NFREQ]int         /* lock counter of phase */
	Outc  [NFREQ]uint32      /* obs outage counter of phase */
	Slipc [NFREQ]uint32      /* cycle-slip counter */
	Rejc  [NFREQ]uint32      /* reject counter */
	Gf    [NFREQ - 1]float64 /* geometry-free phase (m) */
	Mw    [NFREQ - 1]float64 /* MW-LC (m) */
	Phw   float64            /* phase windup (cycle) */
	Pt    [2][NFREQ]Gtime    /* previous carrier-phase time */
	Ph    [2][NFREQ]float64  /* previous carrier-phase observable (cycle) */
}

type AmbC struct { /* ambiguity control type */
	epoch  [4]Gtime     /* last epoch */
	n      [4]int       /* number of epochs */
	LC     [4]float64   /* linear combination average */
	LCv    [4]float64   /* linear combination variance */
	fixcnt int          /* fix count */
	flags  [MAXSAT]byte /* fix flags */
}

type Rtk struct { /* RTK control/result type */
	RtkSol Sol          /* RTK solution */
	Rb     [6]float64   /* base position/velocity (ecef) (m|m/s) */
	Nx, Na int          /* number of float states/fixed states */
	Tt     float64      /* time difference between current and previous (s) */
	X, P   []float64    /* float states and their covariance */
	Xa, Pa []float64    /* fixed states and their covariance */
	Nfix   int          /* number of continuous fixes of ambiguity */
	Ambc   [MAXSAT]AmbC /* ambibuity control */
	Ssat   [MAXSAT]SSat /* satellite status */
	//neb    int             /* bytes in error message buffer, abandon in go */
	ErrBuf string /* error message buffer */
	Opt    PrcOpt /* processing options */
}
type Stream struct { /* stream type */
	Type                   int         /* type (STR_???) */
	Mode                   int         /* mode (STR_MODE_?) */
	State                  int         /* state (-1:error,0:close,1:open) */
	InBytes, InRate        uint32      /* input bytes/rate */
	OutBytes, OutRate      uint32      /* output bytes/rate */
	TickInput              int64       /* input tick tick */
	TickOutput             int64       /* output tick */
	TickActive             int64       /* active tick */
	InByeTick, OutByteTick uint32      /* input/output bytes at tick */
	Lock                   sync.Mutex  /* lock flag */
	Port                   interface{} /* type dependent port control struct */
	Path                   string      /* stream path */
	Msg                    string      /* stream message */
}

type Raw struct { /* receiver raw data control type */
	Time       Gtime                         /* message time */
	Tobs       [MAXSAT][NFREQ_NEXOBS]Gtime   /* observation data time [MAXSAT][]*/
	ObsData    Obs                           /* observation data */
	ObsBuf     Obs                           /* observation data buffer */
	NavData    Nav                           /* satellite ephemerides */
	StaData    Sta                           /* station parameters */
	EphSat     int                           /* update satelle of ephemeris (0:no satellite) */
	EphSet     int                           /* update set of ephemeris (0-1) */
	Sbsmsg     SbsMsg                        /* SBAS message */
	MsgType    [256]byte                     /* last message type */
	SubFrm     [MAXSAT][380]uint8            /* subframe buffer [MAXSAT][380]*/
	LockTime   [MAXSAT][NFREQ_NEXOBS]float64 /* lock time (s)  [MAXSAT][NFREQ_NEXOBS]*/
	Icpp, Off  [MAXSAT]float64               /* carrier params for ss2 */
	Icpc       float64
	PrCA, DpCA [MAXSAT]float64             /* L1/CA pseudrange/doppler for javad */
	Halfc      [MAXSAT][NFREQ_NEXOBS]uint8 /* half-cycle add flag [MAXSAT][NFREQ_NEXOBS]*/
	FreqNum    [MAXOBS]byte                /* frequency number for javad */
	NumByte    int                         /* number of bytes in message buffer */
	Len        int                         /* message length (bytes) */
	Iod        int                         /* issue of data */
	Tod        int                         /* time of day (ms) */
	Tbase      int                         /* time base (0:gpst,1:utc(usno),2:glonass,3:utc(su) */
	Flag       int                         /* general purpose flag */
	OutType    int                         /* output message type */
	Buff       [MAXRAWLEN]uint8            /* message buffer */
	Opt        string                      /* receiver dependent options */
	Format     int                         /* receiver stream format */
	RcvData    []byte                      /* receiver dependent data */
}

type StrConv struct { /* stream converter type */
	InputType, OutputType int         /* input and output stream type */
	NoMsg                 int         /* number of output messages */
	MsgType               [32]int     /* output message types */
	OutInterval           [32]float64 /* output message intervals (s) */
	Tick                  [32]uint32  /* cycle tick of output message */
	EphSat                [32]int     /* satellites of output ephemeris */
	StationSel            int         /* station info selection (0:remote,1:local) */
	RtcmInput             Rtcm        /* rtcm input data buffer */
	RawInput              Raw         /* raw  input data buffer */
	RtcmOutput            Rtcm        /* rtcm output data buffer */
}

type StreamSvr struct { /* stream server type */
	State        int            /* server state (0:stop,1:running) */
	Cycle        int            /* server cycle (ms) */
	BuffSize     int            /* input/monitor buffer size (bytes) */
	NmeaCycle    int            /* NMEA request cycle (ms) (0:no) */
	RelayBack    int            /* relay back of output streams (0:no) */
	NoStream     int            /* number of streams (1 input + (nstr-1) outputs */
	Npb          int            /* data length in peek buffer (bytes) */
	CmdsPeriodic [16]string     /* periodic commands */
	NmeaPos      [3]float64     /* NMEA request position (ecef) (m) */
	Buff         []uint8        /* input buffers */
	PeekBuf      []uint8        /* peek buffer */
	Tick         int64          /* start tick */
	InputStream  [16]Stream     /* input/output streams */
	StreamLog    [16]Stream     /* return log streams */
	Converter    [16]*StrConv   /* stream converter */
	Wg           sync.WaitGroup /* server thread */
	Lock         sync.Mutex     /* lock flag */
}

type RtkSvr struct { /* RTK server type */
	State        int               /* server state (0:stop,1:running) */
	Cycle        int               /* processing cycle (ms) */
	NmeaCycle    int               /* NMEA request cycle (ms) (0:no req) */
	NmeaReq      int               /* NMEA request (0:no,1:nmeapos,2:single sol) */
	NmeaPos      [3]float64        /* NMEA request position (ecef) (m) */
	BuffSize     int               /* input buffer size (bytes) */
	Format       [3]int            /* input format {rov,base,corr} */
	Solopt       [2]SolOpt         /* output solution options {sol1,sol2} */
	NavSel       int               /* ephemeris select (0:all,1:rover,2:base,3:corr) */
	NoSbs        int               /* number of sbas message */
	NoSol        int               /* number of solution buffer */
	RtkCtrl      Rtk               /* RTK control/result struct */
	Nb           [3]int            /* bytes in input buffers {rov,base} */
	Nsb          [2]int            /* bytes in soulution buffers */
	Npb          [3]int            /* bytes in input peek buffers */
	Buff         [3][]uint8        /* input buffers {rov,base,corr} */
	SBuf         [2][]uint8        /* output buffers {sol1,sol2} */
	PBuf         [3][]uint8        /* peek buffers {rov,base,corr} */
	SolBuf       [MAXSOLBUF]Sol    /* solution buffer */
	InputMsg     [3][10]uint32     /* input message counts */
	RawCtrl      [3]Raw            /* receiver raw control {rov,base,corr} */
	RtcmCtrl     [3]Rtcm           /* RTCM control {rov,base,corr} */
	DownloadTime [3]Gtime          /* download time {rov,base,corr} */
	Files        [3]string         /* download paths {rov,base,corr} */
	ObsData      [3][MAXOBSBUF]Obs /* observation data {rov,base,corr} */
	NavData      Nav               /* navigation data */
	SbsMsg       [MAXSBSMSG]SbsMsg /* SBAS message buffer */
	Stream       [8]Stream         /* streams {rov,base,corr,sol1,sol2,logr,logb,logc} */
	Monitor      *Stream           /* monitor stream */
	Tick         uint32            /* start tick */
	Thread       int               /* server thread */
	CpuTime      int               /* CPU time (ms) for a processing cycle */
	PrcOut       int               /* missing observation data count */
	NAve         int               /* number of averaging base pos */
	Rb_ave       [3]float64        /* averaging base pos */
	CmdsPeriodic [3]string         /* periodic commands */
	CmdReset     string            /* reset command */
	BaseLenReset float64           /* baseline length to reset (km) */
	Lock         sync.Mutex        /* lock flag */
	Wg           sync.WaitGroup    /* thread conter is used to indicate thread exit */
}

type RnxOpt struct { /* RINEX options type */
	TS, TE      Gtime                  /* time start/end */
	TInt        float64                /* time interval (s) */
	TTol        float64                /* time tolerance (s) */
	TUnit       float64                /* time unit for multiple-session (s) */
	RnxVer      int                    /* RINEX version (x100) */
	NavSys      int                    /* navigation system */
	ObsType     int                    /* observation type */
	FreqType    int                    /* frequency type */
	Mask        [7][64]byte            /* code mask {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
	Staid       string                 /* station id for rinex file name */
	Prog        string                 /* program */
	RunBy       string                 /* run-by */
	Marker      string                 /* marker name */
	MarkerNo    string                 /* marker number */
	MarkerType  string                 /* marker type (ver.3) */
	Name        [2]string              /* observer/agency */
	Rec         [3]string              /* receiver #/type/vers */
	Ant         [3]string              /* antenna #/type */
	AppPos      [3]float64             /* approx position x/y/z */
	AntDel      [3]float64             /* antenna delta h/e/n */
	Glo_cp_bias [4]float64             /* GLONASS code-phase biases (m) */
	Comment     [MAXCOMMENT]string     /* comments */
	RcvOpt      string                 /* receiver dependent options */
	ExSats      [MAXSAT]uint8          /* excluded satellites */
	GloFcn      [32]int                /* glonass fcn+8 */
	Outiono     int                    /* output iono correction */
	OutputTime  int                    /* output time system correction */
	Outleaps    int                    /* output leap seconds */
	AutoPos     int                    /* auto approx position */
	PhShift     int                    /* phase shift correction */
	Halfcyc     int                    /* half cycle correction */
	Sep_Nav     int                    /* separated nav files */
	TStart      Gtime                  /* first obs time */
	TEnd        Gtime                  /* last obs time */
	TRtcm       Gtime                  /* approx log start time for rtcm */
	TObs        [7][MAXOBSTYPE]string  /* obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
	Shift       [7][MAXOBSTYPE]float64 /* phase shift (cyc) {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
	NObs        [7]int                 /* number of obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
}
type Gis_Pnt struct { /* GIS data point type */
	pos [3]float64 /* point data {lat,lon,height} (rad,m) */
}

type Gis_Poly struct { /* GIS data polyline type */
	npnt  int        /* number of points */
	bound [4]float64 /* boundary {lat0,lat1,lon0,lon1} */
	pos   []float64  /* position data (3 x npnt) */
}

type Gis_Polygon struct { /* GIS data polygon type */
	npnt  int        /* number of points */
	bound [4]float64 /* boundary {lat0,lat1,lon0,lon1} */
	pos   []float64  /* position data (3 x npnt) */
}

type GisD struct { /* GIS data list type */
	dtype int         /* data type (1:point,2:polyline,3:polygon) */
	data  interface{} /* data body */
	next  *GisD       /* pointer to next */
}

type Gis struct { /* GIS type */
	name  [MAXGISLAYER]string /* name */
	flag  [MAXGISLAYER]int    /* flag */
	data  [MAXGISLAYER]*GisD  /* gis data list */
	bound [4]float64          /* boundary {lat0,lat1,lon0,lon1} */
}
