#
#  gnssgo 1.0
#

# DESCRIPTION

The development branch for golang version of RTKLIB 2.4.3.

RTKLIB is an excellent tool for GNSS development and research. Golang became population in recent. According to requirement of project, I recoded RTKLIB with golang for almost one years.

Due to limitaion of commit folder size, I didn't commit Data. If you want to test the project, you could download from RTKLIB.

Author: Dr. Feng Xuebin, IDT Ltd., co

# UPDATE HISTORY

2023/06/06 1.0 

# Build and Installation
In file go.work, you could choose your app by uncommenting app path.

For confiuring app, you could define command arguments in file luanch.json under directory .vscode.
    
    GENERAL:
        NavIC (IRNSS) completely supported.
        RINEX 3.04 supported. BDS-3 and QZSS new signals added.
        RTCM 3.3 amendment-1 supported. MT1041/1131-7 (NavIC ephemeris/MSM) added.
        RTCM3 MT1230 (GLONASS code-phase biases) supported.
        RTCM3 MT4076 (IGS SSR) supported.
        GNSS singal ID changed: L1,L2,L5/3,L6,L7,L8,L9 -> L1,L2,L3,L4,L5.
        Only Windows 64bit APs supported. 32bit APs deleted. 
        Windows scaled DPI APs supported for different DPI screens.
        Directories RTKLIB/app and RTKLIB/data reorganized.
        License clarified. See RTKLIB/LICENSE.txt.
        Bugs and problems fixed including GitHub Issues:
          #461,#477,#480,#514,#540,#547,#555,#560.
    LIBRARY API:
        The following APIs added:
          code2freq(),sat2freq(),code2idx(),timereset(),setseleph(),getseleph(),
          decode_gal_fnav(),decode_irn_nav()
        The following APIs modified:
          obs2code(),code2obs(),setcodepri(),getcodepri(),tickget(),traceb(),
          getbitu(),getbits(),setbitu(),getbits(),rtk_crc32(),rtk_crc24q(),
          rtk_crc16(),docode_word(),decode_frame(),test_glostr(),decode_glostr(),
          decode_bds_d1(),decode_bds_d2(),decode_gal_inav(),input_raw(),input_oem4(),
          input_oem3(),input_ubx(),input_ss2(),input_cres(),input_stq(),gen_ubx(),
          gen_stq(),gen_nvs(),input_rtcm2(),input_rtcm3(),gen_rtcm3(),inputsol(),
          outprcopts(),outsolheads(),outsols(),outsolexs(),outnmea_rmc(),
          out_nmea_gga(),outnmea_gsa(),outnmea_gsv(),sbsdecodemsg(),strread(),
          strwrite(),strsvrstart(),strsvrstat(),showmsg()
        The following APIs deleted:
          lam_carr[],satwavelen(),csmooth(),satseleph(),input_gw10(),input_cmr(),
          input_tersus(),input_lexr(),input_gw10f(),input_cmrf(),input_tersusf(),
          input_lexrf(),gen_lexr(),strgetsel(),strsetsel(),strsetsrctbl(),
          pppcorr_read(),pppcorr_free(),pppcorr_trop().pppcorr_stec(),
          stsvrsetsrctbl(),init_imu(),input_imu(),lexupdatecorr(),lexreadmsg(),
          lexoutmsg(),lexconvbin(),lexeph2pos(),lexioncorr()
        The following types modified:
          obsd_t,eph_t,nav_t,rtcm_t,rnxctr_t,prcopt_t,rnxopt_t,ssat_t,raw_t,strsvr_t
        The following types deleted:
          lexmsg_t,lex_t,lexeph_t,lexion_t,stec_t,trop_t,pppcorr_t,exterr_t,
          half_cyc_t,imud_t,imu_t
    RECEIVER SUPPORTS:
        BINEX NavIC/IRNSS in raw observation data (0x7f-05) supported.
        BINEX IRNSS decoded ephemeris (0x01-07) supported.
        BINEX station info in site metadata (0x00) supported.
        NovAtel OEM7 supported including the following messages:
          RANGEB(43),RANGECMPB(140),RAWEPHEM(41),IONUTCB(8),RAWWAASFRAMEB(287),
          RAWSBASFRAMEB(973),GLOEPHEMERISB(723),GALEPHEMERISB(1122),GALIONOB(1127),
          GALCLOCKB(1121),QZSSRAWEPHEMB(1331),QZSSRAWSUBFRAMEB(1330),QZSSIONUTCB(1347),
          BDSEPHEMERISB(1696),NAVICEPHEMERISB(2123)
        Codes for Septentrio SBF re-written to support Mosaic-X5.
        Septentrio SBF supported including the following messages:
          MEAESPOCH(4027),GPSRAWCA(4017),GLORAWCA(4026),GALRAWFNAV(4022),
          GALRAWINAV(4023),GEORAWL1(4020),BDSRAW(4047),QZSRAWL1CA(4066),
          NAVICRAW(4093)
        u-blox UBX-CFG-VALDEL,VALGET,VALSET messages supported.
        u-blox UBX-RXM-RAWX half-cycle phase shift for BDS GEO satellites added.
        u-blox UBX-RXM-RAWX QZSS L1S supported.
    RTKPLOT:
        Leaflet and standard map tiles (OpenSteetMap etc.) supported in Map View.
        Initial loaded shape files supported.
        Residuals to elevation (Resid-EL) Plot added to solution plots.
        Positions files (.pos) supported by menu Open Waypoint.
        Sliderbar for solution animation widely expanded.
        Line and Fill colors supported in Map Layer dialog.
        TEQC no longer supported for observation data QC. Menu Obs Data QC deleted.
        GE (Google Earth) view, menus and button deleted.
    RTKCONV:
        RINEX 3.04 output supported.
        RINEX NAV BDSA/B and IRNA/B IONOS CORR output supported.
        RINEX NAV GLUT, GAUT, QZUT, BDUT and IRUT TIME SYS CORR output supported.
        RINEX NAV DT_LSF, WN_LSF and DN for LEAP SEC output supported.
        RINEX version check added to exclude unsupported systems and signals.
        Always two-pass processing. Option Scan Obs Types deleted.
        Option Phase Shift added to align carrier phases to refernece signals.
        Option GLONASS FCN added for receiver logs without FCN info like RTCM3 MSM4.
        Default receiver log time obtained from the time-tag file if it exists.
        Recursive new directory generation supported.
        High resolution (16bit) C/N0 suppored.
        Switch of reference stations supported in a RTCM3 log file.
        Format GW10, CMR/CMR+ and TERSUS for receiver logs no longer supported. 
        RINEX 2.12 QZS extension no longer supported.
        LEX log output no longer supported.
    STRSVR:
        Maximum 6 output streams supported.
        Input Log and Output Return Log supported.
        MT1131-7,1041 and 1230 supported for RTCM3 conversion.
        Galileo I/NAV and F/NAV are separately handled in RTCM3 conversion.
        Protocol HTTP/1.1 accepted by NTRIP caster mode.
        Button Get Mountp added in NTRIP Client Options dialog.
        Button Mountp Options added in NTRIP Caster Options dialog.
    RTKPOST:
        TGD and BGD handling improved for Galileo and BDS.
        Always L1-L2 (E1-E5b for Galileo, B1-B2 for BDS) used for Iono-Free LC. 
        Always I/NAV used for Galileo orbits and clocks.
        SP3-d format for precise ephemerides supported.
        CPU usage much improved in SD to DD conversion for ambiguity resolution.
        OpenBLAS linked instead of Intel MKL for fast-matrix computation.
        Option QZSS LEX and Input STEC for ionos-correction no longer supported.
        Option Input ZTD for troposphere correction no longer supported.
        AP RTKPOST_WIN64 and RTKPOST_MKL deleted.
    SRCTBLBROWS:
        Leaflet and OpenStreetMap used instead of Google Maps in Stream Map.
    RTKNAVI:
        NMEA talker ID GQ and GI (NMEA 0183 4.11) for QZSS and NavIC supported.
        NMEA GQ/GB/GI-GSA/GSV sentences supported.
        Option Panel Font added.
        Menus reorganized in RTK Monitor.
        Menu Station Info added to RTK Monitor.
        Satellite positions in Skyplot by TLE data no longer supported.
        Menu LEX and Iono Correction deleted from RTK Monitor.
        AP RTKNAVI_WIN64 deleted.
    RTKGET:
        Data source URL https://... and ftps://... supported.
        Wild-card (*) in file path of data source URL supported.
        Compressed RINEX files with extension .crx or .CRX supported.
    CONVBIN:
        Option -scan forced. Option -noscan deleted.
        Most of new features for RTKCONV involved but not completed.
    RNX2RTKP:
        Most of new features for RTKPOST involved but not completed.
    RTKRCV:
        Most of new features for RTKNAVI involved but not completed.
    STR2STR:
        Most of new features for STRSVR involved but not completed.
    RTKVIDEO and RTKVPLALER:
        APs deleted.
    DATA:
        TLE_GNSS_SATNO.txt, URL_LIST.txt and ant/igs14.atx updated.
    
    LIMITATIONS:
        QT ported APs are just moved to RTKLIB/app/qtapp but not maintained.
        Documents in RTKLIB/doc are not updated.