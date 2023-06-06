CREATE DATABASE IF NOT EXISTS gnss

clickhouse-client --query "CREATE TABLE gnss.obs
(
`Time` DateTime COMMENT '时间',
`Sat` Int32 COMMENT '卫星号',
`Rcv` Int32 COMMENT '接收机号',
`SNR` UInt16 COMMENT '信噪比',
`LLI` UInt8 COMMENT 'loss of lock indicator',
`Code` UInt8 COMMENT 'code indicator',
`L` Float64 COMMENT 'observation data carrier-phase',
`P` String COMMENT 'observation data pseudorange (m)',
`D` String COMMENT 'observation data doppler frequency (Hz)',
)"
