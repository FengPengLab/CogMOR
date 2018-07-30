#!/bin/bash

input_file="CogMOR3.tr"
flow_9_send="flow_9_send.txt"
flow_9_send_data="flow_9_send_data.txt"
flow_9_recv="flow_9_recv.txt"
flow_9_recv_data="flow_9_recv_data.txt"
flow_9_drop="flow_9_drop.txt"

flow_10_send="flow_10_send.txt"
flow_10_send_data="flow_10_send_data.txt"
flow_10_recv="flow_10_recv.txt"
flow_10_recv_data="flow_10_recv_data.txt"
flow_10_drop="flow_10_drop.txt"

flow_11_send="flow_11_send.txt"
flow_11_send_data="flow_11_send_data.txt"
flow_11_recv="flow_11_recv.txt"
flow_11_recv_data="flow_11_recv_data.txt"
flow_11_drop="flow_11_drop.txt"

flow_12_send="flow_12_send.txt"
flow_12_send_data="flow_12_send_data.txt"
flow_12_recv="flow_12_recv.txt"
flow_12_recv_data="flow_12_recv_data.txt"
flow_12_drop="flow_12_drop.txt"

matcher=""

cat /dev/null > ${flow_9_send}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 1 && $35 == 0){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_9_send}

cat /dev/null > ${flow_9_send_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 1 && $35 == 0 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_9_send_data}

cat /dev/null > ${flow_9_recv}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 0 && $33 == 1){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_9_recv}

cat /dev/null > ${flow_9_recv_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 0 && $33 == 1 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_9_recv_data}

cat /dev/null > ${flow_9_drop}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "d" && $19 == "MAC" && $21 == "COL" && $9 == 0 && $33 == 1){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_9_drop}



cat /dev/null > ${flow_10_send}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 2 && $35 == 0){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_10_send}

cat /dev/null > ${flow_10_send_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 2 && $35 == 0 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_10_send_data}

cat /dev/null > ${flow_10_recv}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 0 && $33 == 2){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_10_recv}

cat /dev/null > ${flow_10_recv_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 0 && $33 == 2 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_10_recv_data}

cat /dev/null > ${flow_10_drop}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "d" && $19 == "MAC" && $21 == "COL" && $9 == 0 && $33 == 2){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_10_drop}



cat /dev/null > ${flow_11_send}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 0 && $35 == 3){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_11_send}

cat /dev/null > ${flow_11_send_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 0 && $35 == 3 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_11_send_data}

cat /dev/null > ${flow_11_recv}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 3 && $33 == 0){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_11_recv}

cat /dev/null > ${flow_11_recv_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 3 && $33 == 0 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_11_recv_data}

cat /dev/null > ${flow_11_drop}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "d" && $19 == "MAC" && $21 == "COL" && $9 == 3 && $33 == 0){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_11_drop}



cat /dev/null > ${flow_12_send}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 0 && $35 == 4){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_12_send}

cat /dev/null > ${flow_12_send_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "s" && $19 == "MAC" && $9 == 0 && $35 == 4 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_12_send_data}

cat /dev/null > ${flow_12_recv}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 4 && $33 == 0){printf("%f,%s,%d,%d,%f,%f\n",$3,$25,$29,$31,$37,$39)}} ' >> ${flow_12_recv}

cat /dev/null > ${flow_12_recv_data}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "r" && $19 == "MAC" && $9 == 4 && $33 == 0 && $25 == "DATA"){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_12_recv_data}

cat /dev/null > ${flow_12_drop}

cat ${input_file} | awk -F "\"* \"*"  '{if($1 == "d" && $19 == "MAC" && $21 == "COL" && $9 == 4 && $33 == 0){printf("%f,%d,%d,%f,%f\n",$3,$29,$31,$37,$39)}} ' >> ${flow_12_drop}
